#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "ESP8266WiFi.h"
#include "espnow.h"

Adafruit_BMP3XX bmp;
BMI270 bmi;

#define SEALEVELPRESSURE_HPA (1013.25)

// REPLACE THESE HEX VALUES WITH YOUR ESP2 MAC ADDRESS FOUND IN YOUR TEST
uint8_t groundEspMac[] = {0x5E, 0xCF, 0x7F, 0xD3, 0x4F, 0x21};

// Sensor Fusion Globals
const float FILTER_WEIGHT = 0.98;  
float pitch = 0.0, roll = 0.0;
uint32_t lastTime = 0;

// Explicitly defined data structure pack
struct TelemetryPacket {
    uint32_t timestamp;
    float pitch;
    float roll;
    float net_z;
    float baro_alt;
};
TelemetryPacket packet;

void setup() {
    Serial.begin(115200);
    Wire.begin(4, 5); 
    Wire.setClock(400000); 

    if (!bmp.begin_I2C(0x77, &Wire)) { 
        Serial.println("[-] BMP388 Check Failed");
        while(1); 
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmi.beginI2C(0x68) != BMI2_OK) { 
        Serial.println("[-] BMI270 Check Failed");
        while(1); 
    }

    // Force WiFi radio to Station Mode and disconnect from home networks
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != 0) {
        Serial.println("[-] ESP-NOW Link Failed");
        while(1);
    }

    // Assign communication roles
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(groundEspMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    
    Serial.println("[+] Rocket Transmitter Online.");
    lastTime = millis();
}

void loop() {
    uint32_t currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    float baro_altitude = 0.0;
    if (bmp.performReading()) {
        baro_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    float net_thrust_Z = 0.0;
    if (bmi.getSensorData() == BMI2_OK) {
        float ax = bmi.data.accelX, ay = bmi.data.accelY, az = bmi.data.accelZ;
        float gx = bmi.data.gyroX, gy = bmi.data.gyroY;

        float accelPitch = atan2(ay, az) * 180.0 / PI;
        float accelRoll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

        pitch = (FILTER_WEIGHT * (pitch + (gx * dt))) + ((1.0 - FILTER_WEIGHT) * accelPitch);
        roll  = (FILTER_WEIGHT * (roll  + (gy * dt))) + ((1.0 - FILTER_WEIGHT) * accelRoll);

        float expected_gravity_Z = 9.80665 * cos(pitch * PI / 180.0) * cos(roll * PI / 180.0);
        net_thrust_Z = (az * 9.80665) - expected_gravity_Z;
    }

    // Map structural data fields
    packet.timestamp = currentTime;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.net_z = net_thrust_Z > 0.15 ? net_thrust_Z : 0.0;
    packet.baro_alt = baro_altitude;

    // Transmit structural bundle over the air to ground-esp
    esp_now_send(groundEspMac, (uint8_t *) &packet, sizeof(packet));

    delay(100); 
}
