extern "C" {
  #include <user_interface.h>
}
#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

Adafruit_BMP3XX bmp;
BMI270 bmi;
Servo deploymentServo;

const int SERVO_PIN = 14; 
#define SEALEVELPRESSURE_HPA (1013.25)
const int SERVO_LOCKED_ANGLE = 0;    
const int SERVO_DEPLOY_ANGLE = 120;  

const int STATUS_LED_PIN = 2; 
#define LED_ON LOW
#define LED_OFF HIGH

enum FlightState { GROUND_PAD, ASCENT, APOGEE_MET };
FlightState current_state = GROUND_PAD;

uint8_t groundStationMac[] = {0x5C, 0xCF, 0x7F, 0xD3, 0x4F, 0x21};

// --- COMPRESSED TELEMETRY STRUCTURE
struct __attribute__((packed)) TelemetrySample {
    uint8_t state;
    uint8_t confidence;
    float alt;
    int16_t accX_scaled;
    int16_t accY_scaled;
    int16_t accZ_scaled;
    int16_t pitch_scaled;
    int16_t roll_scaled;
    int16_t yaw_scaled;
};

// Array buffer to hold 10 samples (18 bytes * 10 = 180 bytes total packet)
TelemetrySample dataBundle[10];
int sampleIndex = 0;

float groundAltitude = 0.0;
float maxAltitude = -999.0;
float currentAltitude = 0.0; 
int descentSampleCount = 0;

const float FILTER_WEIGHT = 0.98;  
float pitch = 0.0, roll = 0.0, yaw = 0.0;

uint32_t lastSampleTime = 0;
uint32_t lastFusionTime = 0;
uint32_t lastLedToggle = 0;
bool ledState = true;

const float LAUNCH_ACCEL_THRESHOLD = 16.0;  
const float DESCENT_THRESHOLD_METERS = 0.4;  
const int REQUIRED_DESCENT_SAMPLES = 5;      
const float TILT_THRESHOLD_DEGREES = 65.0;   

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== INITIALIZING 100Hz BUNDLING FIRMWARE ===");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LED_ON);

    deploymentServo.attach(SERVO_PIN, 800, 2600);
    deploymentServo.write(SERVO_LOCKED_ANGLE);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(10);

    wifi_promiscuous_enable(1);
    wifi_set_channel(1);
    wifi_promiscuous_enable(0);
    delay(10);

    if (esp_now_init() != 0) { while(1); }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(groundStationMac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    Wire.begin(4, 5); 
    Wire.setClock(400000); 

    if (!bmp.begin_I2C(0x77, &Wire)) { while(1); }
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING); 
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);    
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ); 

    if (bmi.beginI2C(0x68) != BMI2_OK) { while(1); }

    float altSum = 0;
    for (int i = 0; i < 20; i++) {
        if (bmp.performReading()) { altSum += bmp.readAltitude(SEALEVELPRESSURE_HPA); }
        delay(50);
    }
    groundAltitude = altSum / 20.0;
    currentAltitude = groundAltitude;
    maxAltitude = groundAltitude;

    Serial.println("[+] Armed. Closing Serial pipeline for flight...");
    delay(500);
    Serial.end(); 
}

void loop() {
    uint32_t now = millis();

    // STRICT 10MS TIME WINDOW TICKER (100 Hz Sampling Rate)
    if (now - lastSampleTime >= 10) {
        float dt = (now - lastFusionTime) / 1000.0;
        lastFusionTime = now;
        lastSampleTime = now;

        if (bmp.performReading()) {
            currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        }

        if (bmi.getSensorData() == BMI2_OK) {
            float ax = bmi.data.accelX * 9.80665;
            float ay = bmi.data.accelY * 9.80665;
            float az = bmi.data.accelZ * 9.80665;
            float gx = bmi.data.gyroX;
            float gy = bmi.data.gyroY;
            float gz = bmi.data.gyroZ;

            float totalAcceleration = sqrt(ax*ax + ay*ay + az*az);
            float accelPitch = atan2(bmi.data.accelY, bmi.data.accelZ) * 180.0 / PI;
            float accelRoll  = atan2(-bmi.data.accelX, sqrt(bmi.data.accelY * bmi.data.accelY + bmi.data.accelZ * bmi.data.accelZ)) * 180.0 / PI;

            pitch = (FILTER_WEIGHT * (pitch + (gx * dt))) + ((1.0 - FILTER_WEIGHT) * accelPitch);
            roll  = (FILTER_WEIGHT * (roll  + (gy * dt))) + ((1.0 - FILTER_WEIGHT) * accelRoll);
            yaw   += gz * dt;

            switch (current_state) {
                case GROUND_PAD:
                    maxAltitude = currentAltitude; 
                    if (totalAcceleration > LAUNCH_ACCEL_THRESHOLD) current_state = ASCENT;
                    break;

                case ASCENT:
                    if (currentAltitude > maxAltitude) {
                        maxAltitude = currentAltitude;
                        descentSampleCount = 0; 
                    } 
                    else if (currentAltitude < (maxAltitude - DESCENT_THRESHOLD_METERS)) {
                        descentSampleCount++;
                    } else {
                        if (descentSampleCount > 0) descentSampleCount--;
                    }

                    if (descentSampleCount >= REQUIRED_DESCENT_SAMPLES || abs(pitch) > TILT_THRESHOLD_DEGREES || abs(roll) > TILT_THRESHOLD_DEGREES) {
                        current_state = APOGEE_MET;
                        deploymentServo.write(SERVO_DEPLOY_ANGLE); 
                    }
                    break;
                case APOGEE_MET:
                    break;
            }

            // --- PACK INTO ARRAYS ---
            dataBundle[sampleIndex].state = (uint8_t)current_state;
            dataBundle[sampleIndex].confidence = (current_state == APOGEE_MET) ? 100 : (descentSampleCount * 100) / REQUIRED_DESCENT_SAMPLES;
            dataBundle[sampleIndex].alt = currentAltitude - groundAltitude;
            
            // Scaled compression conversion
            dataBundle[sampleIndex].accX_scaled = (int16_t)(ax * 100.0);
            dataBundle[sampleIndex].accY_scaled = (int16_t)(ay * 100.0);
            dataBundle[sampleIndex].accZ_scaled = (int16_t)(az * 100.0);
            dataBundle[sampleIndex].pitch_scaled = (int16_t)(pitch * 10.0);
            dataBundle[sampleIndex].roll_scaled = (int16_t)(roll * 10.0);
            dataBundle[sampleIndex].yaw_scaled = (int16_t)(yaw * 10.0);

            sampleIndex++;

            // --- TRANSMIT BUNDLE EVERY 100MS ---
            if (sampleIndex >= 10) {
                esp_now_send(groundStationMac, (uint8_t *) &dataBundle, sizeof(dataBundle));
                sampleIndex = 0; // Clear buffer index layout
            }
        }

        // Heartbeat Blink Handler
        if (now - lastLedToggle >= 500) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState ? LED_ON : LED_OFF);
        }
    }
}