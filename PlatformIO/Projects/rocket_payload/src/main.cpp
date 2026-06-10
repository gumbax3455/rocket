#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "ESP8266WiFi.h"
#include "LittleFS.h" 

Adafruit_BMP3XX bmp;
BMI270 bmi;
File flightFile;

String activeFilename = ""; // Tracks our dynamically generated filename
#define SEALEVELPRESSURE_HPA (1013.25)
const float FILTER_WEIGHT = 0.98;  
float pitch = 0.0, roll = 0.0, yaw = 0.0;
uint32_t lastFusionTime = 0;
uint32_t lastSampleTime = 0;
uint32_t lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    delay(5000); 
    Serial.println("\n=== BLACKBOX LOGGER INITIALIZATION ===");

    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);

    Wire.begin(4, 5); 
    Wire.setClock(400000); 

    if (!bmp.begin_I2C(0x77, &Wire)) { while(1); }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmi.beginI2C(0x68) != BMI2_OK) { while(1); }

    Serial.print("[...] Mounting Flash Storage... ");
    if (!LittleFS.begin()) {
        LittleFS.format();
        LittleFS.begin();
    }
    Serial.println("READY!");

    // SMART FILE ROLLING: Find the next available file number
    int fileIndex = 0;
    while (fileIndex < 100) {
        activeFilename = "/flight_" + String(fileIndex) + ".csv";
        if (!LittleFS.exists(activeFilename)) {
            // Found a filename that doesn't exist yet! Break and use it.
            break;
        }
        fileIndex++;
    }

    // Initialize the new file safely without touching older logs
    flightFile = LittleFS.open(activeFilename, "w");
    if (flightFile) {
        flightFile.println("TimeMS,Pitch,Roll,Yaw,ThrustZ,Altitude");
        flightFile.close();
        Serial.print("[+] Created fresh log: ");
        Serial.println(activeFilename);
    }

    lastSampleTime = millis();
    lastFusionTime = millis();
    lastPrintTime = millis();
}

void loop() {
    uint32_t now = millis();

    if (now - lastSampleTime >= 20) {
        float dt = (now - lastFusionTime) / 1000.0;
        lastFusionTime = now;
        lastSampleTime = now;

        float baro_altitude = 0.0;
        if (bmp.performReading()) {
            baro_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        }

        if (bmi.getSensorData() == BMI2_OK) {
            float az_ms2 = bmi.data.accelZ * 9.80665;
            float gx = bmi.data.gyroX;
            float gy = bmi.data.gyroY;
            float gz = bmi.data.gyroZ;

            float accelPitch = atan2(bmi.data.accelY, bmi.data.accelZ) * 180.0 / PI;
            float accelRoll  = atan2(-bmi.data.accelX, sqrt(bmi.data.accelY * bmi.data.accelY + bmi.data.accelZ * bmi.data.accelZ)) * 180.0 / PI;

            pitch = (FILTER_WEIGHT * (pitch + (gx * dt))) + ((1.0 - FILTER_WEIGHT) * accelPitch);
            roll  = (FILTER_WEIGHT * (roll  + (gy * dt))) + ((1.0 - FILTER_WEIGHT) * accelRoll);
            yaw   += gz * dt; 

            float expected_gravity_Z = 9.80665 * cos(pitch * PI / 180.0) * cos(roll * PI / 180.0);
            float net_thrust_Z = az_ms2 - expected_gravity_Z;

            // Append data into our uniquely indexed active file
            flightFile = LittleFS.open(activeFilename, "a");
            if (flightFile) {
                flightFile.print(now); flightFile.print(",");
                flightFile.print(pitch, 1); flightFile.print(",");
                flightFile.print(roll, 1); flightFile.print(",");
                flightFile.print(yaw, 1); flightFile.print(",");
                flightFile.print(net_thrust_Z > 0.15 ? net_thrust_Z : 0.0, 2); flightFile.print(",");
                flightFile.println(baro_altitude, 1);
                flightFile.close(); 
            }
        }
    }

    if (now - lastPrintTime >= 2000) {
        lastPrintTime = now;
        Serial.print("[Logging to "); Serial.print(activeFilename);
        Serial.print("] Live Clock: "); Serial.print(now / 1000.0, 1); Serial.println("s");
    }
}