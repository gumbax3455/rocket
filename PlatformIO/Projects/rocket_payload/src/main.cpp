#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "ESP8266WiFi.h"
#include "LittleFS.h" 

Adafruit_BMP3XX bmp;
BMI270 bmi;
File flightFile;

#define SEALEVELPRESSURE_HPA (1013.25)
const float FILTER_WEIGHT = 0.98;  
float pitch = 0.0, roll = 0.0, yaw = 0.0;
uint32_t lastFusionTime = 0;
uint32_t lastSampleTime = 0;
uint32_t lastPrintTime = 0;

void setup() {
    // 1. Start Serial immediately so you can see what is happening!
    Serial.begin(115200);
    delay(9000); 
    Serial.println("\n=== BLACKBOX LOGGER INITIALIZATION ===");

    // 2. Force Wi-Fi radio off (Saves ~60mA)
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(1);
    Serial.println("[+] Wi-Fi Radio: DISABLED (Power Saver Active)");

    // 3. Initialize I2C Hardware Bus
    Wire.begin(4, 5); 
    Wire.setClock(400000); 

    if (bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("[+] Barometer: ONLINE");
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    } else {
        Serial.println("[!] Barometer: FAILED");
        while(1);
    }

    if (bmi.beginI2C(0x68) == BMI2_OK) {
        Serial.println("[+] IMU (BMI270): ONLINE");
    } else {
        Serial.println("[!] IMU (BMI270): FAILED");
        while(1);
    }

    // 4. Initialize Internal Storage File System
    Serial.print("[...] Mounting Flash Storage... ");
    if (!LittleFS.begin()) {
        // If it fails, the storage is likely unformatted. Format it now.
        Serial.println("Unformatted! Formatting storage area now (Takes ~10 seconds)...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("[!] Flash Storage: CRITICAL ERROR");
            while(1);
        }
    }
    Serial.println("READY!");

    // Create / wipe the old flight log file and write headers
    flightFile = LittleFS.open("/flight_log.csv", "w");
    if (flightFile) {
        flightFile.println("TimeMS,Pitch,Roll,Yaw,ThrustZ,Altitude");
        flightFile.close();
        Serial.println("[+] New log file '/flight_log.csv' created successfully.");
    } else {
        Serial.println("[!] Error creating flight log file!");
    }

    Serial.println("=== SYSTEM RUNNING: LOGGING SENSORS AT 50HZ ===\n");
    lastSampleTime = millis();
    lastFusionTime = millis();
    lastPrintTime = millis();
}

void loop() {
    uint32_t now = millis();

    // High-speed logging loop (50Hz / every 20ms)
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

            // Append data straight into the silicon flash storage
            flightFile = LittleFS.open("/flight_log.csv", "a");
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

    // Visual heartbeat indicator: Blips the terminal every 2 seconds 
    // to prove the chip hasn't crashed, without flooding the screen.
    if (now - lastPrintTime >= 2000) {
        lastPrintTime = now;
        Serial.print("[Active] Internal flight clock: ");
        Serial.print(now / 1000.0, 1);
        Serial.println("s | Writing to Flash...");
    }
}