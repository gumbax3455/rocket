#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_BMP3XX.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Servo.h>

Adafruit_BMP3XX bmp;
BMI270 bmi;
Servo deploymentServo;

const int SERVO_PIN = 14;          // Pin D5 (GPIO 14)
#define SEALEVELPRESSURE_HPA (1013.25)

const int SERVO_LOCKED_ANGLE = 0;    
const int SERVO_DEPLOY_ANGLE = 120;  

enum FlightState { GROUND_PAD, ASCENT, APOGEE_MET };
FlightState current_state = GROUND_PAD;

float groundAltitude = 0.0;
float maxAltitude = -999.0;
float currentAltitude = 0.0; 
int descentSampleCount = 0;

const float FILTER_WEIGHT = 0.98;  
float pitch = 0.0, roll = 0.0, yaw = 0.0;

uint32_t lastSampleTime = 0;
uint32_t lastFusionTime = 0;
uint32_t lastPrintTime = 0;

const float LAUNCH_THRESHOLD_METERS = 0.5;   
const float DESCENT_THRESHOLD_METERS = 0.8;  // !!!!!!
const int REQUIRED_DESCENT_SAMPLES = 5;      
const float TILT_THRESHOLD_DEGREES = 65.0;   

void setup() {
    Serial.begin(115200);
    delay(8000);
    Serial.println("\n=== TERMINAL STANDALONE FLIGHT COMPUTER REVISION ===");

    deploymentServo.attach(SERVO_PIN, 800, 2600);
    deploymentServo.write(SERVO_LOCKED_ANGLE);

    Wire.begin(4, 5); 
    Wire.setClock(400000); 

    if (!bmp.begin_I2C(0x77, &Wire)) { Serial.println("[!] BMP390 Error"); while(1); }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmi.beginI2C(0x68) != BMI2_OK) { Serial.println("[!] BMI270 Error"); while(1); }

    float altSum = 0;
    for (int i = 0; i < 20; i++) {
        if (bmp.performReading()) { altSum += bmp.readAltitude(SEALEVELPRESSURE_HPA); }
        delay(100);
    }
    groundAltitude = altSum / 20.0;
    currentAltitude = groundAltitude;
    maxAltitude = groundAltitude;

    Serial.println("[+] System Calibrated and Armed.");
}

void loop() {
    uint32_t now = millis();

    if (Serial.available() > 0) {
        String inputCommand = Serial.readStringUntil('\n');
        inputCommand.trim();
        if (inputCommand.equalsIgnoreCase("LAUNCH") && current_state == GROUND_PAD) {
            current_state = ASCENT;
            Serial.println("\n[!] MANUAL OVERRIDE: Entering ASCENT mode.");
        }
    }

    if (now - lastSampleTime >= 20) {
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

            float accelPitch = atan2(bmi.data.accelY, bmi.data.accelZ) * 180.0 / PI;
            float accelRoll  = atan2(-bmi.data.accelX, sqrt(bmi.data.accelY * bmi.data.accelY + bmi.data.accelZ * bmi.data.accelZ)) * 180.0 / PI;

            pitch = (FILTER_WEIGHT * (pitch + (gx * dt))) + ((1.0 - FILTER_WEIGHT) * accelPitch);
            roll  = (FILTER_WEIGHT * (roll  + (gy * dt))) + ((1.0 - FILTER_WEIGHT) * accelRoll);
            yaw   += gz * dt;

            switch (current_state) {
                case GROUND_PAD:
                    maxAltitude = currentAltitude; 
                    if (currentAltitude > (groundAltitude + LAUNCH_THRESHOLD_METERS)) {
                        current_state = ASCENT;
                    }
                    break;

                case ASCENT: { 
                    if (currentAltitude > maxAltitude) {
                        maxAltitude = currentAltitude;
                        descentSampleCount = 0; 
                    } 
                    else if (currentAltitude < (maxAltitude - DESCENT_THRESHOLD_METERS)) {
                        descentSampleCount++;
                    } else {
                        if (descentSampleCount > 0) descentSampleCount--;
                    }

                    bool criticalTilt = (abs(pitch) > TILT_THRESHOLD_DEGREES || abs(roll) > TILT_THRESHOLD_DEGREES);

                    if (descentSampleCount >= REQUIRED_DESCENT_SAMPLES || criticalTilt) {
                        current_state = APOGEE_MET;
                        deploymentServo.write(SERVO_DEPLOY_ANGLE); 
                    }
                    break;
                } 
                case APOGEE_MET:
                    break;
            }

            if (now - lastPrintTime >= 200) {
                lastPrintTime = now;
                
                Serial.print("State: ");
                if(current_state == GROUND_PAD) Serial.print("PAD    | ");
                if(current_state == ASCENT)     Serial.print("ASCENT | ");
                if(current_state == APOGEE_MET) Serial.print("DEPLOY | ");

                Serial.print("Alt: "); Serial.print(currentAltitude - groundAltitude, 2); Serial.print("m | ");
                Serial.print("Max: "); Serial.print(maxAltitude - groundAltitude, 2); Serial.print("m | ");
                Serial.print("Acc: "); Serial.print(ax,1); Serial.print(","); Serial.print(ay,1); Serial.print(","); Serial.print(az,1); Serial.print(" m/s² | ");
                Serial.print("Ori: P:"); Serial.print(pitch,0); Serial.print(" R:"); Serial.print(roll,0); Serial.print(" Y:"); Serial.print(yaw,0); Serial.print(" | ");
                
                int confidence = (descentSampleCount * 100) / REQUIRED_DESCENT_SAMPLES;
                if (current_state == APOGEE_MET) {
                    Serial.println("Deploy: 100% -> [DEPLOYED]");
                } else {
                    Serial.print("Confidence: "); Serial.print(confidence); Serial.println("%");
                }
            }
        }
    }
}