#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_BMI270_Arduino_Library.h>

Adafruit_BMP3XX bmp;
BMI270 bmi;

#define SEALEVELPRESSURE_HPA (1013.25)

// ==========================================
// CALIBRATION & TUNING SETTINGS
// ==========================================
const uint32_t CALIBRATION_TIME_MS = 10000; // 10 seconds
const float NOISE_GATE = 0.15;              // Gate for movement sensitivity
const float VEL_BLEED  = 0.96;              // Velocity retention
const float ALT_BLEED  = 0.985;             // Height retention

// Offset tracking
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;

// Integration tracking
uint32_t lastTime = 0;
float velocity_3d = 0.0;
float imu_height = 0.0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; } 
    
    Serial.println("\n=========================================================================");
    Serial.println("           3D MAGNITUDE FLIGHT TELEMETRY (AUTO-CALIBRATING)              ");
    Serial.println("=========================================================================");

    Wire.begin(4, 5); 
    Wire.setClock(100000); 

    if (!bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("[-] BMP388 missing!");
        while(1);
    }
    
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmi.beginI2C(0x68) != BMI2_OK) {
        Serial.println("[-] BMI270 missing!");
        while(1);
    }

    // STARTUP CALIBRATION ROUTINE
    Serial.printf("[+] Calibrating IMU for %d seconds. DO NOT MOVE SENSOR...\n", CALIBRATION_TIME_MS / 1000);
    
    uint32_t calibStart = millis();
    long samples = 0;
    float sumX = 0, sumY = 0, sumZ = 0;

    // Loop until the calibration time runs out
    while (millis() - calibStart < CALIBRATION_TIME_MS) {
        if (bmi.getSensorData() == BMI2_OK) {
            sumX += bmi.data.accelX * 9.80665;
            sumY += bmi.data.accelY * 9.80665;
            sumZ += bmi.data.accelZ * 9.80665;
            samples++;
        }
        delay(10); // Feed the watchdog timer so the ESP doesn't crash
    }

    // Calculate the exact resting averages
    offsetX = sumX / samples;
    offsetY = sumY / samples;
    offsetZ = sumZ / samples;

    Serial.printf("[+] Calibration Complete! (%ld samples taken)\n", samples);
    Serial.printf("    Offsets -> X: %.2f | Y: %.2f | Z: %.2f\n", offsetX, offsetY, offsetZ);
    Serial.println("-------------------------------------------------------------------------");
    Serial.println("Time(ms)\tBaro_Alt(m)\tNet_Thrust(m/s^2)\tIMU_Alt(m)");
    Serial.println("-------------------------------------------------------------------------");
    
    lastTime = millis();
}

void loop() {
    uint32_t currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    // 1. Read Barometer
    float baro_altitude = 0.0;
    if (bmp.performReading()) {
        baro_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    // 2. Read Accelerometer & Apply 3D Magnitude Math
    float net_thrust_ms2 = 0.0;
    
    if (bmi.getSensorData() == BMI2_OK) {
        // Convert to m/s^2
        float x = bmi.data.accelX * 9.80665;
        float y = bmi.data.accelY * 9.80665;
        float z = bmi.data.accelZ * 9.80665;

        // Calculate the total 3D magnitude
        float total_magnitude = sqrt((x * x) + (y * y) + (z * z));
        
        // Calculate the calibrated resting magnitude
        float resting_magnitude = sqrt((offsetX * offsetX) + (offsetY * offsetY) + (offsetZ * offsetZ));

        // Subtract the resting gravity to find ONLY the new thrust
        net_thrust_ms2 = total_magnitude - resting_magnitude;
    }

    // 3. IMU Height Integration (Using Net Thrust)
    if (net_thrust_ms2 > NOISE_GATE) {
        velocity_3d += net_thrust_ms2 * dt;
        imu_height += velocity_3d * dt;
    } else {
        velocity_3d *= VEL_BLEED; 
        imu_height *= ALT_BLEED; 
        if (imu_height < 0.02) imu_height = 0.0;
        if (velocity_3d < 0.02) velocity_3d = 0.0;
    }

    // 4. Print Output
    Serial.print(currentTime);
    Serial.print("\t\t");
    Serial.print(baro_altitude, 2);
    Serial.print("\t\t");
    Serial.print(net_thrust_ms2 > NOISE_GATE ? net_thrust_ms2 : 0.0, 2);
    Serial.print("\t\t\t");
    Serial.println(imu_height, 2);

    delay(50); 
}