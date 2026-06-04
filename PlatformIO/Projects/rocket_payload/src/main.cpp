#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_BMI270_Arduino_Library.h>

Adafruit_BMP3XX bmp;
BMI270 bmi;

#define SEALEVELPRESSURE_HPA (1013.25)

// Tracking variables for acceleration math
uint32_t lastTime = 0;
float velocity_z = 0.0;
float imu_height = 0.0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; } 
    
    Serial.println("\n--- Upward Motion & Absolute Baro Test ---");

    Wire.begin(4, 5); // SDA = D2, SCL = D1
    Wire.setClock(100000); 

    if (!bmp.begin_I2C(0x77, &Wire)) {
        Serial.println("[-] BMP388 missing!");
        while(1);
    }
    
    // Standard high-rate rocket settings
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmi.beginI2C(0x68) != BMI2_OK) {
        Serial.println("[-] BMI270 missing!");
        while(1);
    }

    Serial.println("[+] Sensors initialized successfully.");
    Serial.println("\nTime(ms)\tAbs_Baro(m)\tUp_Accel(m/s^2)\tIMU_Height(m)");
    lastTime = millis();
}

void loop() {
    uint32_t currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    // 1. Absolute Barometer Reading (No calibration math)
    float baro_altitude = 0.0;
    if (bmp.performReading()) {
        baro_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    // 2. Read Accelerometer & Isolate Upward Motion
    float accel_z_ms2 = 0.0;
    if (bmi.getSensorData() == BMI2_OK) {
        // Convert to m/s^2 and remove gravity baseline
        accel_z_ms2 = (bmi.data.accelZ * 9.80665) - 9.80665;
    }

    // --- THE FIX: Upward-Only Filter & Noise Gate ---
    // If acceleration is negative (downward/tilt) or just tiny noise, clamp it to 0
    if (accel_z_ms2 < 0.15) {
        accel_z_ms2 = 0.0;
    }

    // 3. Integration Step
    if (accel_z_ms2 > 0.0) {
        // Active upward motion detected -> Calculate height
        velocity_z += accel_z_ms2 * dt;
        imu_height += velocity_z * dt;
    } else {
        // --- THE FIX: Leaky Integrator (Drift Bleed) ---
        // When no upward force is felt, bleed off velocity and height 
        // to force the math back to baseline instead of drifting away.
        velocity_z *= 0.85; 
        imu_height *= 0.92; 
        
        // Fully zero out the values if they get close enough to baseline
        if (imu_height < 0.05) imu_height = 0.0;
        if (velocity_z < 0.05) velocity_z = 0.0;
    }

    // 4. Print results
    Serial.print(currentTime);
    Serial.print("\t\t");
    Serial.print(baro_altitude, 2);
    Serial.print(" m\t\t");
    Serial.print(accel_z_ms2, 2);
    Serial.print("\t\t\t");
    Serial.print(imu_height, 2);
    Serial.println(" m");

    delay(50); 
}