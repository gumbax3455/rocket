#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <math.h>

Adafruit_BMP3XX bmp;
BMI270 bmi;

#define SEALEVELPRESSURE_HPA (1013.25)

// ==========================================
// SENSOR FUSION SETTINGS
// ==========================================
const float FILTER_WEIGHT = 0.98;  // 98% Gyro (fast/smooth), 2% Accel (anti-drift anchor)
const float NOISE_GATE = 0.15;     
const float VEL_BLEED  = 0.96;     
const float ALT_BLEED  = 0.985;    

// Attitude Tracking (Pitch & Roll in degrees)
float pitch = 0.0;
float roll = 0.0;

// Integration tracking
uint32_t lastTime = 0;
float velocity_z = 0.0;
float imu_height = 0.0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; } 
    
    Serial.println("\n=========================================================================");
    Serial.println("               6-DoF SENSOR FUSION TELEMETRY SYSTEM                      ");
    Serial.println("=========================================================================");

    Wire.begin(4, 5); 
    Wire.setClock(400000); // Bumped to 400kHz Fast I2C to handle 6-DoF math smoothly

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

    Serial.println("[+] Barometer, Accelerometer, and Gyroscope Online.");
    Serial.println("-------------------------------------------------------------------------");
    Serial.println("Time\tPitch\tRoll\tNet_Z(Thrust)\tIMU_Alt\t\tBaro_Alt");
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

    // 2. Read 6-DoF IMU Data
    float net_thrust_Z = 0.0;
    
    if (bmi.getSensorData() == BMI2_OK) {
        // Raw Accelerations in G's
        float ax = bmi.data.accelX;
        float ay = bmi.data.accelY;
        float az = bmi.data.accelZ;

        // Raw Gyro Rotations in Degrees Per Second (DPS)
        float gx = bmi.data.gyroX;
        float gy = bmi.data.gyroY;

        // --- THE COMPLEMENTARY FILTER ---
        // Step A: Calculate absolute angle from Accelerometer
        float accelPitch = atan2(ay, az) * 180.0 / PI;
        float accelRoll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

        // Step B: Integrate Gyroscope & Fuse with Accelerometer
        pitch = (FILTER_WEIGHT * (pitch + (gx * dt))) + ((1.0 - FILTER_WEIGHT) * accelPitch);
        roll  = (FILTER_WEIGHT * (roll  + (gy * dt))) + ((1.0 - FILTER_WEIGHT) * accelRoll);

        // --- DYNAMIC GRAVITY REMOVAL ---
        // Now that we know our exact tilt, calculate how much of the 9.81m/s^2 
        // gravity vector is currently resting on the Z-axis.
        float pitchRad = pitch * PI / 180.0;
        float rollRad  = roll  * PI / 180.0;
        
        float expected_gravity_Z = 9.80665 * cos(pitchRad) * cos(rollRad);
        
        // Subtract only the expected gravity from the total Z acceleration
        net_thrust_Z = (az * 9.80665) - expected_gravity_Z;
    }

    // 3. IMU Height Integration (Pure vertical thrust only)
    if (net_thrust_Z > NOISE_GATE) {
        velocity_z += net_thrust_Z * dt;
        imu_height += velocity_z * dt;
    } else {
        velocity_z *= VEL_BLEED; 
        imu_height *= ALT_BLEED; 
        if (imu_height < 0.02) imu_height = 0.0;
        if (velocity_z < 0.02) velocity_z = 0.0;
    }

    // 4. Print Output
    Serial.print(currentTime);
    Serial.print("\t");
    
    Serial.print(pitch, 1);
    Serial.print("*\t");
    
    Serial.print(roll, 1);
    Serial.print("*\t");
    
    Serial.print(net_thrust_Z > NOISE_GATE ? net_thrust_Z : 0.0, 2);
    Serial.print(" m/s^2\t");
    
    Serial.print(imu_height, 2);
    Serial.print(" m\t\t");
    
    Serial.print(baro_altitude, 2);
    Serial.println(" m");

    delay(50); 
}