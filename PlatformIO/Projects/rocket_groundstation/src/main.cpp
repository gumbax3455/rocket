// Add this at the absolute top of the Ground Station file (Line 1)
extern "C" {
  #include <user_interface.h>
}
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <espnow.h>

// Packed telemetry structure MUST match payload byte-for-byte
struct __attribute__((packed)) TelemetryPacket {
    uint8_t state;
    float alt;
    float maxAlt;
    float accX;
    float accY;
    float accZ;
    float pitch;
    float roll;
    float yaw;
    uint8_t confidence;
};
TelemetryPacket incomingTelemetry;

// Callback function executed automatically when a radio packet arrives
void onDataReceive(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
    if (len == sizeof(incomingTelemetry)) {
        memcpy(&incomingTelemetry, incomingData, sizeof(incomingTelemetry));
        
        Serial.print("State: ");
        if(incomingTelemetry.state == 0) Serial.print("PAD    | ");
        if(incomingTelemetry.state == 1) Serial.print("ASCENT | ");
        if(incomingTelemetry.state == 2) Serial.print("DEPLOY | ");

        Serial.print("Alt: "); Serial.print(incomingTelemetry.alt, 2); Serial.print("m | ");
        Serial.print("Max: "); Serial.print(incomingTelemetry.maxAlt, 2); Serial.print("m | ");
        
        Serial.print("Acc XYZ: "); 
        Serial.print(incomingTelemetry.accX, 1); Serial.print(","); 
        Serial.print(incomingTelemetry.accY, 1); Serial.print(","); 
        Serial.print(incomingTelemetry.accZ, 1); Serial.print(" m/s² | ");
        
        Serial.print("Ori: P:"); Serial.print(incomingTelemetry.pitch, 0); 
        Serial.print(" R:"); Serial.print(incomingTelemetry.roll, 0); 
        Serial.print(" Y:"); Serial.print(incomingTelemetry.yaw, 0); Serial.print("° | ");
        
        if (incomingTelemetry.state == 2) {
            Serial.println("Deploy: [!!! APOGEE MET !!!]");
        } else {
            Serial.print("Confidence: "); Serial.print(incomingTelemetry.confidence); Serial.println("%");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=============================================");
    Serial.println("     GROUND STATION RECEIVER ONLINE         ");
    Serial.println("=============================================");

    // --- UPDATED WIFI / ESP-NOW CONFIGURATION ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(10);

    // Force the physical radio hardware onto Channel 1
    wifi_promiscuous_enable(1);
    wifi_set_channel(1);
    wifi_promiscuous_enable(0);
    delay(10);

    Serial.print("[INFO] Ground Station MAC Address is: ");
    Serial.println(WiFi.macAddress()); 
    Serial.println("=============================================");

    if (esp_now_init() != 0) {
        Serial.println("[!] ESP-NOW Ground Station Initialization Failed");
        while(1);
    }
    
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(onDataReceive);
    
    Serial.println("[+] Listening for Rocket telemetry... Standby.");
}

void loop() {
    delay(100);
}