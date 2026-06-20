extern "C" {
  #include <user_interface.h>
}
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <espnow.h>

// --- STATUS LED CONFIGURATION ---
const int STATUS_LED_PIN = 2; // GPIO2 / Pin D4 (Onboard LED)
#define LED_ON LOW
#define LED_OFF HIGH

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

uint32_t lastPacketTime = 0;
uint32_t lastLedToggle = 0;
bool ledState = true;

// Session baseline variables
uint32_t sessionStartTime = 0;
bool isSessionActive = false;

void onDataReceive(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
    if (len == sizeof(incomingTelemetry)) {
        memcpy(&incomingTelemetry, incomingData, sizeof(incomingTelemetry));
        uint32_t currentTime = millis();
        lastPacketTime = currentTime; 
        
        // Locks the baseline once
        if (!isSessionActive) {
            sessionStartTime = currentTime;
            isSessionActive = true;
        }

        // Calculate exact relative elapsed time since tracking began
        uint32_t relativeSessionTime = currentTime - sessionStartTime;
        
        // CSV style data output
        Serial.print(relativeSessionTime); Serial.print(",");
        Serial.print(incomingTelemetry.state); Serial.print(",");
        Serial.print(incomingTelemetry.alt, 2); Serial.print(",");
        Serial.print(incomingTelemetry.maxAlt, 2); Serial.print(",");
        Serial.print(incomingTelemetry.accX, 2); Serial.print(",");
        Serial.print(incomingTelemetry.accY, 2); Serial.print(",");
        Serial.print(incomingTelemetry.accZ, 2); Serial.print(",");
        Serial.print(incomingTelemetry.pitch, 1); Serial.print(",");
        Serial.print(incomingTelemetry.roll, 1); Serial.print(",");
        Serial.print(incomingTelemetry.yaw, 1); Serial.print(",");
        Serial.println(incomingTelemetry.confidence);
    }
}
void setup() {
    Serial.begin(115200);

    // Initialize Status LED as ON to indicate power
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LED_ON);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(10);

    wifi_promiscuous_enable(1);
    wifi_set_channel(1);
    wifi_promiscuous_enable(0);
    delay(10);

    if (esp_now_init() != 0) { while(1); }
    
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(onDataReceive);
}

// status LED loop
void loop() {
    uint32_t now = millis();

    // Check if receiving data packets
    if (now - lastPacketTime < 1000) {
        // Active data streaming -> Blink
        if (now - lastLedToggle >= 500) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState ? LED_ON : LED_OFF);
        }
    } else {
        // No data stream present -> Stay ON
        digitalWrite(STATUS_LED_PIN, LED_ON);
    }
    
    delay(20); // Small yielding break for core background tasks
}