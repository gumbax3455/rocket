#include "Arduino.h"
#include "LittleFS.h"

void setup() {
    Serial.begin(115200);
    delay(9000);
    
    Serial.println("\n=== RETRIEVING STORED FLIGHT DATA ===");

    if (!LittleFS.begin()) {
        Serial.println("Error: Could not mount flash storage file system.");
        return;
    }

    // Open the file in read ("r") mode
    File flightFile = LittleFS.open("/flight_log.csv", "r");
    
    if (!flightFile) {
        Serial.println("Error: No flight log file found on chip.");
        return;
    }

    // Stream every character from the chip's internal storage straight to your VSCode window
    while (flightFile.available()) {
        Serial.write(flightFile.read());
    }

    flightFile.close();
    Serial.println("=== END OF LOG ===");
}

void loop() {
    // Nothing to do here
}