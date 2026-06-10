#include "Arduino.h"
#include "LittleFS.h"

void setup() {
    Serial.begin(115200);
    delay(5000);
    
    Serial.println("\n=============================================");
    Serial.println("       INTERNAL STORAGE STORAGE DUMP         ");
    Serial.println("=============================================");

    if (!LittleFS.begin()) {
        Serial.println("Error: Could not mount flash storage file system.");
        return;
    }

    // Open the root directory to locate all stored logs
    Dir dir = LittleFS.openDir("/");
    int fileCount = 0;

    while (dir.next()) {
        String fileName = dir.fileName();
        size_t fileSize = dir.fileSize();
        
        Serial.println("\n---------------------------------------------");
        Serial.print("FOUND FILE: "); Serial.print(fileName);
        Serial.print(" (Size: "); Serial.print(fileSize); Serial.println(" bytes)");
        Serial.println("---------------------------------------------");

        // Open this specific file for streaming
        File file = LittleFS.open(fileName, "r");
        if (file) {
            while (file.available()) {
                // Read individual file characters and dump directly to VSCode
                Serial.write(file.read());
            }
            file.close();
        }
        fileCount++;
    }

    if (fileCount == 0) {
        Serial.println("\n[-] No data files found anywhere on this chip.");
    }
    
    Serial.println("\n=============================================");
    Serial.println("         ALL STORAGE OVERVIEWS COMPLETE      ");
    Serial.println("=============================================");
}

void loop() {
    // Left empty intentionally
}