#include "Arduino.h"
#include "ESP8266WiFi.h"
#include "ESP8266WebServer.h"
#include "espnow.h"

struct DataPoint {
    uint32_t timestamp;
    float pitch;
    float roll;
    float yaw;
    float net_z;
    float alt;
};

struct TelemetryFrame {
    DataPoint samples[5];
};
TelemetryFrame incomingFrame;

ESP8266WebServer server(80);

// Volatile buffer: holds data ONLY until the browser fetches it
String webBuffer = "";

void onDataRecv(uint8_t * mac, uint8_t *incomingDataRaw, uint8_t len) {
    memcpy(&incomingFrame, incomingDataRaw, sizeof(incomingFrame));
    
    // Append incoming data points to our temporary transfer buffer
    for(int i = 0; i < 5; i++) {
        webBuffer += "[" + String(incomingFrame.samples[i].timestamp) + " ms] " +
                     "P: " + String(incomingFrame.samples[i].pitch, 1) + " | " +
                     "R: " + String(incomingFrame.samples[i].roll, 1) + " | " +
                     "Y: " + String(incomingFrame.samples[i].yaw, 1) + " | " +
                     "Thrust-Z: " + String(incomingFrame.samples[i].net_z, 2) + " m/s2 | " +
                     "Alt: " + String(incomingFrame.samples[i].alt, 1) + " m\n";
    }
}

// Serves the new text data and flushes the buffer to free up RAM
void handleDataStream() {
    if (webBuffer.length() > 0) {
        server.send(200, "text/plain", webBuffer);
        webBuffer = ""; // Flush buffer immediately after transmission
    } else {
        server.send(200, "text/plain", ""); // Send empty response if no new data
    }
}

// user interface
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #ffffff; color: #111111; padding: 20px; margin: 0; }";
    html += "h1 { font-size: 20px; font-weight: 600; margin: 0 0 15px 0; color: #333333; }";
    html += "#terminal { width: 100%; height: 85vh; background: #fafafa; color: #111111; border: 1px solid #dddddd; border-radius: 4px; padding: 12px; font-family: 'SF Mono', Consolas, 'Liberation Mono', monospace; font-size: 13px; line-height: 1.5; overflow-y: scroll; white-space: pre; box-sizing: border-box; }";
    html += "</style></head><body>";
    
    html += "<h1>Telemetry Log</h1>";
    html += "<div id='terminal'>[System initialized. Awaiting data...]</div>";
    
    html += "<script>";
    html += "const term = document.getElementById('terminal');";
    html += "let firstData = true;";
    html += "setInterval(() => {";
    html += "  fetch('/stream').then(res => res.text()).then(data => {";
    html += "    if (data.trim().length > 0) {";
    html += "      if (firstData) { term.textContent = ''; firstData = false; }";
    html += "      ";
    // Smart-Scroll Logic: Checks if the user is currently looking at old data higher up
    html += "      const isAtBottom = (term.scrollHeight - term.clientHeight <= term.scrollTop + 50);";
    html += "      ";
    html += "      term.textContent += data;"; // Append new data directly to the end of the existing log
    html += "      ";
    // Only snap the window down if the user was already sitting at the bottom of the log
    html += "      if (isAtBottom) { term.scrollTop = term.scrollHeight; }";
    html += "    }";
    html += "  }).catch(err => {});";
    html += "}, 200);";
    html += "</script></body></html>";
    
    server.send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Rocket_Telemetry", "password123"); 
    
    if (esp_now_init() != 0) { return; }
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(onDataRecv);

    server.on("/", handleRoot);
    server.on("/stream", handleDataStream);
    server.begin();
}

void loop() {
    server.handleClient(); 
}