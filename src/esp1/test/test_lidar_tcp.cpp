/*
  ESP32 RPLIDAR C1 Serial->TCP bridge
  - RX pin = 5
  - TX pin = 4
  - Baudrate = 460800
  - No motor control
  - WiFi AP mode (change to STA if needed)
  - TCP port: 9000

  Usage:
   - Flash on ESP32
   - Connect PC to WiFi SSID "LIDAR_AP" / password "l1darpass"
   - On PC, connect a TCP client to ESP IP (default 192.168.4.1) port 9000
     e.g. socat or netcat. See instructions below.
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include "test_common_esp1.h"

const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

// TCP server and client
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// simple stats
unsigned long lastPrint = 0;
uint32_t bytesFromLidarToNet = 0;
uint32_t bytesFromNetToLidar = 0;
uint32_t tcpClientsAccepted = 0;

void setup_test_lidar_tcp() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== ESP32 LIDAR Serial->TCP Bridge ===");

    bool ret = lidar->start(standard);
    delay(1000);
    if (ret) {
        Serial.println("🟢 Rplidar C1 started correctly!\r\n");
    } else {
        Serial.println("🔴 Error starting Rplidar C1\r\n");
    }

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    Serial.println("[LiDAR] STANDARD mode started.");
    // Give the lidar some time
    delay(100);

    // Start WiFi in AP mode
    Serial.printf("Starting WiFi AP '%s' ...\n", ssid);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(ssid, password);
    if (!ok) {
        Serial.println("Failed to start AP!");
    } else {
        IPAddress ip = WiFi.softAPIP();
        Serial.print("AP started. IP: ");
        Serial.println(ip);
        Serial.printf("Connect your PC to %s (password: %s)\n", ssid, password);
    }

    // Start TCP server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("TCP server started on port %u\n", TCP_PORT);
}

void loop_test_lidar_tcp() {
    // Accept new client if none
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient newClient = tcpServer.available();
        if (newClient) {
            if (!tcpClient || !tcpClient.connected()) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.printf("Client connected from %s\n",
                              tcpClient.remoteIP().toString().c_str());
            } else {
                newClient.stop();  // reject extras
            }
        }
    }

    ///////// 1) READ LIDAR DATA USING YOUR SDK /////////
    uint16_t count = lidar->readMeasurePoints();   // fills DataBuffer[]

    if (count > 0 && tcpClient && tcpClient.connected()) {
        for (uint16_t i = 0; i < count; i++) {
            stScanDataPoint_t &p = lidar->DataBuffer[i];

            // ---- Distance in mm ----
            uint16_t dist_mm = ((uint16_t)p.distance_high << 8) | p.distance_low;
            if (dist_mm == 0) continue;   // invalid point

            // ---- Angle in degrees ----
            // Angle is Q6: angle_raw / 64.0
            uint16_t angle_raw = ((uint16_t)p.angle_high << 8) | p.angle_low;
            float angle_deg = angle_raw / 64.0f;

            // ---- (Optional) Only keep valid/strong points ----
            // quality bit 0 = start flag, NOT actual quality
            // Real quality = p.quality >> 2 for RPLIDAR A2/A3/C1
            uint8_t qual = p.quality >> 2;
            // if (qual == 0) continue;    // uncomment if too noisy

            // ---- Send ASCII "angle,distance\n" ----
            char line[40];
            int len = snprintf(line, sizeof(line), "%.2f,%u\n", angle_deg, dist_mm);
            tcpClient.write((uint8_t*)line, len);
        }
    }

    ///////// 2) OPTIONAL: If PC sends commands (rare), ignore or handle /////////
    if (tcpClient && tcpClient.connected() && tcpClient.available()) {
        // read but ignore → prevents TCP buffer blockage
        while (tcpClient.available()) tcpClient.read();
    }

    ///////// 3) Periodic serial info /////////
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        Serial.printf("AP %s - streaming %u points\n",
                      WiFi.softAPIP().toString().c_str(), count);
    }

    delay(1);
}