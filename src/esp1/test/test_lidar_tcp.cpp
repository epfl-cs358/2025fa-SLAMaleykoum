/*
  ESP32 RPLIDAR C1 Serial->TCP bridge
  - RX pin = 5
  - TX pin = 4
  - Baudrate = 460800
  - WiFi AP mode
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

#define MAX_BUFFER_POINTS 3000

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

point_t pointBuffer[MAX_BUFFER_POINTS];
uint16_t pointBufferCount = 0;

float lastAngleESP = 0.0;
unsigned long lastSendTime = 0;

bool scanComplete = false;

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

    // read lidar data
    uint16_t count = lidar->readMeasurePoints();   // fills DataBuffer[]

    if (count > 0 && tcpClient && tcpClient.connected()) {
        for (uint16_t i = 0; i < count; i++) {
            stScanDataPoint_t &p = lidar->DataBuffer[i];

            // Distance in mm
            uint16_t dist_mm = (((uint16_t)p.distance_high << 8) | p.distance_low) / 4.0;
            if (dist_mm == 0) continue;

            // Angle in degrees (Q6)
            uint16_t angle_raw = ((uint16_t)p.angle_high << 7) | p.angle_low >> 1;
            float angle_deg = angle_raw / 64.0f;

            // if(!lidar->isDataBetweenBorders(p))
            //     continue;

            // Store in buffer
            if (pointBufferCount < MAX_BUFFER_POINTS) {
                pointBuffer[pointBufferCount].angle = angle_deg;
                pointBuffer[pointBufferCount].distance = dist_mm;
                pointBufferCount++;
            }

            // Detect full 360° sweep (angle wrapped)
            if (angle_deg < lastAngleESP) {
                scanComplete = true;
            }
            lastAngleESP = angle_deg;
        }
    }

    if ((scanComplete || millis() - lastSendTime > 1000) && pointBufferCount > 0) {
        scanComplete = false;
        lastSendTime = millis();

        if (tcpClient && tcpClient.connected()) {
            // Build packet once
            String packet;
            packet.reserve(pointBufferCount * 12); // approximate size per point

            for (int i = 0; i < pointBufferCount; i++) {
                packet += String(pointBuffer[i].angle, 2) + "," +
                        String(pointBuffer[i].distance) + "\n";
            }

            tcpClient.write((uint8_t*)packet.c_str(), packet.length());
        }

        pointBufferCount = 0;
    }

    // if PC sends commands (rare), ignore or handle
    if (tcpClient && tcpClient.connected() && tcpClient.available()) {
        // read but ignore → prevents TCP buffer blockage
        while (tcpClient.available()) tcpClient.read();
    }

    // periodic serial info
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        Serial.printf("AP %s - streaming %u points\n",
                      WiFi.softAPIP().toString().c_str(), count);
    }

    delay(1);
}