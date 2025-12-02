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

#include <WiFiClient.h>
#include <WiFiServer.h>
#include "test_common_esp1.h"

float lastAngleESP = 0.0;
unsigned long lastSendTime = 0;
bool scanComplete = false;

void setup_test_lidar_tcp() {
    lidar.start();
    delay(1000);

    // Start WiFi in AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(3000);
    
    // Start TCP server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    delay(1000);
}

void loop_test_lidar_tcp() {
    // Accept new client if none
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient newClient = tcpServer.available();
        if (newClient) {
            tcpClient = newClient;
            tcpClient.setNoDelay(true);
        }
    }

    lidar.build_scan(&scan, scanComplete, lastAngleESP);

    // Send full scan only when scanComplete is true
    if ((scanComplete || millis() - lastSendTime > 50) && scan.count > 0) {
        scanComplete = false;
        lastSendTime = millis();

        if (tcpClient && tcpClient.connected()) {
            // Build packet once
            String packet;
            packet.reserve(scan.count * 12); // approximate size per point

            for (int i = 0; i < scan.count; i++) {
                packet += String(scan.angles[i], 2) + "," +
                        String(scan.distances[i]) + "\n";
            }

            tcpClient.write((uint8_t*)packet.c_str(), packet.length());
        }
        scan.count = 0;
    }
}