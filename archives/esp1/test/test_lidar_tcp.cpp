/*
  ESP32 RPLIDAR C1 Serial->TCP bridge
  - RX pin = 5
  - TX pin = 4
  - Baudrate = 460800
  - WiFi STA mode
  - TCP port: 9000

  Usage:
   - Flash on ESP32
   - Connect PC to SPOT-iot WiFi
    - On PC, connect a TCP client to ESP IP port 9000
      e.g. socat or netcat. See instructions below.
*/

#include "test_common_esp1.h"

float lastAngleESP = 0.0;

void setup_test_lidar_tcp() {
    Serial.begin(115200);

    lidar.start();
    delay(1000);

    // Connect to Wifi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("esp32-lidar-slamaleykoum");
    WiFi.begin(ssid, password);
    Serial.print("Connexion WiFi...");

    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 8000) {
        Serial.print(".");
        delay(300);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n❌ WiFi failed. Restarting...");
        ESP.restart();
    }

    Serial.println("\n✔ WiFi connected!");
    Serial.println(WiFi.localIP());
    
    // Start TCP server
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    delay(1000);

    Serial.println(WiFi.localIP());
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
    if ((scanComplete || millis() - lastSendTime > 33) && scan.count > 0) {
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