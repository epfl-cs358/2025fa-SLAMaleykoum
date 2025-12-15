/*
  ESP32 RPLIDAR C1 Serial->UDP bridge
  - RX pin = 5
  - TX pin = 4
  - Baudrate = 460800
  - WiFi STA mode
  - UDP port: 9000

  Usage:
   - Flash on ESP32
   - Connect PC to SPOT-iot WiFi
   - On PC, run the UDP Python visualizer
*/

#include "test_common_esp1.h"
#include <WiFiUdp.h>

float lastAngleESP__ = 0.0;
WiFiUDP udp;
IPAddress broadcastIP;
const int UDP_PORT = 9000;
const int BATCH_SIZE = 20; // Send 20 points per UDP packet

void setup_test_lidar_udp() {
    lidar.start();
    delay(1000);

    // Connect to Wifi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("esp32-lidar-slamaleykoum");
    WiFi.begin(ssid, password);
    delay(3000);
    
    // Calculate broadcast address
    IPAddress localIP = WiFi.localIP();
    IPAddress subnet = WiFi.subnetMask();
    broadcastIP = IPAddress(
        localIP[0] | (~subnet[0]),
        localIP[1] | (~subnet[1]),
        localIP[2] | (~subnet[2]),
        localIP[3] | (~subnet[3])
    );
    
    // Start UDP
    udp.begin(UDP_PORT);
    delay(1000);
}

void loop_test_lidar_udp() {
    lidar.build_scan(&scan, scanComplete, lastAngleESP__);

    // Send full scan only when scanComplete is true
    if (scanComplete) {
        scanComplete = false;

        // Send points in batches
        String batch = "";
        int batchCount = 0;
        
        for (int i = 0; i < scan.count; i++) {
            // Add point to batch
            batch += String(scan.angles[i], 2) + "," + String(scan.distances[i]) + "\n";
            batchCount++;
            
            // Send batch when full or at end of scan
            if (batchCount >= BATCH_SIZE || i == scan.count - 1) {
                udp.beginPacket(broadcastIP, UDP_PORT);
                udp.write((uint8_t*)batch.c_str(), batch.length());
                udp.endPacket();
                
                // Reset batch
                batch = "";
                batchCount = 0;
                
                // Small delay to prevent overwhelming the network
                delayMicroseconds(100);
            }
        }

        scan.count = 0;
    }
}