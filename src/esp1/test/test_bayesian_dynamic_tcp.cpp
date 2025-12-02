/*
  ESP32 RPLIDAR C1 Serial->TCP bridge
  - RX pin = 5
  - TX pin = 4
  - Baudrate = 460800
  - WiFi AP mode
  - TCP port: 9000

  Usage:
   - Flash on ESP32
   - Connect PC to WiFi SSID_ "LIDAR_AP" / password_ "l1darpass"
   - On PC, connect a TCP client to ESP IP (default 192.168.4.1) port 9000
     e.g. socat or netcat. See instructions below.
*/

#include <WiFiClient.h>
#include <WiFiServer.h>
#include "test_common_esp1.h"

float lastAngleESP_ = 0.0;
unsigned long lastSendTime_ = 0;
bool scanComplete_ = false;

Pose2D last_known_pose_ = {0.0f, 0.0f, 0.0f, 0};

BayesianOccupancyGrid TheMap_(0.05f, 150, 150);

void setup_bayesian_dynamic_tcp() {
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

void loop_bayesian_dynamic_tcp() {
    // Accept new client if none
    if (!tcpClient || !tcpClient.connected()) {
        WiFiClient newClient = tcpServer.available();
        if (newClient) {
            tcpClient = newClient;
            tcpClient.setNoDelay(true);
        }
    }

    lidar.build_scan(&scan, scanComplete_, lastAngleESP_);

    if ((scanComplete_ || millis() - lastSendTime_ > 100) && scan.count > 0) {
        scanComplete_ = false;

        // Update Bayesian map
        TheMap_.update_map(scan,
                           last_known_pose_,
                           8.0f);
        
        // TCP STREAM
        if (tcpClient && tcpClient.connected()) {

            const uint8_t* map = TheMap_.get_map_data_color();
            uint16_t w = TheMap_.grid_size_x;
            uint16_t h = TheMap_.grid_size_y;
            uint32_t map_size = w * h;

            uint8_t header[20];

            // Sync bytes
            header[0] = 0xAB;
            header[1] = 0xCD;

            // Grid size
            header[2] = w >> 8; header[3] = w & 0xFF;
            header[4] = h >> 8; header[5] = h & 0xFF;

            // Pose
            memcpy(&header[6],  &last_known_pose_.x,     4);
            memcpy(&header[10], &last_known_pose_.y,     4);
            memcpy(&header[14], &last_known_pose_.theta, 4);

            // Map size
            header[18] = (map_size >> 8) & 0xFF;
            header[19] = (map_size & 0xFF);

            // Send header + map
            tcpClient.write(header, 20);
            tcpClient.write(map, map_size);
        }

        lastSendTime_ = millis();
    }
}