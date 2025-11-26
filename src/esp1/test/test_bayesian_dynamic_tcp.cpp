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

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include "test_common_esp1.h"

const char* ssid_ = "LIDAR_AP";
const char* password_ = "l1darpass";
const uint16_t TCP_PORT_ = 9000;

// TCP server and client
WiFiServer tcpServerLidar_(TCP_PORT_);
WiFiClient tcpClient_;

// simple stats
unsigned long lastPrint_ = 0;
uint32_t bytesFromLidarToNet_ = 0;
uint32_t bytesFromNetToLidar_ = 0;
uint32_t tcpClient_sAccepted_ = 0;

float lastAngleESP_ = 0.0;
unsigned long lastSendTime_ = 0;

bool scanComplete_ = false;

Pose2D last_known_pose_ = {0.0f, 0.0f, 0.0f};

const int grid_size_x_ = 80;
const int grid_size_y_ = 80;
const float resolution_ = 0.20f;
BayesianOccupancyGrid TheMap_(resolution_, grid_size_x_, grid_size_y_);

void setup_bayesian_dynamic_tcp() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("=== ESP32 LIDAR Serial->TCP Bridge ===");

    bool ret = lidar.start();
    delay(1000);
    if (ret) {
        Serial.println("🟢 Rplidar C1 started correctly!\r\n");
    } else {
        Serial.println("🔴 Error starting Rplidar C1\r\n");
    }

    Serial.println("[LiDAR] STANDARD mode started.");
    // Give the lidar some time
    delay(100);

    // Start WiFi in AP mode
    Serial.printf("Starting WiFi AP '%s' ...\n", ssid_);
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(ssid_, password_);
    if (!ok) {
        Serial.println("Failed to start AP!");
    } else {
        IPAddress ip = WiFi.softAPIP();
        Serial.print("AP started. IP: ");
        Serial.println(ip);
        Serial.printf("Connect your PC to %s (password_: %s)\n", ssid_, password_);
    }

    // Start TCP server
    tcpServerLidar_.begin();
    tcpServerLidar_.setNoDelay(true);
    Serial.printf("TCP server started on port %u\n", TCP_PORT_);
}

void loop_bayesian_dynamic_tcp() {
    // Accept new client if none
    if (!tcpClient_ || !tcpClient_.connected()) {
        WiFiClient newClient = tcpServerLidar_.available();
        if (newClient) {
            tcpClient_ = newClient;
            tcpClient_.setNoDelay(true);
            Serial.printf("Client connected from %s\n",
                            tcpClient_.remoteIP().toString().c_str());
        }
    }

    LiDARScan scan;
    scan.count = 0;

    // read lidar data
    int count = lidar.readMeasurePoints();

    if(count > 0) {
        // Copy data into scan structure
        for (int i = 0; i < count; i++) {
            if (scan.count >= MAX_LIDAR_POINTS)
                break;

            rawScanDataPoint_t dataPoint = lidar.DataBuffer[i];

            uint16_t dist_mm = lidar.calcDistance(dataPoint.distance_low, dataPoint.distance_high);
            if (dist_mm <= 0) continue;
            uint16_t angle_deg = lidar.calcAngle(dataPoint.angle_low, dataPoint.angle_high);
            uint8_t quality = dataPoint.quality >> 2;

            // Store in buffer
            scan.angles[scan.count] = angle_deg;
            scan.distances[scan.count] = dist_mm;
            scan.qualities[scan.count] = quality;
            scan.count++;

            if (angle_deg < lastAngleESP_) {
                scanComplete_ = true;
            }
            lastAngleESP_ = angle_deg;
        }
        scan.timestamp_ms = millis();
    }

    if (millis() - lastSendTime_ > 1000 && scan.count > 0) {
        SyncedScan synced_data;
        scanComplete_ = false;
        synced_data.scan = scan;
        synced_data.pose = last_known_pose_;

        // ----------------------------
        // Accept new client
        // ----------------------------
        if (!tcpClient_ || !tcpClient_.connected()) {
            WiFiClient newClient = tcpServerLidar_.available();
            if (newClient) {
                tcpClient_ = newClient;
                tcpClient_.setNoDelay(true);
                Serial.printf("Client connected from %s\n",
                                tcpClient_.remoteIP().toString().c_str());
            }
        }

        // ----------------------------
        // Update Bayesian map
        // ----------------------------
        TheMap_.update_map(synced_data.scan, synced_data.pose, 8);
        
        // ----------------------------
        // TCP STREAM
        // ----------------------------
        if (tcpClient_ && tcpClient_.connected()) {

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

            // Robot pose (float → bytes)
            float rx = synced_data.pose.x;
            float ry = synced_data.pose.y;
            float rt = synced_data.pose.theta;

            memcpy(&header[6],  &rx, 4);
            memcpy(&header[10], &ry, 4);
            memcpy(&header[14], &rt, 4);

            // Map size
            header[18] = (map_size >> 8) & 0xFF;
            header[19] = (map_size & 0xFF);

            // Send header + map
            tcpClient_.write(header, 20);
            tcpClient_.write(map, map_size);
        }

        lastSendTime_ = millis();
    }
}