#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <vector>

// --- Include Headers ---
#include "common/data_types.h"
#include "common/esp_link.h"
#include "esp1/hardware/lidar.h"
#include "esp1/mapping/occupancy/bayesian_grid.h"
#include "esp1/planning/global_planner.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

// TCP server and client
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// --- Global FreeRTOS Handles (Defined) ---
QueueHandle_t Lidar_Buffer_Queue;    // Stores incoming LiDARScan (from Lidar_Read_Task)
QueueHandle_t Pose_Receive_Queue;    // Stores incoming Pose2D (from IPC_Receive_Task)
QueueHandle_t Lidar_Pose_Queue;      // Stores {LiDARScan, Pose2D} (to Bayesian_Grid_Task)
QueueHandle_t Path_Send_Queue;       // Stores GlobalPathMessage (to IPC_Send_Task)
SemaphoreHandle_t Bayesian_Grid_Mutex; // Mutex to protect TheMap

// --- Shared Data & Instances ---
// Note: In a real system, the grid is likely too large for a single queue item.
const int grid_size_x = 150;
const int grid_size_y = 150;
const float resolution = 0.05f;
BayesianOccupancyGrid TheMap(resolution, grid_size_x, grid_size_y);

// Lidar instance
HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

// Assuming Serial2 is configured for ESP-to-ESP communication
HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial); 

// V1 Assumption: Perfect initial pose. You'd replace this with a clock sync offset.
uint32_t pose_to_lidar_time_offset = 0; 
Pose2D last_known_pose = {0.0f, 0.0f, 0.0f};

// For scan throttling
static uint32_t last_update = 0;
static int scan_counter = 0;

// -----------------------------------------------------------
// 1. Lidar_Read_Task (Core 0, High Freq)
// -----------------------------------------------------------
void Lidar_Read_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 10 Hz
    LiDARScan scan;
    scan.count = 0;

    while (1) {
        bool  scanComplete_ = false;
        float lastAngleESP = 0.0f;
        lidar.build_scan(&scan, scanComplete_, lastAngleESP);

        // Send to Lidar_Buffer_Queue
        if (xQueueSend(Lidar_Buffer_Queue, &scan, 0) != pdPASS) {
            // Serial.println("E1: Lidar buffer queue full.");
        }
        
        vTaskDelay(xDelay);
    }
}

// -----------------------------------------------------------
// 2. IPC_Receive_Task (Core 0, Continuous Poll)
// -----------------------------------------------------------
void IPC_Receive_Task(void* parameter) {
    /*Pose2D incoming_pose;
    GlobalPathMessage path_debug; // For protocol completeness
    while (1) {
        // Poll and decode incoming data
        esp_link.poll(); 
        
        // Process all available poses and push to queue
        while (esp_link.get_pos(incoming_pose)) {
            // Update last known pose for interpolation/sync
            last_known_pose = incoming_pose;
            
            // Send to queue for processing in Lidar_Sync_Map_Task
            if (xQueueSend(Pose_Receive_Queue, &incoming_pose, 0) != pdPASS) {
                // Serial.println("E1: Pose receive queue full.");
            }
        }
        
        // Check for any unexpected path messages (optional cleanup)
        esp_link.get_path(path_debug); 

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }*/
}

// -----------------------------------------------------------
// 3. Lidar_Sync_Map_Task (Core 1, Medium Freq)
// -----------------------------------------------------------
void Lidar_Sync_Map_Task(void* parameter) {
    SyncedScan synced_data;
    LiDARScan scan;
    
    // We wait for a new scan before trying to match the pose
    while (xQueueReceive(Lidar_Buffer_Queue, &scan, portMAX_DELAY) == pdPASS) {
        scan_counter++;

        // Only process 1 out of 3 scans → 3x lighter mapping
        if (scan_counter % 3 != 0)  
            continue;

        // V1: Use the latest received pose as the "perfect" pose
        // In a real scenario, you'd iterate through Pose_Receive_Queue 
        // to find the closest timestamp and interpolate.
        
        // For V1, we simply wait for the latest pose to be processed by IPC_Receive_Task
        // Note: This assumes the pose delay is small.
        synced_data.scan = scan;
        synced_data.pose = last_known_pose; // Simplification for V1
        
        if(xQueueSend(Lidar_Pose_Queue, &synced_data, 0) != pdPASS) {
            // Serial.println("E1: Lidar-Pose queue full.");
        }
    }
}

// -----------------------------------------------------------
// 4. Bayesian_Grid_Task (Core 1, Medium Freq)
// -----------------------------------------------------------
void Bayesian_Grid_Task(void* parameter) {
    SyncedScan synced_data;
    
    while (xQueueReceive(Lidar_Pose_Queue, &synced_data, portMAX_DELAY) == pdPASS) {

        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {

            // ----------------------------
            // Accept new client
            // ----------------------------
            if (!tcpClient || !tcpClient.connected()) {
                WiFiClient newClient = tcpServer.available();
                if (newClient) {
                    tcpClient = newClient;
                    tcpClient.setNoDelay(true);
                    Serial.printf("Client connected from %s\n",
                                  tcpClient.remoteIP().toString().c_str());
                }
            }

            // ----------------------------
            // Update Bayesian map
            // ----------------------------
            TheMap.update_map(synced_data.scan, synced_data.pose, 8);

            // ----------------------------
            // TCP STREAM
            // ----------------------------
            if (tcpClient && tcpClient.connected()) {

                const uint8_t* map = TheMap.get_map_data_color();
                uint16_t w = TheMap.grid_size_x;
                uint16_t h = TheMap.grid_size_y;
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
                tcpClient.write(header, 20);
                tcpClient.write(map, map_size);
            }

            xSemaphoreGive(Bayesian_Grid_Mutex);
        }
    }
}

// -----------------------------------------------------------
// 5. Global_Planner_Task (Core 1, Low Freq)
// -----------------------------------------------------------
void Global_Planner_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 Hz
    GlobalPathMessage new_path;
    
    // Initial dummy path creation
    new_path.path_id = 1;
    new_path.current_length = 2;
    new_path.path[0] = {1.0f, 0.0f};
    new_path.path[1] = {2.0f, 1.0f};
    
    while (1) {
        // 1. Access the Map (read-only)
        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            // GlobalPlanner::computePath(TheMap, goal, &new_path); // Placeholder
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } else {
            // Serial.println("E1: Failed to acquire map mutex for planning.");
        }
        
        // 2. Update path details and send
        new_path.path_id++;
        new_path.timestamp_ms = millis();
        
        // Overwrite any previous path in the queue (size 1)
        xQueueOverwrite(Path_Send_Queue, &new_path); 
        vTaskDelay(xDelay);
    }
}

// -----------------------------------------------------------
// 6. IPC_Send_Task (Core 0, Low Freq)
// -----------------------------------------------------------
void IPC_Send_Task(void* parameter) {
    /*GlobalPathMessage path_to_send;
    // Wait indefinitely for a new path to be available
    while (xQueueReceive(Path_Send_Queue, &path_to_send, portMAX_DELAY) == pdPASS) {
        esp_link.sendPath(path_to_send);
        // Serial.printf("E1: Sent path ID: %lu\n", path_to_send.path_id);
    }*/
}


// ===========================================================
// SETUP AND LOOP
// ===========================================================
void setup() {
    Serial.begin(115200);

    bool ret = lidar.start();

    // Wait for Lidar to initialize
    delay(1000);

    // Start IPC link
    esp_link.begin();

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

    // --- 1. Create Queues ---
    Lidar_Buffer_Queue    = xQueueCreate(2, sizeof(LiDARScan));
    Pose_Receive_Queue    = xQueueCreate(4, sizeof(Pose2D));
    Lidar_Pose_Queue      = xQueueCreate(2, sizeof(SyncedScan)); // Using the new struct
    Path_Send_Queue       = xQueueCreate(1, sizeof(GlobalPathMessage));

    // --- 2. Create Mutexes ---
    Bayesian_Grid_Mutex   = xSemaphoreCreateMutex();

    // --- 3. Create Tasks ---
    // Core 0: I/O and communication
    // xTaskCreatePinnedToCore(IPC_Receive_Task,      "IPC_RX_E1", 4096, NULL, 5, NULL, 0); // Highest priority for continuous poll
    xTaskCreatePinnedToCore(Lidar_Read_Task,       "Lidar_Read", 4096, NULL, 4, NULL, 0);
    // xTaskCreatePinnedToCore(IPC_Send_Task,         "IPC_TX_E1", 4096, NULL, 3, NULL, 0);

    // Core 1: Processing
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task,   "Lidar_Sync", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task,    "Map_Update", 8192, NULL, 3, NULL, 1); // Larger stack for map processing
    // xTaskCreatePinnedToCore(Global_Planner_Task,   "Global_Plan", 6144, NULL, 2, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}