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

// --- NETWORK CONFIG ---
const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// --- RTOS HANDLES ---
QueueHandle_t Lidar_Buffer_Queue;
QueueHandle_t Lidar_Pose_Queue;
QueueHandle_t Path_Send_Queue;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex; // <--- NEW: Protects 'last_known_pose'

// --- GLOBAL OBJECTS ---
const int grid_size_x = 80;
const int grid_size_y = 80;
const float resolution = 0.05f;

BayesianOccupancyGrid* TheMap = nullptr;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial); 

// Global Pose (Protected by Pose_Mutex) - Initialized to zero
Pose2D last_known_pose = {0.0f, 0.0f, 0.0f, 0};

// --- HELPER FUNCTIONS ---
void print_heap(const char* label) {
    Serial.printf("[%s] Free Heap: %d bytes\n", label, esp_get_free_heap_size());
}

// =============================================================
// TASK 1: IPC RECEIVE
// Reads pose from ESP2 so the map knows where the car is.
// =============================================================
void IPC_Receive_Task(void* parameter) {
    Serial.println("[Task] IPC Rx Started");
    
    // We expect packets starting with a MsgId byte
    while (1) {
        if (IPC_Serial.available()) {
            uint8_t msg_id = IPC_Serial.read();

            // Check against data_types.h enums
            if (msg_id == MSG_POSE) {
                Pose2D incoming_pose;
                // Read the struct bytes directly
                if (IPC_Serial.readBytes((char*)&incoming_pose, sizeof(Pose2D)) == sizeof(Pose2D)) {
                    
                    // Safely update the global variable
                    if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        last_known_pose = incoming_pose;
                        // Debug print to verify rotation
                        // Serial.printf("Pose Update: x=%.2f y=%.2f th=%.2f\n", 
                        //               last_known_pose.x, last_known_pose.y, last_known_pose.theta);
                        xSemaphoreGive(Pose_Mutex);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
    }
}

// =============================================================
// TASK 2: LIDAR READ (DRIVER)
// =============================================================
void Lidar_Read_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(10); 
    static LiDARScan scan; 
    float lastAngleESP = 0.0f; 

    Serial.println("[Task] Real Lidar Read Started");

    while (1) {
        bool scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            if (scan.count > 10) { 
                if (xQueueSend(Lidar_Buffer_Queue, &scan, 0) != pdPASS) {
                    // Queue full, drop scan
                }
            }
            scan.count = 0;
            lastAngleESP = 0.0f; 
        }
        vTaskDelay(xDelay);
    }
}

// =============================================================
// TASK 3: SYNC (FUSION)
// Combines Lidar Data + Current Pose
// =============================================================
void Lidar_Sync_Map_Task(void* parameter) {
    Serial.println("[Task] Sync Started");

    SyncedScan* synced_data = new SyncedScan();
    LiDARScan* scan_buffer = new LiDARScan();
    
    int local_counter = 0;
    
    while (1) {
        if (xQueueReceive(Lidar_Buffer_Queue, scan_buffer, portMAX_DELAY) == pdPASS) {
            local_counter++;
            // Downsample: Process only 1 out of 3 scans to save CPU
            if (local_counter % 3 != 0) continue;

            synced_data->scan = *scan_buffer;
            
            // --- CRITICAL: READ POSE SAFELY ---
            if (xSemaphoreTake(Pose_Mutex, portMAX_DELAY) == pdTRUE) {
                synced_data->pose = last_known_pose;
                xSemaphoreGive(Pose_Mutex);
            }
            // ----------------------------------
            
            xQueueSend(Lidar_Pose_Queue, synced_data, 0);
        }
    }
}

// =============================================================
// TASK 4: BAYESIAN MAPPING & TCP
// =============================================================
void Bayesian_Grid_Task(void* parameter) {
    static SyncedScan synced_data; 
    Serial.println("[Task] Bayesian Started");
    
    while (1) {
        // 1. Connection Logic
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.println(">>> TCP CLIENT CONNECTED <<<");
            }
        }

        // 2. Data Logic
        if (xQueueReceive(Lidar_Pose_Queue, &synced_data, pdMS_TO_TICKS(100)) == pdPASS) {
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                
                // Update map logic
                TheMap->update_map(synced_data, 8.0f); 

                // Send to PC
                if (tcpClient && tcpClient.connected()) {
                    uint32_t map_size = grid_size_x * grid_size_y;
                    
                    // Create Header: [Header ID ... Pose ... Size]
                    // We send the pose in the header so the Visualizer knows how to rotate the car arrow
                    uint8_t header[24];
                    memset(header, 0, 24);
                    
                    // Bytes 0-3: Magic Word
                    header[0] = 0xBE; header[1] = 0xEF;
                    
                    // Bytes 4-15: Pose (x, y, theta floats)
                    memcpy(&header[4], &synced_data.pose.x, 4);
                    memcpy(&header[8], &synced_data.pose.y, 4);
                    memcpy(&header[12], &synced_data.pose.theta, 4);

                    // Bytes 20-21: Map Size
                    header[20] = (map_size >> 8) & 0xFF;
                    header[21] = (map_size & 0xFF);

                    // Send Header + Map
                    tcpClient.write(header, 24);
                    tcpClient.write(TheMap->get_map_data_color(), map_size);
                }
                xSemaphoreGive(Bayesian_Grid_Mutex);
            }
        } 
    }
}

// =============================================================
// MISSION PLANNER STUB
// =============================================================
void Mission_Planner_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(5000); 
    while (1) {
        // Placeholder for future mission planning logic
        vTaskDelay(xDelay);
    }
}

// =============================================================
// GLOBAL PLANNER STUB
// =============================================================
void Global_Planner_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); 
    GlobalPathMessage new_path;
    new_path.path_id = 1;
    while (1) {
        new_path.path_id++;
        xQueueOverwrite(Path_Send_Queue, &new_path); 
        vTaskDelay(xDelay);
    }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP1 MAPPING NODE ---");

    // 1. Create Mutexes FIRST
    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();

    // 2. Alloc Map
    TheMap = new BayesianOccupancyGrid(resolution, grid_size_x, grid_size_y);
    if (!TheMap) { Serial.println("MAP ALLOC FAILED"); while(1); }

    // 3. WiFi
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    // 4. Hardware
    esp_link.begin(); // Ensure Serial1 is started
    lidar.start(); 

    // 5. Queues
    Lidar_Buffer_Queue = xQueueCreate(1, sizeof(LiDARScan));
    Lidar_Pose_Queue   = xQueueCreate(1, sizeof(SyncedScan));
    Path_Send_Queue    = xQueueCreate(1, sizeof(GlobalPathMessage));

    // 6. Tasks
    // IPC Receive on Core 0 (Comms)
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC_Rx", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission_Plan", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global_Plan", 2048, NULL, 2, NULL, 1);

    // Mapping on Core 1 (Math)
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar_Read", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Lidar_Sync", 8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map_Update", 8192, NULL, 3, NULL, 1);
}

void loop() {}
