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
#include "esp1/planning/mission_planner.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// --- NETWORK CONFIG ---
const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

WiFiServer tcpServer(TCP_PORT);

// --- RTOS HANDLES ---
QueueHandle_t Lidar_Buffer_Queue;
QueueHandle_t Lidar_Pose_Queue;
QueueHandle_t Path_Send_Queue;
TaskHandle_t  TCP_Task_Handle = NULL;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex;
SemaphoreHandle_t State_Mutex; 

// --- GLOBAL OBJECTS ---
const int grid_size_x = 200;
const int grid_size_y = 200;
const float resolution = 0.05f;

BayesianOccupancyGrid* TheMap = nullptr;
GoalManager* goalManager = nullptr; // <--- ADDED OBJECT

// --- STATE VARIABLES ---
Pose2D last_known_pose = {0,0,0,0};
GlobalPathMessage current_global_path;
MissionGoal current_mission_goal;

// --- TRANSMISSION BUFFERS ---
uint8_t* Tx_Map_Buffer = nullptr; 
struct StateSnapshot {
    Pose2D pose;
    MissionGoal goal;
    GlobalPathMessage path;
} Tx_Snapshot;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial); 

// =============================================================
// TASK 1: IPC RECEIVE
// =============================================================
void IPC_Receive_Task(void* parameter) {
    while (1) {
        if (IPC_Serial.available()) {
            uint8_t msg_id = IPC_Serial.read();
            if (msg_id == MSG_POSE) {
                Pose2D incoming_pose;
                if (IPC_Serial.readBytes((char*)&incoming_pose, sizeof(Pose2D)) == sizeof(Pose2D)) {
                    if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        last_known_pose = incoming_pose;
                        xSemaphoreGive(Pose_Mutex);
                    }

                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================
// TASK 2: LIDAR READ 
// =============================================================
void Lidar_Read_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(10); 
    static LiDARScan scan; 
    float lastAngleESP = 0.0f; 

    while (1) {
        bool scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            if (scan.count > 10) { 
                xQueueSend(Lidar_Buffer_Queue, &scan, 0);
            }
            scan.count = 0;
            lastAngleESP = 0.0f; 
        }
        vTaskDelay(xDelay);
    }
}

// =============================================================
// TASK 3: SYNC FUSION
// =============================================================
void Lidar_Sync_Map_Task(void* parameter) {
    SyncedScan* synced_data = new SyncedScan();
    LiDARScan* scan_buffer = new LiDARScan();
    int local_counter = 0;
    
    while (1) {
        if (xQueueReceive(Lidar_Buffer_Queue, scan_buffer, portMAX_DELAY) == pdPASS) {
            local_counter++;
            if (local_counter % 3 != 0) continue; 

            synced_data->scan = *scan_buffer;
            if (xSemaphoreTake(Pose_Mutex, portMAX_DELAY) == pdTRUE) {
                synced_data->pose = last_known_pose;
                xSemaphoreGive(Pose_Mutex);
            }
            xQueueSend(Lidar_Pose_Queue, synced_data, 0);
        }
    }
}

// =============================================================
// TASK 4: BAYESIAN MAPPING
// =============================================================
void Bayesian_Grid_Task(void* parameter) {
    static SyncedScan synced_data; 
    
    while (1) {
        if (xQueueReceive(Lidar_Pose_Queue, &synced_data, pdMS_TO_TICKS(100)) == pdPASS) {
            
            // 1. Update Map
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                TheMap->update_map(synced_data, 8.0f); 
                xSemaphoreGive(Bayesian_Grid_Mutex);

                // 2. Notify TCP Task
                if (TCP_Task_Handle != NULL) xTaskNotifyGive(TCP_Task_Handle);
            }
            
            // Update Pose Snapshot
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                last_known_pose = synced_data.pose;
                xSemaphoreGive(State_Mutex);
            }
        } 
    }
}

// =============================================================
// TASK 5: TCP TRANSMIT
// =============================================================
void TCP_Transmit_Task(void* parameter) {
    WiFiClient tcpClient;
    const uint32_t map_size = grid_size_x * grid_size_y;
    uint8_t header[64]; 

    while (1) {
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // SNAPSHOT
        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(Tx_Map_Buffer, TheMap->get_map_data_color(), map_size);
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } else continue;

        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            Tx_Snapshot.path = current_global_path;
            Tx_Snapshot.goal = current_mission_goal;
            xSemaphoreGive(State_Mutex);
        }

        // HEADER
        memset(header, 0, 64);
        header[0] = 0xBE; header[1] = 0xEF;
        uint32_t sz = map_size;
        memcpy(&header[2], &sz, 4);
        uint16_t plen = Tx_Snapshot.path.current_length;
        if(plen > MAX_PATH_LENGTH) plen = 0;
        memcpy(&header[6], &plen, 2);

        memcpy(&header[8], &Tx_Snapshot.pose.x, 4);
        memcpy(&header[12], &Tx_Snapshot.pose.y, 4);
        memcpy(&header[16], &Tx_Snapshot.pose.theta, 4);

        memcpy(&header[20], &Tx_Snapshot.goal.target_pose.x, 4);
        memcpy(&header[24], &Tx_Snapshot.goal.target_pose.y, 4);
        memcpy(&header[28], &Tx_Snapshot.goal.target_pose.theta, 4);

        // SEND
        if (tcpClient.connected()) {
            tcpClient.write(header, 64);
            if(plen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.path.path, plen * sizeof(Waypoint));
            tcpClient.write(Tx_Map_Buffer, map_size);
            taskYIELD(); 
        }
    }
}

// =============================================================
// REAL MISSION PLANNER TASK
// =============================================================
void Mission_Planner_Task(void* parameter) {
    Serial.println("[Task] Mission Planner Started");
    
    // Set initial state
    goalManager->set_mission_state(GoalManager::STATE_EXPLORING);

    while (1) {
        Pose2D current_p;
        
        // 1. Get Current Pose safely
        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10)); 
            continue;
        }

        // 2. Compute New Goal
        // CRITICAL: We MUST lock the grid mutex because update_goal() reads the map.
        // If the map is being updated (written) while we read, we get garbage or crash.
        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            
            MissionGoal new_goal = goalManager->update_goal(current_p, *TheMap);
            
            xSemaphoreGive(Bayesian_Grid_Mutex);

            // 3. Publish Goal to Global State (for TCP & Global Planner)
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                current_mission_goal = new_goal;
                xSemaphoreGive(State_Mutex);
            }
        }

        // Run at 2Hz (every 500ms)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================
// GLOBAL PLANNER STUB
// =============================================================
void Global_Planner_Task(void* parameter) {
    // Generate a fake path for visualization
    while (1) {
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Here you would normally call globalPlanner->plan(pose, current_mission_goal);
            current_global_path.current_length = 5;
            for(int i=0; i<5; i++) {
                current_global_path.path[i].x = current_mission_goal.target_pose.x * (i/5.0);
                current_global_path.path[i].y = current_mission_goal.target_pose.y * (i/5.0);
            }
            xSemaphoreGive(State_Mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP1 FULL NODE ---");

    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    State_Mutex = xSemaphoreCreateMutex();

    TheMap = new BayesianOccupancyGrid(resolution, grid_size_x, grid_size_y);
    Tx_Map_Buffer = (uint8_t*)malloc(grid_size_x * grid_size_y);
    
    // Instantiate Goal Manager
    goalManager = new GoalManager({0.0f, 0.0f, 0.0f, 0});
    
    if (!TheMap || !Tx_Map_Buffer || !goalManager) { 
        Serial.println("MEMORY ALLOC FAILED"); 
        while(1); 
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(3000);
    Serial.println("AP Mode set");
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    esp_link.begin(); 
    lidar.start(); 
    delay(1000);

    Lidar_Buffer_Queue = xQueueCreate(1, sizeof(LiDARScan));
    Lidar_Pose_Queue   = xQueueCreate(1, sizeof(SyncedScan));
    Path_Send_Queue    = xQueueCreate(1, sizeof(GlobalPathMessage));

    // Core 0
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC_Rx", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP_Tx", 8192, NULL, 3, &TCP_Task_Handle, 0); 
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 2, NULL, 0); // Increased Stack

    // Core 1
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar_Read", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Lidar_Sync", 8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map_Comp", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global_Plan", 2048, NULL, 2, NULL, 1);
}

void loop() {}
