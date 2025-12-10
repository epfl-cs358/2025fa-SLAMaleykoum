#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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
MissionPlanner* mission_planner = nullptr; 

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

float pathA_X[] = {
    0.00, 0.00, 2.00, 4.00, 6.00, 8.00,
    8.50, 8.50, 8.50, 4.50, 2.50, 0.50,
    -2.50, -3.50, -3.50
};
float pathA_Y[] = {
    0.00, 0.30, 0.30, 0.30, 0.30, 0.30,
    0.30, -3.70, -5.40, -5.40, -5.40, -5.40,
    -5.40, -5.40, 0.30
};
const int pathA_size = sizeof(pathA_X) / sizeof(pathA_X[0]);

// =============================================================
// TASK 1: IPC RECEIVE
// =============================================================
void IPC_Receive_Task(void* parameter) {
    while (1) {
        esp_link.poll();
        Pose2D incoming_pose;
        if (esp_link.get_pos(incoming_pose)) {
            if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                last_known_pose = incoming_pose;
                xSemaphoreGive(Pose_Mutex);
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

    // TODO: SYNC DATA FROM TIMESTAMPS !!!
    
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
                TheMap->update_map(synced_data, 3.0f); 
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
// TODO: We can optimize here
// RLE COMPRESSION FOR MAP TRANSMISSION (!!!! CAN GET RID OF THIS IF IT TAKES TOO MUCH SPACE or time !!!!)
// RLE Buffer: 200x200 = 40000 bytes. RLE typically reduces this >90%.
// We allocate a static buffer to handle the worst case.
static uint8_t rle_buffer[10000]; // 10KB static buffer for compressed map

void TCP_Transmit_Task(void* parameter) {
    WiFiClient tcpClient;
    uint8_t header[64]; 
    
    const TickType_t TelemetryFreq = pdMS_TO_TICKS(100); 
    const TickType_t MapFreq = pdMS_TO_TICKS(4000); // 4 Seconds
    
    TickType_t lastMapTime = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, TelemetryFreq);

        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                Serial.println("TCP: Connected!");
            } else {
                continue;
            }
        }

        // 1. Prepare State (Quick Copy)
        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            Tx_Snapshot.path = current_global_path;
            Tx_Snapshot.goal = current_mission_goal;
            xSemaphoreGive(State_Mutex);
        }

        // 2. Decide if we send map
        bool send_map_now = (xTaskGetTickCount() - lastMapTime > MapFreq);
        uint32_t payload_map_sz = 0;

        // ---------------------------------------------------------
        // RLE COMPRESSION (NO MUTEX)
        // ---------------------------------------------------------
        // We read the map "Dirty". If Lidar writes to it while we read,
        // we might get a single pixel "tear", which is fine for debug.
        if (send_map_now) {
            // OPTIMIZATION: REMOVED MUTEX LOCK
            const int8_t* raw_map = TheMap->get_map_data();
            uint32_t total_cells = TheMap->get_total_size();
            
            uint32_t rle_idx = 0;
            uint32_t raw_idx = 0;

            while (raw_idx < total_cells && rle_idx < sizeof(rle_buffer) - 2) {
                int8_t current_val = raw_map[raw_idx];
                uint8_t count = 0;

                // Count identical bytes (max 255)
                // If raw_map[raw_idx] changes mid-loop due to Lidar, loop breaks.
                // This is safe; it just starts a new RLE run.
                while (raw_idx < total_cells && raw_map[raw_idx] == current_val && count < 255) {
                    count++;
                    raw_idx++;
                }

                rle_buffer[rle_idx++] = count;
                rle_buffer[rle_idx++] = (uint8_t)current_val;
            }
            
            payload_map_sz = rle_idx; // Compressed size
        }

        // 3. Header
        memset(header, 0, 64);
        header[0] = 0xBE; header[1] = 0xEF;
        
        // Send COMPRESSED size
        memcpy(&header[2], &payload_map_sz, 4);
        
        uint16_t plen = Tx_Snapshot.path.current_length;
        if(plen > MAX_PATH_LENGTH) plen = 0;
        memcpy(&header[6], &plen, 2);

        memcpy(&header[8], &Tx_Snapshot.pose.x, 4);
        memcpy(&header[12], &Tx_Snapshot.pose.y, 4);
        memcpy(&header[16], &Tx_Snapshot.pose.theta, 4);

        memcpy(&header[20], &Tx_Snapshot.goal.target_pose.x, 4);
        memcpy(&header[24], &Tx_Snapshot.goal.target_pose.y, 4);
        memcpy(&header[28], &Tx_Snapshot.goal.target_pose.theta, 4);

        if (tcpClient.connected()) {
            tcpClient.write(header, 64);
            if(plen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.path.path, plen * sizeof(Waypoint));
            
            // 4. SEND COMPRESSED MAP 
            if (payload_map_sz > 0) {
                tcpClient.write(rle_buffer, payload_map_sz);
                lastMapTime = xTaskGetTickCount();
            }
        }
    }
}

// =============================================================
// MISSION PLANNER TASK
// =============================================================
void Mission_Planner_Task(void* parameter) {
    Serial.println("[Task] Mission Planner Started");
    
    // Set initial state
    mission_planner->set_mission_state(MissionGoalType::EXPLORATION_MODE);

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
        // OPTIMIZATION: REMOVED MUTEX LOCK!
        // We read the map "dirty" (without lock). The map is a fixed array of int8.
        // Reading a byte while it is being written is atomic on ESP32.
        // This prevents the heavy Mission Planner from blocking TCP or Lidar tasks.
        if (mission_planner->get_current_state() != RETURN_HOME) {
            
            MissionGoal new_goal = mission_planner->update_goal(current_p, *TheMap);

            // 3. Publish Goal to Global State (for TCP & Global Planner)
            if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                current_mission_goal = new_goal;
                xSemaphoreGive(State_Mutex);
            }
            
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } 

        // Run at 2Hz (every 500ms)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================
// GLOBAL PLANNER TASK
// =============================================================
void Global_Planner_Task(void* parameter) {
    while (1) {
        GlobalPathMessage pathMsg;  // Local variable
        
        // Build the path inside the mutex (fast)
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            pathMsg.current_length = pathA_size;
            pathMsg.timestamp_ms = millis();
            pathMsg.path_id = 0;
            
            for (int i = 0; i < pathA_size; i++) {
                pathMsg.path[i].x = pathA_X[i];
                pathMsg.path[i].y = pathA_Y[i];
            }
            
            // Also update the global state for TCP
            current_global_path = pathMsg;
            
            xSemaphoreGive(State_Mutex);  // ← Release BEFORE sending
        }
        
       
        
        // Send AFTER releasing the mutex (slow operation)
        esp_link.sendPath(pathMsg);        
        
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
    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});
    current_mission_goal.target_pose = {0.0f, 0.0f, 0.0f, 0};
    current_mission_goal.type = EXPLORATION_MODE;
    current_global_path.current_length = 0;
    
    if (!TheMap || !mission_planner) { 
        Serial.println("MEMORY ALLOC FAILED"); 
        while(1); 
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(1000); // Give WiFi time to settle
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    esp_link.begin(); 
    lidar.start(); 
    delay(1000);

    // Queue Sizes: Increased Lidar buffer slightly to handle bursts
    Lidar_Buffer_Queue = xQueueCreate(2, sizeof(LiDARScan));
    Lidar_Pose_Queue   = xQueueCreate(2, sizeof(SyncedScan));
    Path_Send_Queue    = xQueueCreate(1, sizeof(GlobalPathMessage));

    // --- TASK PRIORITIES & CORE ASSIGNMENT ---
    // Rule: Higher number = Higher Priority.
    // Core 0: System & Comm | Core 1: Math & Sensors

    // -- Core 0 (Comms) --
    // IPC is critical (100Hz), keep high.
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC_Rx", 4096, NULL, 5, NULL, 0);
    // TCP is slow but needs to stay alive. Priority 3 is fine.
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP_Tx", 8192, NULL, 3, &TCP_Task_Handle, 0); 
    
    // Mission Planner (OPTIMIZATION: LOWERED PRIORITY)
    // Was 2, now 1. It runs in background and shouldn't starve TCP.
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 1, NULL, 0);

    // -- Core 1 (Heavy Lifting) --
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar_Read", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Lidar_Sync", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map_Comp", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global_Plan", 8192, NULL, 2, NULL, 1);
}

void loop() {}
