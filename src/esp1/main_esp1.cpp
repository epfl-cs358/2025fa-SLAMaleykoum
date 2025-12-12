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
// #include "esp1/planning/local_planner.h"
#include "../../include/common/utils.h"

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
QueueHandle_t Path_Send_Queue;

// Double Buffering Queues
QueueHandle_t Lidar_Pose_Queue_Ready; // Holds pointers to FULL buffers (Sync -> Map)
QueueHandle_t Lidar_Pose_Queue_Free;  // Holds pointers to EMPTY buffers (Map -> Sync)

TaskHandle_t  TCP_Task_Handle = NULL;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex;
SemaphoreHandle_t State_Mutex; 

// --- GLOBAL OBJECTS ---
const int GRID_SIZE_X = 70;
const int GRID_SIZE_Y = 70;
const float RESOLUTION = 0.2f;
// const int factor = 4;

const uint32_t RLE_BUFFER_SIZE = 10000;

BayesianOccupancyGrid* TheMap = nullptr;
MissionPlanner* mission_planner = nullptr; 
GlobalPlannerWorkspace* gp_workspace = nullptr;

// --- DOUBLE BUFFERING POOL ---
// We allocate these ONCE on the Heap. They never move.
SyncedScan* Scan_Buffer_1 = nullptr;
SyncedScan* Scan_Buffer_2 = nullptr;

// --- STATE VARIABLES ---
Pose2D last_known_pose = {0,0,0,0};
PathMessage current_global_path = {{{0.f, 0.f}}, 1, 0, 0};
MissionGoal current_mission_goal = {{0.0f, 0.f, 0.f, 0}, EXPLORATION_MODE};
// PathMessage current_local_path = {{}, 0, 0, 0};
bool new_goal_arrived = true;
bool global_path_invalid = false; // Set by Global Planner, read by Mission Planner

// --- TRANSMISSION BUFFERS ---
uint8_t* Tx_Map_Buffer = nullptr; 
struct StateSnapshot {
    Pose2D pose;
    MissionGoal goal;
    PathMessage global_path;
    uint8_t state;
    // PathMessage local_path;
} Tx_Snapshot;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial);


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
    const TickType_t MapFreq = pdMS_TO_TICKS(2000); // 4 Seconds
    
    TickType_t lastMapTime = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, TelemetryFreq);

        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                // Serial.println("TCP: Connected!");
            } else {
                continue;
            }
        }

        // 1. Prepare State (Quick Copy)
        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            Tx_Snapshot.global_path = current_global_path;
            // Tx_Snapshot.local_path = current_local_path;
            Tx_Snapshot.goal = current_mission_goal;
            Tx_Snapshot.state = (uint8_t)current_mission_goal.type;
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

            while (raw_idx < total_cells && rle_idx < RLE_BUFFER_SIZE - 2) {
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
        
        uint8_t gplen = Tx_Snapshot.global_path.current_length;
        if(gplen > MAX_PATH_LENGTH) gplen = 0;
        uint16_t gplen_2bytes = (0 << 8) | gplen;
        memcpy(&header[6], &gplen_2bytes, 2);

        // uint16_t lplen = Tx_Snapshot.local_path.current_length;
        // if(lplen > MAX_LOCAL_PATH_LENGTH) lplen = 0;
        // memcpy(&header[8], &lplen, 2);

        memcpy(&header[8], &Tx_Snapshot.pose.x, 4);
        memcpy(&header[12], &Tx_Snapshot.pose.y, 4);
        memcpy(&header[16], &Tx_Snapshot.pose.theta, 4);

        memcpy(&header[20], &Tx_Snapshot.goal.target_pose.x, 4);
        memcpy(&header[24], &Tx_Snapshot.goal.target_pose.y, 4);
        memcpy(&header[28], &Tx_Snapshot.goal.target_pose.theta, 4);

        header[32] = Tx_Snapshot.state;

        if (tcpClient.connected()) {
            tcpClient.write(header, 64);
            if(gplen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            // if(lplen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.local_path.path, lplen * sizeof(Waypoint));
            
            // 4. SEND COMPRESSED MAP 
            if (payload_map_sz > 0) {
                tcpClient.write(rle_buffer, payload_map_sz);
                lastMapTime = xTaskGetTickCount();
            }
        }
    }
}

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
    bool scanComplete = false;

    while (1) {
        bool scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);
        // lidar.readScanLive(&scan, scanComplete, lastAngleESP);

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
    LiDARScan* scan_buffer = new LiDARScan(); // Heap alloc for temp storage
    SyncedScan* current_work_buffer = nullptr;
    int local_counter = 0;

    while (1) {
        // 1. Get raw Lidar data (Copy from ISR/Serial task)
        // We still copy here because LiDARScan is "only" ~1-2KB and it's safer for serial reading
        if (xQueueReceive(Lidar_Buffer_Queue, scan_buffer, portMAX_DELAY) == pdPASS) {
            
            // Downsample: Only process every 3rd scan to save CPU
            local_counter++;
            if (local_counter % 3 != 0) continue;

            // 2. Get a FREE buffer from the pool
            // If the queue is empty, it means the Map task is too slow. 
            // We wait 0ms (drop the frame) or small delay.
            if (xQueueReceive(Lidar_Pose_Queue_Free, &current_work_buffer, 0) == pdPASS) {
                
                // 3. Fill the buffer (Direct write to Heap, NO STACK usage)
                current_work_buffer->scan = *scan_buffer;
                
                if (xSemaphoreTake(Pose_Mutex, portMAX_DELAY) == pdTRUE) {
                    current_work_buffer->pose = last_known_pose;
                    xSemaphoreGive(Pose_Mutex);
                }

                // 4. Send the POINTER to the consumer
                // This copies 4 bytes (address), not 4500 bytes.
                xQueueSend(Lidar_Pose_Queue_Ready, &current_work_buffer, 0);
            } else {
                // Serial.println("DROP: Map task too slow, no free buffers!");
            }
        }
    }
}

// =============================================================
// TASK 4: BAYESIAN MAPPING
// =============================================================
void Bayesian_Grid_Task(void* parameter) {
    SyncedScan* incoming_data_ptr = nullptr;
    
    while (1) {
        // 1. Wait for a pointer to a FULL buffer
        if (xQueueReceive(Lidar_Pose_Queue_Ready, &incoming_data_ptr, pdMS_TO_TICKS(100)) == pdPASS) {
            
            // 2. Update Map (Read directly from Heap)
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                // Dereference pointer: *incoming_data_ptr
                TheMap->update_map(*incoming_data_ptr); 
                xSemaphoreGive(Bayesian_Grid_Mutex);

                if (TCP_Task_Handle != NULL) xTaskNotifyGive(TCP_Task_Handle);
            }
            
            // Update Pose Snapshot
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                last_known_pose = incoming_data_ptr->pose;
                xSemaphoreGive(State_Mutex);
            }

            // 3. RECYCLE: Return the pointer to the Free Queue
            // The Producer can now use this memory again.
            xQueueSend(Lidar_Pose_Queue_Free, &incoming_data_ptr, 0);
        } 
    }
}

// =============================================================
// MISSION PLANNER TASK
// =============================================================
void Mission_Planner_Task(void* parameter) {
    mission_planner->set_mission_state(MissionGoalType::EXPLORATION_MODE);
    
    while (1) {
        Pose2D current_p;
        
        // Securely read pose
        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10)); continue;
        }

        // Securely read Planner Status
        bool planner_fail = false;
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            planner_fail = global_path_invalid;
            xSemaphoreGive(State_Mutex);
        }

        // --- THE OPTIMIZED CALL ---
        // Pass the planner failure flag directly
        MissionGoal new_goal = mission_planner->update_goal(current_p, *TheMap, planner_fail);
        new_goal_arrived = true;       // Notify Global Planner


        // Check if goal actually changed before taking Mutex
        if (new_goal.target_pose.x != current_mission_goal.target_pose.x || 
            new_goal.target_pose.y != current_mission_goal.target_pose.y ||
            new_goal.type != current_mission_goal.type) 
        {
            if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                current_mission_goal = new_goal;
                global_path_invalid = false;   // Reset failure flag since we have a new goal
                xSemaphoreGive(State_Mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================
// GLOBAL PLANNER TASK
// =============================================================
void Global_Planner_Task(void* parameter) {
    GlobalPlanner planner; 
    Pose2D local_pose;
    MissionGoal global_goal;

    while (1) {
        // Initialize explicitly to ensure length is 0 if not updated
        PathMessage pathMsg = {{{0}}, 0, 0, 0};      
        
        if (new_goal_arrived) {
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                local_pose = last_known_pose;
                global_goal = current_mission_goal;
                xSemaphoreGive(State_Mutex); 
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            pathMsg = planner.generate_path(local_pose, global_goal, *TheMap, gp_workspace);

            if (pathMsg.current_length == 0) {
                 // Path planning failed!
                 if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    global_path_invalid = true;
                    current_global_path.current_length = 0; 
                    xSemaphoreGive(State_Mutex);
                }
                //  Serial.println("GP: A* Failed or Empty.");
            } else {
                // Success
                current_global_path = pathMsg;
                esp_link.sendPath(current_global_path, GLOBAL);
                new_goal_arrived = false; 
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);


    // --- MEMORY ALLOCATION ---
    // Allocate the two ping-pong buffers on the HEAP (SPIRAM if available, or internal RAM)
    // Using heap_caps_malloc ensures we don't use Stack.
    Scan_Buffer_1 = new SyncedScan();
    Scan_Buffer_2 = new SyncedScan();
    
    // if (!Scan_Buffer_1 || !Scan_Buffer_2) {
    //     Serial.println("FATAL: Heap alloc failed!"); while(1);
    // }
    gp_workspace = new GlobalPlannerWorkspace();
    if (!gp_workspace) {
        Serial.println("FATAL: Not enough RAM for Planner!");
        while(1);
    }


    // --- QUEUES ---
    Lidar_Buffer_Queue = xQueueCreate(2, sizeof(LiDARScan));
    
    // NEW: Create queues that hold POINTERS (SyncedScan*)
    // Size is number of items * sizeof(pointer) -> 2 * 4 bytes = 8 bytes total on stack. Tiny!
    Lidar_Pose_Queue_Ready = xQueueCreate(2, sizeof(SyncedScan*));
    Lidar_Pose_Queue_Free  = xQueueCreate(2, sizeof(SyncedScan*));
    
    // Prime the Free Queue with our two buffers
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_1, 0);
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_2, 0);

    Path_Send_Queue = xQueueCreate(1, sizeof(PathMessage));


// --- MUTEXES & OBJECTS ---
    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    State_Mutex = xSemaphoreCreateMutex();

    TheMap = new BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});
    
    // if (!TheMap || !mission_planner) { 
    //     // Serial.println("MEMORY ALLOC FAILED"); 
    //     while(1); 
    // }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(1000); // Give WiFi time to settle
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    esp_link.begin(); 
    lidar.start(); 
    delay(1000);

    // delay(50000);

    // --- TASK PRIORITIES & CORE ASSIGNMENT ---
    // Rule: Higher number = Higher Priority.
    // Core 0: System & Comm | Core 1: Math & Sensors

    // -- Core 0 (Comms) --
    // IPC is critical (100Hz), keep high.
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC_Rx", 2048, NULL, 4, NULL, 0);
    // TCP is slow but needs to stay alive. Priority 3 is fine.
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP_Tx", 4096, NULL, 3, &TCP_Task_Handle, 0); 
    
    // Mission Planner (OPTIMIZATION: LOWERED PRIORITY)
    // Was 2, now 1. It runs in background and shouldn't starve TCP.
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 1, NULL, 0);

    // -- Core 1 (Heavy Lifting) --
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar_Read", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Lidar_Sync", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map_Comp", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global_Plan", 8192, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(Local_Planner_Task, "Local_Plan", 8192, NULL, 2, NULL, 1);
}

void loop() {}
