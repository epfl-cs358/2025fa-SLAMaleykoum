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
QueueHandle_t Lidar_Pose_Queue;
QueueHandle_t Path_Send_Queue;
TaskHandle_t  TCP_Task_Handle = NULL;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex;
SemaphoreHandle_t State_Mutex; 

// --- GLOBAL OBJECTS ---
const int GRID_SIZE_X = 25;
const int GRID_SIZE_Y = 25;
const float RESOLUTION = 0.4f;
// const int factor = 4;

const uint32_t RLE_BUFFER_SIZE = 10000;

BayesianOccupancyGrid* TheMap = nullptr;
MissionPlanner* mission_planner = nullptr; 
// BayesianOccupancyGrid* coarse = nullptr;

// --- STATE VARIABLES ---
Pose2D last_known_pose = {0,0,0,0};
PathMessage current_global_path = {{{0.f, 0.f}}, 1, 0, 0};
MissionGoal current_mission_goal = {{0.0f, 0.f, 0.f, 0}, EXPLORATION_MODE};
PathMessage current_local_path = {{}, 0, 0, 0};
bool new_goal_arrived = true;

// --- TRANSMISSION BUFFERS ---
uint8_t* Tx_Map_Buffer = nullptr; 
struct StateSnapshot {
    Pose2D pose;
    MissionGoal goal;
    PathMessage global_path;
    PathMessage local_path;
} Tx_Snapshot;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial);

// uint8_t* rle_buffer = nullptr;

// float pathA_X[] = {
//     0.00, 0.00, 2.00, 4.00, 6.00, 8.00,
//     8.50, 8.50, 8.50, 4.50, 2.50, 0.50,
//     -2.50, -3.50, -3.50
// };
// float pathA_Y[] = {
//     0.00, 0.30, 0.30, 0.30, 0.30, 0.30,
//     0.30, -3.70, -5.40, -5.40, -5.40, -5.40,
//     -5.40, -5.40, 0.30
// };
// const int pathA_size = 5;

// int i;

/*
bool downsample_bayesian_grid(BayesianOccupancyGrid& coarse, int factor) {
    const int Wc = GRID_SIZE_X / factor;
    const int Hc = GRID_SIZE_Y / factor;

    // On suppose que coarse a déjà été créé ou possède un buffer assez grand.
    coarse.grid_size_x = Wc;
    coarse.grid_size_y = Hc;
    coarse.grid_resolution = RESOLUTION * factor;

    const int8_t* fine_data = TheMap->get_map_data();
    int8_t* coarse_data = coarse.get_map_data();  // mémoire déjà allouée par toi

    // Parcours de la coarse map
    for (int cy = 0; cy < Hc; cy++) {
        for (int cx = 0; cx < Wc; cx++) {

            int occ_count = 0;
            int unk_count = 0;
            int total = 0;

            // Bloc dans la fine map
            for (int dy = 0; dy < factor; dy++) {
                for (int dx = 0; dx < factor; dx++) {
                    int fx = cx * factor + dx;
                    int fy = cy * factor + dy;

                    float lo = fine_data[fy * GRID_SIZE_X + fx];
                    if (lo > 0.4) occ_count++;
                    else if (lo > -0.2) unk_count++;
                    total++;
                }
            }

            float result_lo;
            if (occ_count > total * 0.2f)
                result_lo = 4.f;        // occupé
            else if (unk_count > total * 0.7f)
                result_lo = 0.f;        // inconnu
            else
                result_lo = -4.f;        // libre

            coarse_data[cy * Wc + cx] = result_lo;
        }
    }

    return true;
}
*/

bool goal_need_to_change(Pose2D pos, MissionGoal goal, const BayesianOccupancyGrid& TheMap) {
    float distance = sqrtf((pos.x-goal.target_pose.x)*(pos.x-goal.target_pose.x) 
            + (pos.y-goal.target_pose.y)*(pos.y-goal.target_pose.y));
    if (distance < 1) return true;

    int px, py, gx, gy;
    world_to_grid(pos.x, pos.y, px, py, TheMap.grid_resolution, TheMap.grid_size_x, TheMap.grid_size_y);
    world_to_grid(goal.target_pose.x, goal.target_pose.y, gx, gy, TheMap.grid_resolution, TheMap.grid_size_x, TheMap.grid_size_y);

    // int radius = 0.5 / TheMap.grid_resolution; // 1 for 1m
    int radius = 1;

    for (int x = gx - radius ; x < gx + radius; ++x) {
        for (int y = gy - radius ; y < gy + radius ; ++y) {
            float point = TheMap.get_cell_probability(x, y);
            if (point == 0.5) return false;
        }
    }

    return true;
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
            Tx_Snapshot.local_path = current_local_path;
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
        
        uint16_t gplen = Tx_Snapshot.global_path.current_length;
        if(gplen > MAX_PATH_LENGTH) gplen = 0;
        memcpy(&header[6], &gplen, 2);

        uint16_t lplen = Tx_Snapshot.local_path.current_length;
        if(lplen > MAX_LOCAL_PATH_LENGTH) lplen = 0;
        memcpy(&header[8], &lplen, 2);

        memcpy(&header[10], &Tx_Snapshot.pose.x, 4);
        memcpy(&header[14], &Tx_Snapshot.pose.y, 4);
        memcpy(&header[18], &Tx_Snapshot.pose.theta, 4);

        memcpy(&header[22], &Tx_Snapshot.goal.target_pose.x, 4);
        memcpy(&header[26], &Tx_Snapshot.goal.target_pose.y, 4);
        memcpy(&header[30], &Tx_Snapshot.goal.target_pose.theta, 4);

        if (tcpClient.connected()) {
            tcpClient.write(header, 64);
            if(gplen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            if(lplen > 0) tcpClient.write((uint8_t*)Tx_Snapshot.local_path.path, lplen * sizeof(Waypoint));
            
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
                TheMap->update_map(synced_data); 
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
// MISSION PLANNER TASK
// =============================================================
void Mission_Planner_Task(void* parameter) {
    // Serial.println("[Task] Mission Planner Started");
    
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
        if (mission_planner->get_current_state() != RETURN_HOME 
            && goal_need_to_change(current_p, current_mission_goal, *TheMap)) {
            
            MissionGoal new_goal = mission_planner->update_goal(current_p, *TheMap);
            new_goal_arrived = true;
            
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
    // Serial.println("[Task] Global Planner Started (A*)");
    GlobalPlanner planner; 
    
    Pose2D local_pose;
    MissionGoal global_goal;

    while (1) {
        PathMessage pathMsg;        
        
        if (new_goal_arrived) {
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                local_pose = last_known_pose;
                global_goal = current_mission_goal;
                xSemaphoreGive(State_Mutex); 
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        
        // if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        
            // downsample_bayesian_grid(*coarse, factor);
            pathMsg = planner.generate_path(local_pose, global_goal, *TheMap); 
            
            // if (pathMsg.current_length > 0) {
            //      Serial.printf("A* Path Found! Length: %d waypoints\n", pathMsg.current_length);
                 
            //      Serial.printf("Start: (%.2f, %.2f) | Goal: (%.2f, %.2f)\n", 
            //                    pathMsg.path[0].x, pathMsg.path[0].y, 
            //                    pathMsg.path[pathMsg.current_length-1].x, 
            //                    pathMsg.path[pathMsg.current_length-1].y);
            // } else {
            //      Serial.println("A* Path NOT Found or length is zero.");
            // }
            
            //     xSemaphoreGive(Bayesian_Grid_Mutex);
            // } else {
            //     vTaskDelay(pdMS_TO_TICKS(100));
            //     continue; 
            // }
            
            // pathMsg.current_length = pathA_size;
            // pathMsg.timestamp_ms = millis();
            // pathMsg.path_id = 0;
            
            // for (int i = 0; i < 5; i++) {
            //     pathMsg.path[i].x = pathA_X[i];
            //     pathMsg.path[i].y = pathA_Y[i];
            // }
            
            // Also update the global state for TCP
            current_global_path = pathMsg;
            // i = 0;
            
            // if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            //     current_global_path = pathMsg; 
            //     i = 0;
            //     xSemaphoreGive(State_Mutex);
            // }
            
            esp_link.sendPath(current_global_path, GLOBAL);
            new_goal_arrived = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(800));
    }
}

/*
void Local_Planner_Task(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    Serial.println("[Task] Local Planner Started (A*)");
    LocalPlanner local_planner(SEARCH_BOUND_M); 
    
    Pose2D local_pose;
    Waypoint local_goal;

    while (1) {
    //     PathMessage local_pathMsg;
        
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE && i < current_global_path.current_length) {
    //         local_pose = last_known_pose;
    //         local_goal = current_global_path.path[i++];
            xSemaphoreGive(State_Mutex); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); 
            continue;
        }

    //     if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            
    //         local_pathMsg = local_planner.compute_local_path(local_pose, local_goal, *TheMap);
            
    //         xSemaphoreGive(Bayesian_Grid_Mutex);
    //     } else {
    //         vTaskDelay(pdMS_TO_TICKS(100));
    //         continue; 
    //     }
        
    //     if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    //         current_local_path = local_pathMsg; 
    //         xSemaphoreGive(State_Mutex);
    //     }
        
    //     esp_link.sendPath(local_pathMsg, LOCAL); 
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}*/

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    // Serial.println("\n--- ESP1 FULL NODE ---");

    // rle_buffer = (uint8_t*) heap_caps_malloc(10000, MALLOC_CAP_SPIRAM);

    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    State_Mutex = xSemaphoreCreateMutex();

    TheMap = new BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
    // TheMap = (BayesianOccupancyGrid*) heap_caps_malloc(sizeof(BayesianOccupancyGrid), MALLOC_CAP_SPIRAM);
    // new (TheMap) BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);

    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});
    // current_mission_goal.target_pose = {0.0f, 0.f, 0.f, 0};
    // current_mission_goal.type = EXPLORATION_MODE;

    // coarse = new BayesianOccupancyGrid(RESOLUTION * factor, GRID_SIZE_X / factor, 
    //     GRID_SIZE_Y / factor);
    // coarse = (BayesianOccupancyGrid*) heap_caps_malloc(sizeof(BayesianOccupancyGrid), MALLOC_CAP_SPIRAM);
    // new (coarse) BayesianOccupancyGrid(RESOLUTION * factor, GRID_SIZE_X / factor, GRID_SIZE_Y / factor);
    
    if (!TheMap || !mission_planner) { 
        // Serial.println("MEMORY ALLOC FAILED"); 
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

    delay(30000);

    // Queue Sizes: Increased Lidar buffer slightly to handle bursts
    Lidar_Buffer_Queue = xQueueCreate(2, sizeof(LiDARScan));
    Lidar_Pose_Queue   = xQueueCreate(2, sizeof(SyncedScan));
    Path_Send_Queue    = xQueueCreate(1, sizeof(PathMessage));

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
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 2048, NULL, 1, NULL, 0);

    // -- Core 1 (Heavy Lifting) --
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar_Read", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Lidar_Sync", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map_Comp", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global_Plan", 8192, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(Local_Planner_Task, "Local_Plan", 8192, NULL, 2, NULL, 1);
}

void loop() {}
