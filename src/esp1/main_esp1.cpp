#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- Include Headers ---
#include "common/data_types.h"
#include "common/esp_link.h"
#include "esp1/hardware/lidar.h"
#include "esp1/mapping/bayesian_grid.h"
#include "esp1/planning/global_planner.h"
#include "esp1/planning/mission_planner.h"
#include "../../include/common/utils.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#define TCP_HEADER_SIZE 96

// --- NETWORK CONFIG ---
const char* ssid = "LIDAR_AP";
const char* password = "l1darpass";
const uint16_t TCP_PORT = 9000;

WiFiServer tcpServer(TCP_PORT);

// --- RTOS HANDLES ---
QueueHandle_t Lidar_Buffer_Queue;
QueueHandle_t Path_Send_Queue;

// --- MAILBOX for scans ---
SyncedScan Shared_Mailbox_Scan;
SemaphoreHandle_t Mailbox_Mutex;
volatile bool Mailbox_Has_New_Data = false;

TaskHandle_t GPlan_Task_Handle = NULL;
TaskHandle_t MPlan_Task_Handle = NULL;
TaskHandle_t Lidar_Task_Handle = NULL;
TaskHandle_t TCP_Task_Handle = NULL;
TaskHandle_t Sync_Task_Handle = NULL;
TaskHandle_t Map_Task_Handle = NULL;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex;
SemaphoreHandle_t State_Mutex; 
SemaphoreHandle_t Invalid_Mutex;

// PingPong Goal/Path
SemaphoreHandle_t Token_Goal_Planner; // Start with 1 (Mission Planner goes first)
SemaphoreHandle_t Token_Path_Planner; // Start with 0

// --- GLOBAL OBJECTS ---
const int GRID_SIZE_X = 70;
const int GRID_SIZE_Y = 70;
const float RESOLUTION = 0.15f;

const uint32_t RLE_BUFFER_SIZE = 10000;

BayesianOccupancyGrid* TheMap = nullptr;
MissionPlanner* mission_planner = nullptr; 
GlobalPlannerWorkspace* gp_workspace = nullptr;

// --- STATE VARIABLES ---
Pose2D last_known_pose = {0,0,0,0};
PathMessage current_global_path = {{{0.f, 0.f}}, 1, 0, 0};
MissionGoal current_mission_goal = {{0.0f, 0.f, 0.f, 0}, EXPLORATION_MODE};
bool new_goal_arrived = true;
bool global_path_invalid = false;

// --- SYSTEM HEALTH MONITOR ---
SystemHealth sys_health = {0, 0, 0, 0, 0, 0};
uint32_t last_ipc_activity_ts = 0;

// --- DIAGNOSTIC COUNTERS ---
struct DiagnosticCounters {
    uint32_t lidar_scans_received;
    uint32_t sync_frames_processed;
    uint32_t sync_frames_dropped;
    uint32_t map_updates_completed;
    uint32_t mutex_failures_sync;
    uint32_t mutex_failures_map;
    uint32_t nan_detections;
    uint32_t invalid_poses;
} diag_counters = {0, 0, 0, 0, 0, 0, 0, 0};

// --- TRANSMISSION BUFFERS ---
struct StateSnapshot {
    Pose2D pose;
    MissionGoal goal;
    PathMessage global_path;
    uint8_t state;
} Tx_Snapshot;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial);

InvalidGoals invalid_goals;

// =============================================================
// TRACE LOGGING SYSTEM
// =============================================================
#define ID_LIDAR     1
#define ID_SYNC      2
#define ID_MAP       3
#define ID_IPC       4
#define ID_GPLAN     5
#define ID_MPLAN     6
#define ID_TCP       7

#define EVENT_START  1
#define EVENT_END    0

#define MAX_TRACE_EVENTS 200

struct TaskEvent {
    uint32_t timestamp_us;
    uint8_t task_id;
    uint8_t type;
    uint8_t core_id;
    uint8_t padding;
} __attribute__((packed));

TaskEvent trace_buffer[MAX_TRACE_EVENTS];
volatile uint16_t trace_head = 0;
portMUX_TYPE trace_mux = portMUX_INITIALIZER_UNLOCKED;

void log_event(uint8_t task_id, uint8_t type) {
    portENTER_CRITICAL(&trace_mux);
    uint16_t idx = trace_head % MAX_TRACE_EVENTS;
    trace_buffer[idx].timestamp_us = micros();
    trace_buffer[idx].task_id = task_id;
    trace_buffer[idx].type = type;
    trace_buffer[idx].core_id = xPortGetCoreID();
    trace_buffer[idx].padding = 0;
    trace_head++;
    portEXIT_CRITICAL(&trace_mux);
}

// =============================================================
// HELPER FUNCTIONS
// =============================================================
bool is_pose_valid(const Pose2D& pose) {
    if (isnan(pose.x) || isnan(pose.y) || isnan(pose.theta)) return false;
    if (isinf(pose.x) || isinf(pose.y) || isinf(pose.theta)) return false;
    return true;
}

bool is_scan_valid(const LiDARScan& scan) {
    if (scan.count == 0 || scan.count > MAX_LIDAR_POINTS) return false;
    return true;
}

// =============================================================
// TASK 1: IPC RECEIVE
// =============================================================
void IPC_Receive_Task(void* parameter) {
    while (1) {
        esp_link.poll();
        Pose2D incoming_pose;
        
        if (esp_link.get_pos(incoming_pose)) {
            uint32_t now = millis();
            
            if (!is_pose_valid(incoming_pose)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            
            if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                float x_diff = incoming_pose.x -  last_known_pose.x;
                float y_diff = incoming_pose.y -  last_known_pose.y;
                float theta_diff = incoming_pose.theta - last_known_pose.theta;
                if (((x_diff) * (x_diff) + (y_diff) * (y_diff)) > 1.0f) {
                    // Discard obviously wrong pose
                    xSemaphoreGive(Pose_Mutex);
                    continue;
                }
                // Normalize to [-π, π]
                while (theta_diff > M_PI) theta_diff -= 2.0f * M_PI;
                while (theta_diff < -M_PI) theta_diff += 2.0f * M_PI;

                if (fabsf(theta_diff) > 0.25f) {  // Max 28.6° per update
                    xSemaphoreGive(Pose_Mutex);
                continue;
                }
                last_known_pose = incoming_pose;
                sys_health.last_esp2_packet_ms = millis(); 
                last_ipc_activity_ts = millis();
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
        if (scanComplete) {
            scan.count = 0;
            lastAngleESP = 0.0f;
            scanComplete = false;
        }
        
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            log_event(ID_LIDAR, EVENT_START);
            if (scan.count > 10 && is_scan_valid(scan)) {
                if (xQueueSend(Lidar_Buffer_Queue, &scan, 0) == pdTRUE) {
                    diag_counters.lidar_scans_received++;
                } else {
                    diag_counters.sync_frames_dropped++;
                }
            }
            log_event(ID_LIDAR, EVENT_END);
        }
        vTaskDelay(xDelay);
    }
}

// =============================================================
// TASK 3: SYNC FUSION
// =============================================================
void Lidar_Sync_Map_Task(void* parameter) {
    static LiDARScan local_scan_buffer;

    while (1) {
        if (xQueueReceive(Lidar_Buffer_Queue, &local_scan_buffer, portMAX_DELAY) == pdPASS) {
            log_event(ID_SYNC, EVENT_START);
            
            Pose2D current_pose;
            if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                current_pose = last_known_pose;
                xSemaphoreGive(Pose_Mutex);
            } else {
                diag_counters.mutex_failures_sync++;
                log_event(ID_SYNC, EVENT_END);
                continue;
            }

            if (!is_pose_valid(current_pose)) {
                log_event(ID_SYNC, EVENT_END);
                continue;
            }

            SyncedScan local_synced_scan = {local_scan_buffer, current_pose};

            if (xSemaphoreTake(Mailbox_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                Shared_Mailbox_Scan = local_synced_scan;
                Mailbox_Has_New_Data = true;
                xSemaphoreGive(Mailbox_Mutex);
                diag_counters.sync_frames_processed++;
            } else {
                diag_counters.mutex_failures_sync++;
            }
            log_event(ID_SYNC, EVENT_END);
        }
    }
}

// =============================================================
// TASK 4: BAYESIAN GRID UPDATE
// =============================================================
void Bayesian_Grid_Task(void* parameter) {
    static SyncedScan local_map_scan;
    bool has_data = false;

    while (1) {
        log_event(ID_MAP, EVENT_START);

        if (xSemaphoreTake(Mailbox_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (Mailbox_Has_New_Data) {
                local_map_scan = Shared_Mailbox_Scan;
                Mailbox_Has_New_Data = false;
                has_data = true;
            } else {
                has_data = false;
            }
            xSemaphoreGive(Mailbox_Mutex);
        } else {
            diag_counters.mutex_failures_map++;
            has_data = false;
        }

        if (has_data) {
            if (is_pose_valid(local_map_scan.pose) && is_scan_valid(local_map_scan.scan)) {
                if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    TheMap->update_map(local_map_scan);
                    diag_counters.map_updates_completed++;
                    xSemaphoreGive(Bayesian_Grid_Mutex);
                } else {
                    diag_counters.mutex_failures_map++;
                }
            }
        }
        log_event(ID_MAP, EVENT_END);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================
// TASK 5: MISSION PLANNER (PING)
// =============================================================
void Mission_Planner_Task(void* parameter) {
    while (1) {
        // 1. WAIT FOR TOKEN (Blocks until Global Planner is done)
        xSemaphoreTake(Token_Goal_Planner, portMAX_DELAY);

        log_event(ID_MPLAN, EVENT_START);
        
        Pose2D current_pose;
        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_pose = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        }

        if (is_pose_valid(current_pose)) {
            InvalidGoals local_invalid;
            if (xSemaphoreTake(Invalid_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                local_invalid = invalid_goals;
                xSemaphoreGive(Invalid_Mutex);
            }

            MissionGoal new_goal;
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                new_goal = mission_planner->update_goal(
                    current_pose, *TheMap, global_path_invalid, local_invalid
                );
                xSemaphoreGive(Bayesian_Grid_Mutex);
                
                // Update Global State
                if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    current_mission_goal = new_goal;
                    new_goal_arrived = true; 
                    xSemaphoreGive(State_Mutex);
                }
            }
        }
        
        log_event(ID_MPLAN, EVENT_END);

        // 2. PASS TOKEN TO PATH PLANNER
        xSemaphoreGive(Token_Path_Planner);
        
        // Slight delay to allow context switch cleanly
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// =============================================================
// TASK 6: GLOBAL PLANNER (PONG)
// =============================================================
void Global_Planner_Task(void* parameter) {
    GlobalPlanner gplan;

    while (1) {
        // 1. WAIT FOR TOKEN (Blocks until Mission Planner is done)
        xSemaphoreTake(Token_Path_Planner, portMAX_DELAY);
        
        log_event(ID_GPLAN, EVENT_START);

        // We only plan if a "new goal" signal is high (or we just want to re-verify path)
        // In ping-pong, we usually check every time.
        
        Pose2D current_pose;
        MissionGoal current_goal;

        // Fetch Data
        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_pose = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        }
        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_goal = current_mission_goal;
            xSemaphoreGive(State_Mutex);
        }

        if (is_pose_valid(current_pose) && is_pose_valid(current_goal.target_pose)) {
            PathMessage new_path;
            
            // Compute Path
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                new_path = gplan.generate_path(current_pose, current_goal, *TheMap, gp_workspace);
                xSemaphoreGive(Bayesian_Grid_Mutex);
            }

            // Handle Result
            if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (new_path.current_length > 0) {
                    // SUCCESS
                    esp_link.sendPath(new_path);
                    current_global_path = new_path;
                    global_path_invalid = false;
                    new_goal_arrived = false;
                } else {
                    // FAILURE
                    global_path_invalid = true;
                    
                    // NEW: Blacklist Goal + Neighbors
                    if (xSemaphoreTake(Invalid_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        int gx, gy;
                        world_to_grid(current_goal.target_pose.x, current_goal.target_pose.y,
                                    gx, gy, RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
                        
                        // Add goal AND 8-neighbors to invalid list
                        for(int dy=-1; dy<=1; dy++) {
                            for(int dx=-1; dx<=1; dx++) {
                                if(invalid_goals.size < MAX_INVALID_GOALS) {
                                    invalid_goals.lasts[invalid_goals.size].x = gx + dx;
                                    invalid_goals.lasts[invalid_goals.size].y = gy + dy;
                                    invalid_goals.size++;
                                }
                            }
                        }
                        xSemaphoreGive(Invalid_Mutex);
                    }
                }
                xSemaphoreGive(State_Mutex);
            }
        }

        log_event(ID_GPLAN, EVENT_END);

        // 2. PASS TOKEN BACK TO MISSION PLANNER
        xSemaphoreGive(Token_Goal_Planner);
        
        // Give it a second to run
        vTaskDelay(pdMS_TO_TICKS(1500)); 
    }
}

// =============================================================
// TASK 7: TCP TRANSMIT (FIXED BLOCKING & RACE)
// =============================================================
void TCP_Transmit_Task(void* parameter) {
    WiFiClient tcpClient;
    static uint8_t rle_buffer[RLE_BUFFER_SIZE];
    static TaskEvent trace_buffer_copy[MAX_TRACE_EVENTS];
    uint8_t header[TCP_HEADER_SIZE]; 
    
    const TickType_t TelemetryFreq = pdMS_TO_TICKS(100); 
    const TickType_t MapFreq = pdMS_TO_TICKS(3000);
    
    TickType_t lastMapTime = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, TelemetryFreq);
        log_event(ID_TCP, EVENT_START);

        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
            } else {
                log_event(ID_TCP, EVENT_END);
                continue;
            }
        }

        // Snapshot
        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            Tx_Snapshot.global_path = current_global_path;
            Tx_Snapshot.goal = current_mission_goal;
            Tx_Snapshot.state = (uint8_t)current_mission_goal.type;
            xSemaphoreGive(State_Mutex);
        }

        // Map Compression
        bool send_map_now = (xTaskGetTickCount() - lastMapTime > MapFreq);
        uint32_t payload_map_sz = 0;

        if (send_map_now) {
            const int8_t* raw_map = TheMap->get_map_data();
            uint32_t total_cells = TheMap->get_total_size();
            uint32_t rle_idx = 0;
            uint32_t raw_idx = 0;

            while (raw_idx < total_cells && rle_idx < RLE_BUFFER_SIZE - 2) {
                int8_t current_val = raw_map[raw_idx];
                uint8_t count = 0;
                while (raw_idx < total_cells && raw_map[raw_idx] == current_val && count < 255) {
                    count++;
                    raw_idx++;
                }
                rle_buffer[rle_idx++] = count;
                rle_buffer[rle_idx++] = (uint8_t)current_val;
            }
            payload_map_sz = rle_idx;
        }

        // Trace Events - CRITICAL FIX for Race Condition
        uint16_t events_to_send = 0;
        portENTER_CRITICAL(&trace_mux);
        if (trace_head > 0) {
            events_to_send = (trace_head < MAX_TRACE_EVENTS) ? trace_head : MAX_TRACE_EVENTS;
            // Drain logic: Copy only what we need
            if (trace_head >= MAX_TRACE_EVENTS) {
                uint16_t start_idx = trace_head % MAX_TRACE_EVENTS;
                uint16_t first_part = MAX_TRACE_EVENTS - start_idx;
                memcpy(trace_buffer_copy, &trace_buffer[start_idx], first_part * sizeof(TaskEvent));
                memcpy(&trace_buffer_copy[first_part], trace_buffer, (events_to_send - first_part) * sizeof(TaskEvent));
            } else {
                memcpy(trace_buffer_copy, trace_buffer, events_to_send * sizeof(TaskEvent));
            }
            trace_head = 0; // Reset safe inside lock
        }
        portEXIT_CRITICAL(&trace_mux);

        // Header Construction
        memset(header, 0, TCP_HEADER_SIZE);
        header[0] = 0xBE; header[1] = 0xEF;
        memcpy(&header[2], &payload_map_sz, 4);
        
        uint8_t gplen = Tx_Snapshot.global_path.current_length;
        if(gplen > MAX_PATH_LENGTH) gplen = 0;
        uint16_t gplen_2bytes = (0 << 8) | gplen;
        memcpy(&header[6], &gplen_2bytes, 2);

        memcpy(&header[8], &Tx_Snapshot.pose.x, 4);
        memcpy(&header[12], &Tx_Snapshot.pose.y, 4);
        memcpy(&header[16], &Tx_Snapshot.pose.theta, 4);
        memcpy(&header[20], &Tx_Snapshot.goal.target_pose.x, 4);
        memcpy(&header[24], &Tx_Snapshot.goal.target_pose.y, 4);
        memcpy(&header[28], &Tx_Snapshot.goal.target_pose.theta, 4);
        header[32] = Tx_Snapshot.state;

        sys_health.free_heap = esp_get_free_heap_size();
        sys_health.min_free_heap = esp_get_minimum_free_heap_size();
        sys_health.stack_min_tcp = uxTaskGetStackHighWaterMark(NULL) * 4;
        
        memcpy(&header[33], &sys_health, sizeof(SystemHealth));

        // Sending Logic - CRITICAL FIX: Chunking to prevent Blocking > WDT
        if (tcpClient.connected()) {
            tcpClient.write(header, TCP_HEADER_SIZE);
            tcpClient.write((uint8_t*)&events_to_send, 2);
            
            if(gplen > 0) 
                tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            
            // Chunk the map sending
            if (payload_map_sz > 0) {
                size_t sent = 0;
                const size_t CHUNK_SIZE = 1024; // 1KB chunks
                
                while(sent < payload_map_sz) {
                    size_t remaining = payload_map_sz - sent;
                    size_t to_send = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
                    
                    // Prevent WDT reset if writing blocks
                    tcpClient.write(&rle_buffer[sent], to_send);
                    sent += to_send;
                    vTaskDelay(1); 
                }
                lastMapTime = xTaskGetTickCount();
            }
            
            if (events_to_send > 0) {
                tcpClient.write((uint8_t*)trace_buffer_copy, events_to_send * sizeof(TaskEvent));
            }
        }

        log_event(ID_TCP, EVENT_END);
    }
}

// =============================================================
// SETUP
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== ESP1 Autonomous Robot Starting ===");

    gp_workspace = new GlobalPlannerWorkspace();
    Lidar_Buffer_Queue = xQueueCreate(3, sizeof(LiDARScan));
    Path_Send_Queue = xQueueCreate(1, sizeof(PathMessage));

    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    State_Mutex = xSemaphoreCreateMutex();
    Invalid_Mutex = xSemaphoreCreateMutex();
    Mailbox_Mutex = xSemaphoreCreateMutex();

    // Ping Pong Tokens
    Token_Goal_Planner = xSemaphoreCreateBinary();
    Token_Path_Planner = xSemaphoreCreateBinary();

    // Start the cycle: Give token to Goal Planner first
    xSemaphoreGive(Token_Goal_Planner);

    TheMap = new BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});

    current_mission_goal.target_pose = {0.0f, 0.0f, 0.0f, 0};
    current_mission_goal.type = EXPLORATION_MODE;
    new_goal_arrived = true; 

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    // --- Hardware Init ---
    esp_link.begin(); 
    lidar.start();

    // --- TASK CREATION ---
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar", 3072, NULL, 4, &Lidar_Task_Handle, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Sync", 8192, NULL, 4, &Sync_Task_Handle, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map", 20480, NULL, 3, &Map_Task_Handle, 1);

    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP", 8192, NULL, 1, &TCP_Task_Handle, 0);
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 2, &MPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global", 8192, NULL, 2, &GPlan_Task_Handle, 0);

    Serial.println("=== System ready ===");
}

void loop() {}