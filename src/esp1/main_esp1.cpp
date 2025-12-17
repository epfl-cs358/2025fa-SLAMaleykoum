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

// Double Buffering Queues
QueueHandle_t Lidar_Pose_Queue_Ready; // Holds pointers to FULL buffers (Sync -> Map)
QueueHandle_t Lidar_Pose_Queue_Free;  // Holds pointers to EMPTY buffers (Map -> Sync)

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
SemaphoreHandle_t xSemaphoreGOAL;
SemaphoreHandle_t xSemaphorePATH;

// --- GLOBAL OBJECTS ---
const int GRID_SIZE_X = 70;
const int GRID_SIZE_Y = 70;
const float RESOLUTION = 0.2f;

const uint32_t RLE_BUFFER_SIZE = 10000;

BayesianOccupancyGrid* TheMap = nullptr;
MissionPlanner* mission_planner = nullptr; 
GlobalPlannerWorkspace* gp_workspace = nullptr;

// --- DOUBLE BUFFERING POOL ---
SyncedScan* Scan_Buffer_1 = nullptr;
SyncedScan* Scan_Buffer_2 = nullptr;

// --- STATE VARIABLES ---
Pose2D last_known_pose = {0,0,0,0};
PathMessage current_global_path = {{{0.f, 0.f}}, 1, 0, 0};
MissionGoal current_mission_goal = {{0.0f, 0.f, 0.f, 0}, EXPLORATION_MODE};
bool new_goal_arrived = true;
bool global_path_invalid = false;

// --- SYSTEM HEALTH MONITOR ---
SystemHealth sys_health = {0, 0, 0, 0, 0, 0};
uint32_t last_ipc_activity_ts = 0;

// --- DIAGNOSTIC COUNTERS (NEW) ---
struct DiagnosticCounters {
    uint32_t lidar_scans_received;
    uint32_t sync_frames_processed;
    uint32_t sync_frames_dropped;
    uint32_t map_updates_completed;
    uint32_t mutex_failures_sync;
    uint32_t mutex_failures_map;
    uint32_t queue_ready_depth;
    uint32_t queue_free_depth;
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
// Task IDs for profiling
#define ID_LIDAR     1
#define ID_SYNC      2
#define ID_MAP       3
#define ID_IPC       4
#define ID_GPLAN     5
#define ID_MPLAN     6
#define ID_TCP       7

// Event types
#define EVENT_START  1
#define EVENT_END    0

// Trace buffer
#define MAX_TRACE_EVENTS 200

struct TaskEvent {
    uint32_t timestamp_us;
    uint8_t task_id;
    uint8_t type;        // 0=END, 1=START
    uint8_t core_id;
    uint8_t padding;
} __attribute__((packed));

TaskEvent trace_buffer[MAX_TRACE_EVENTS];
volatile uint16_t trace_head = 0;
portMUX_TYPE trace_mux = portMUX_INITIALIZER_UNLOCKED;

// Log a task event (call at start/end of task work)
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
// TASK 1: IPC RECEIVE
// =============================================================
void IPC_Receive_Task(void* parameter) {
    while (1) {
        log_event(ID_IPC, EVENT_START);
        
        esp_link.poll();
        Pose2D incoming_pose;
        if (esp_link.get_pos(incoming_pose)) {
            if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                float x_diff = incoming_pose.x -  last_known_pose.x;
                float y_diff = incoming_pose.y -  last_known_pose.y;
                if (((x_diff) * (x_diff) + (y_diff) * (y_diff)) > 1.0f) {
                    // Discard obviously wrong pose
                    xSemaphoreGive(Pose_Mutex);
                    log_event(ID_IPC, EVENT_END);
                    continue;
                }
                last_known_pose = incoming_pose;
                sys_health.last_esp2_packet_ms = millis(); 
                last_ipc_activity_ts = millis();
                xSemaphoreGive(Pose_Mutex);
            }
        }
        
        log_event(ID_IPC, EVENT_END);
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
        scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            log_event(ID_LIDAR, EVENT_START);
            
            if (scan.count > 10) { 
                xQueueSend(Lidar_Buffer_Queue, &scan, 0);
                diag_counters.lidar_scans_received++;
            }
            scan.count = 0;
            lastAngleESP = 0.0f;
            
            log_event(ID_LIDAR, EVENT_END);
        }
        vTaskDelay(xDelay);
    }
}

// =============================================================
// TASK 3: SYNC FUSION (FIXED - NO BLOCKING ON MUTEX)
// =============================================================
void Lidar_Sync_Map_Task(void* parameter) {
    LiDARScan* scan_buffer = new LiDARScan();
    SyncedScan* current_work_buffer = nullptr;
    int local_counter = 0;

    while (1) {
        if (xQueueReceive(Lidar_Buffer_Queue, scan_buffer, portMAX_DELAY) == pdPASS) {
            
            log_event(ID_SYNC, EVENT_START);
            
            local_counter++;
            if (local_counter % 3 != 0) {
                log_event(ID_SYNC, EVENT_END);
                continue;
            }

            // Get free buffer
            if (xQueueReceive(Lidar_Pose_Queue_Free, &current_work_buffer, 0) == pdPASS) {
                
                // Fill scan data
                current_work_buffer->scan = *scan_buffer;
                
                // If we can't get it quickly, use last known pose
                if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    current_work_buffer->pose = last_known_pose;
                    xSemaphoreGive(Pose_Mutex);
                } else {
                    // Fallback: use last known pose without mutex
                    current_work_buffer->pose = last_known_pose;
                    diag_counters.mutex_failures_sync++;
                }

                // Always send to map task
                xQueueSend(Lidar_Pose_Queue_Ready, &current_work_buffer, 0);
                diag_counters.sync_frames_processed++;
                
                // Update queue depths for diagnostics
                diag_counters.queue_ready_depth = uxQueueMessagesWaiting(Lidar_Pose_Queue_Ready);
                diag_counters.queue_free_depth = uxQueueMessagesWaiting(Lidar_Pose_Queue_Free);
                
            } else {
                diag_counters.sync_frames_dropped++;
            }
            
            log_event(ID_SYNC, EVENT_END);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =============================================================
// TASK 4: BAYESIAN MAPPING (FIXED - ALWAYS LOG END)
// =============================================================
void Bayesian_Grid_Task(void* parameter) {
    SyncedScan* incoming_data_ptr = nullptr;
    
    while (1) {
        if (xQueueReceive(Lidar_Pose_Queue_Ready, &incoming_data_ptr, pdMS_TO_TICKS(100)) == pdPASS) {
            
            log_event(ID_MAP, EVENT_START);
            
            bool map_updated = false;
            
            // Try to update map
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                uint32_t t0 = micros();
                
                TheMap->update_map(*incoming_data_ptr); 
                
                sys_health.map_update_time_us = micros() - t0;
                map_updated = true;
                diag_counters.map_updates_completed++;
                
                xSemaphoreGive(Bayesian_Grid_Mutex);
            } else {
                diag_counters.mutex_failures_map++;
            }
            
            // Update pose snapshot (non-critical, try briefly)
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                last_known_pose = incoming_data_ptr->pose;
                xSemaphoreGive(State_Mutex);
            }

            // ALWAYS recycle buffer
            xQueueSend(Lidar_Pose_Queue_Free, &incoming_data_ptr, 0);
            log_event(ID_MAP, EVENT_END);
            
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            // Queue receive timeout - this is normal, just wait
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// =============================================================
// MISSION PLANNER TASK
// =============================================================
void Mission_Planner_Task(void* parameter) {
    InvalidGoals mission_invalid;

    // Wait for first map data before starting
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1) {
        xSemaphoreTake(xSemaphoreGOAL, portMAX_DELAY);

        log_event(ID_MPLAN, EVENT_START);

        Pose2D current_p;
        
        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            log_event(ID_MPLAN, EVENT_END);
            xSemaphoreGive(xSemaphorePATH);
            vTaskDelay(pdMS_TO_TICKS(100)); 
            continue;
        }

        bool planner_fail = false;
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            planner_fail = global_path_invalid;
            xSemaphoreGive(State_Mutex);
        }

        uint32_t t0 = micros();
        if (xSemaphoreTake(Invalid_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            mission_invalid = invalid_goals;
            xSemaphoreGive(Invalid_Mutex);
        } else {
            log_event(ID_MPLAN, EVENT_END);
            xSemaphoreGive(xSemaphorePATH);
            continue;
        }

        MissionGoal new_goal = mission_planner->update_goal(current_p, *TheMap, planner_fail, mission_invalid);
        sys_health.mission_plan_time_us = micros() - t0;

        if (new_goal.target_pose.x != current_mission_goal.target_pose.x || 
            new_goal.target_pose.y != current_mission_goal.target_pose.y ||
            new_goal.type != current_mission_goal.type) 
        {
            if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                current_mission_goal = new_goal;
                new_goal_arrived = true;
                xSemaphoreGive(State_Mutex);
            }
        }

        log_event(ID_MPLAN, EVENT_END);
        
        xSemaphoreGive(xSemaphorePATH);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================
// GLOBAL PLANNER TASK
// =============================================================
void Global_Planner_Task(void* parameter) {
    GlobalPlanner planner; 
    Pose2D local_pose;
    MissionGoal local_goal_copy;

    while (1) {
        xSemaphoreTake(xSemaphorePATH, portMAX_DELAY);

        log_event(ID_GPLAN, EVENT_START);

        PathMessage pathMsg = {{{0}}, 0, 0, 0};
        bool goal_pending = false;
        
        // Read flag + goal
        if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            goal_pending = new_goal_arrived;
            if (goal_pending) {
                local_pose = last_known_pose;
                local_goal_copy = current_mission_goal;
            }
            xSemaphoreGive(State_Mutex); 
        }
        
        if (goal_pending) {
            uint32_t t0 = millis();
            
            // A* computation (can take 300ms)
            pathMsg = planner.generate_path(local_pose, local_goal_copy, *TheMap, gp_workspace);
            
            uint32_t duration = millis() - t0;
            sys_health.global_plan_time_us = duration * 1000;

            // Check if goal changed during A*
            if(xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                bool goal_changed = (
                    current_mission_goal.target_pose.x != local_goal_copy.target_pose.x ||
                    current_mission_goal.target_pose.y != local_goal_copy.target_pose.y ||
                    current_mission_goal.type != local_goal_copy.type
                );
                
                if (goal_changed) {
                    // Goal changed during A* - don't update anything
                } else {
                    // Goal is still current - process result
                    new_goal_arrived = false;
                    
                    if (pathMsg.current_length == 0) {
                        // Path planning failed
                        global_path_invalid = true;
                        current_global_path.current_length = 0;
                        
                        if (xSemaphoreTake(Invalid_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                            if (invalid_goals.size >= MAX_INVALID_GOALS) invalid_goals.size = 0;

                            int gx, gy;
                            world_to_grid(local_goal_copy.target_pose.x, local_goal_copy.target_pose.y, 
                                        gx, gy, RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
                            invalid_goals.lasts[invalid_goals.size].x = (float)gx;
                            invalid_goals.lasts[invalid_goals.size].y = (float)gy;

                            invalid_goals.size++;
                            xSemaphoreGive(Invalid_Mutex);
                        }
                    } else {
                        // Success
                        global_path_invalid = false;
                        current_global_path = pathMsg;
                        esp_link.sendPath(current_global_path);
                    }
                }
                
                xSemaphoreGive(State_Mutex);
            }
        }
        
        log_event(ID_GPLAN, EVENT_END);
        
        xSemaphoreGive(xSemaphoreGOAL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================
// TASK 5: TCP TRANSMIT (WITH DIAGNOSTICS)
// =============================================================
static uint8_t rle_buffer[10000];
static TaskEvent trace_buffer_copy[MAX_TRACE_EVENTS];

void TCP_Transmit_Task(void* parameter) {
    WiFiClient tcpClient;
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

        // Quick snapshot of state
        if (xSemaphoreTake(State_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            Tx_Snapshot.global_path = current_global_path;
            Tx_Snapshot.goal = current_mission_goal;
            Tx_Snapshot.state = (uint8_t)current_mission_goal.type;
            xSemaphoreGive(State_Mutex);
        }

        // RLE compress map
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

        // Extract trace events
        uint16_t events_to_send = 0;
        portENTER_CRITICAL(&trace_mux);
        events_to_send = (trace_head < MAX_TRACE_EVENTS) ? trace_head : MAX_TRACE_EVENTS;
        
        if (events_to_send > 0) {
            if (trace_head >= MAX_TRACE_EVENTS) {
                uint16_t start_idx = trace_head % MAX_TRACE_EVENTS;
                if (start_idx + events_to_send > MAX_TRACE_EVENTS) {
                    uint16_t first_part = MAX_TRACE_EVENTS - start_idx;
                    memcpy(trace_buffer_copy, &trace_buffer[start_idx], first_part * sizeof(TaskEvent));
                    memcpy(&trace_buffer_copy[first_part], trace_buffer, (events_to_send - first_part) * sizeof(TaskEvent));
                } else {
                    memcpy(trace_buffer_copy, &trace_buffer[start_idx], events_to_send * sizeof(TaskEvent));
                }
            } else {
                memcpy(trace_buffer_copy, trace_buffer, events_to_send * sizeof(TaskEvent));
            }
        }
        trace_head = 0;
        portEXIT_CRITICAL(&trace_mux);

        // Build header
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

        // Update health metrics + DIAGNOSTICS
        sys_health.free_heap = esp_get_free_heap_size();
        sys_health.min_free_heap = esp_get_minimum_free_heap_size();

        sys_health.stack_min_tcp   = uxTaskGetStackHighWaterMark(NULL) * 4;
        if(GPlan_Task_Handle) sys_health.stack_min_gplan = uxTaskGetStackHighWaterMark(GPlan_Task_Handle) * 4;
        if(MPlan_Task_Handle) sys_health.stack_min_mplan = uxTaskGetStackHighWaterMark(MPlan_Task_Handle) * 4;
        if(Lidar_Task_Handle) sys_health.stack_min_lidar = uxTaskGetStackHighWaterMark(Lidar_Task_Handle) * 4;
        
        sys_health.last_esp2_packet_ms = millis() - last_ipc_activity_ts;
        
        // ADD DIAGNOSTIC COUNTERS TO HEALTH STRUCT
        sys_health.lidar_frames_processed = diag_counters.lidar_scans_received;
        sys_health.map_frames_processed = diag_counters.map_updates_completed;
        sys_health.planner_fail_count = diag_counters.sync_frames_dropped;
        sys_health.last_planner_status = diag_counters.mutex_failures_map; // Repurpose
        
        memcpy(&header[33], &sys_health, sizeof(SystemHealth));

        // Send packet
        if (tcpClient.connected()) {
            tcpClient.write(header, TCP_HEADER_SIZE);
            tcpClient.write((uint8_t*)&events_to_send, 2);
            
            if(gplen > 0) 
                tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            
            if (payload_map_sz > 0) {
                tcpClient.write(rle_buffer, payload_map_sz);
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
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    // --- MEMORY ALLOCATION ---
    Scan_Buffer_1 = new SyncedScan();
    Scan_Buffer_2 = new SyncedScan();
    
    gp_workspace = new GlobalPlannerWorkspace();
    if (!gp_workspace) {
        while(1);
    }

    // --- QUEUES ---
    Lidar_Buffer_Queue = xQueueCreate(3, sizeof(LiDARScan)); // Increased to 3
    Lidar_Pose_Queue_Ready = xQueueCreate(2, sizeof(SyncedScan*));
    Lidar_Pose_Queue_Free  = xQueueCreate(2, sizeof(SyncedScan*));
    
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_1, 0);
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_2, 0);

    Path_Send_Queue = xQueueCreate(1, sizeof(PathMessage));

    // --- MUTEXES & OBJECTS ---
    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    State_Mutex = xSemaphoreCreateMutex();
    Invalid_Mutex = xSemaphoreCreateMutex();
    xSemaphoreGOAL = xSemaphoreCreateBinary();
    xSemaphorePATH = xSemaphoreCreateBinary();

    xSemaphoreGive(xSemaphoreGOAL);

    TheMap = new BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});

    // --- WIFI SETUP ---
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(1000);

    // --- TCP SERVER SETUP ---
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    esp_link.begin(); 
    lidar.start(); 
    delay(1000);

    // --- LED BLINK ---
    pinMode(LED_BUILTIN, OUTPUT);
    for(int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }

    // --- TASK CREATION ---
    // Core 1 (Sensors) - Start first!
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar", 3072, NULL, 4, &Lidar_Task_Handle, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Sync", 4096, NULL, 4, &Sync_Task_Handle, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map", 4096, NULL, 3, &Map_Task_Handle, 1);

    // Core 0 (Comms) - Start after sensors
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP", 4096, NULL, 1, &TCP_Task_Handle, 0); 
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 2, &MPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(Global_Planner_Task, "Global", 8192, NULL, 2, &GPlan_Task_Handle, 0);
}

void loop() {}