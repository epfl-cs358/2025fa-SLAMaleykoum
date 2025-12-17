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

// Double Buffering Queues
QueueHandle_t Lidar_Pose_Queue_Ready; // Holds pointers to FULL buffers (Sync -> Map)
QueueHandle_t Lidar_Pose_Queue_Free;  // Holds pointers to EMPTY buffers (Map -> Sync)

TaskHandle_t GPlan_Task_Handle = NULL;
TaskHandle_t MPlan_Task_Handle = NULL;
TaskHandle_t Lidar_Task_Handle = NULL;
TaskHandle_t TCP_Task_Handle = NULL;
TaskHandle_t Sync_Task_Handle = NULL;
TaskHandle_t Map_Task_Handle = NULL;
TaskHandle_t Goal_Validator_Handle = NULL;

// MUTEXES
SemaphoreHandle_t Bayesian_Grid_Mutex;
SemaphoreHandle_t Pose_Mutex;
SemaphoreHandle_t Goal_Mutex;
SemaphoreHandle_t Path_Mutex;
SemaphoreHandle_t Invalid_Goals_Mutex;
SemaphoreHandle_t Planner_State_Mutex;

// --- GLOBAL OBJECTS ---
const int GRID_SIZE_X = 70;
const int GRID_SIZE_Y = 70;
const float RESOLUTION = 0.2f;

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
bool path_found = false;
bool first_map_ready = false;
int i = 0;

InvalidGoals invalid_goals = {{}, 0};

// --- SYSTEM HEALTH MONITOR ---
SystemHealth sys_health = {0, 0, 0, 0, 0, 0};
uint32_t last_ipc_activity_ts = 0;

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);

HardwareSerial& IPC_Serial = Serial1; 
Esp_link esp_link(IPC_Serial);

// --- NOTIFICATION BITS ---
// For clearer event tracking
#define NOTIFY_NEW_SCAN          (1 << 0)  // Bit 0: New lidar scan available
#define NOTIFY_NEW_GOAL          (1 << 1)  // Bit 1: New mission goal computed
#define NOTIFY_MAP_UPDATED       (1 << 2)  // Bit 2: Map has been updated
#define NOTIFY_POSE_UPDATED      (1 << 3)  // Bit 3: Robot pose updated
#define NOTIFY_GOAL_INVALID      (1 << 4)  // Bit 4: Current goal became invalid
#define NOTIFY_FIRST_MAP_READY   (1 << 5)  // Bit 5: First map ready for planning

// ============================================================================
// TASK 1: LIDAR READ (High Priority, Core 1)
// Runs continuously at high frequency, sends scan to sync task via queue
// ============================================================================
void Lidar_Read_Task(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(10); 
    static LiDARScan scan; 
    float lastAngleESP = 0.0f;
    bool scanComplete;

    while (1) {
        scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            if (scan.count > 10) { 
                // Send to sync task (notify handled by queue reception)
                xQueueSend(Lidar_Buffer_Queue, &scan, 0);
                sys_health.lidar_frames_processed++; // DEBUG
            }
            scan.count = 0;
            lastAngleESP = 0.0f; 
        }
        vTaskDelay(xDelay);
    }
}

// ============================================================================
// TASK 2: IPC RECEIVE (High Priority, Core 0)
// Continuously polls for pose updates
// ============================================================================
void IPC_Receive_Task(void* parameter) {
    while (1) {
        esp_link.poll();
        Pose2D incoming_pose;

        if (esp_link.get_pos(incoming_pose)) {
            if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                float x_diff = incoming_pose.x -  last_known_pose.x;
                float y_diff = incoming_pose.y -  last_known_pose.y;
                
                if (((x_diff) * (x_diff) + (y_diff) * (y_diff)) > 0.2f) {
                    // Discard obviously wrong pose
                    xSemaphoreGive(Pose_Mutex);
                    continue;
                }

                last_known_pose = incoming_pose;
                sys_health.last_esp2_packet_ms = millis(); 
                last_ipc_activity_ts = millis();
                xSemaphoreGive(Pose_Mutex);

                // Notify goal validator that pose changed
                if(Goal_Validator_Handle != NULL)
                    xTaskNotify(Goal_Validator_Handle, NOTIFY_POSE_UPDATED, eSetBits);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// TASK 3: LIDAR SYNC (High Priority, Core 1)
// EVENT-DRIVEN: Triggered by new scan from Lidar_Read_Task
// ============================================================================
void Lidar_Sync_Map_Task(void* parameter) {
    LiDARScan* scan_buffer = new LiDARScan(); // Heap alloc for temp storage
    SyncedScan* current_work_buffer = nullptr;
    int local_counter = 0;

    while (1) {
        // BLOCKING wait for new scan (event-driven)
        if (xQueueReceive(Lidar_Buffer_Queue, scan_buffer, portMAX_DELAY) == pdPASS) {
            
            // Downsample: Only process every 7th scan to save CPU
            local_counter++;
            if (local_counter % 6 != 0) continue;

            // Get free buffer (non-blocking, drop frame if none available)
            if (xQueueReceive(Lidar_Pose_Queue_Free, &current_work_buffer, 0) == pdPASS) {
                
                // 3. Fill the buffer (Direct write to Heap, NO STACK usage)
                current_work_buffer->scan = *scan_buffer;
                
                if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    current_work_buffer->pose = last_known_pose;
                    xSemaphoreGive(Pose_Mutex);
                }

                // Send to map task
                xQueueSend(Lidar_Pose_Queue_Ready, &current_work_buffer, 0);
                if(i<10) ++i;
                
                if (Map_Task_Handle != NULL && (i == 10)) {
                    xTaskNotify(Map_Task_Handle, NOTIFY_NEW_SCAN, eSetBits);
                }
            }
        }
    }
}

// ============================================================================
// TASK 4: BAYESIAN MAPPING (Medium Priority, Core 1)
// EVENT-DRIVEN: Triggered by Lidar_Sync_Map_Task notification
// ============================================================================
void Bayesian_Grid_Task(void* parameter) {
    SyncedScan* incoming_data_ptr = nullptr;
    uint32_t notification_bits;
    const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(2000); // Check every 2s

    while (1) {
        // WAIT for NOTIFY_NEW_SCAN bit
        BaseType_t ret = xTaskNotifyWait(
            0x00,                    // Don't clear bits on entry
            NOTIFY_NEW_SCAN,     // Clear POSE_UPDATED bit on exit
            &notification_bits,      // Store notification value
            CHECK_PERIOD             // Timeout
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE) {
            notification_bits |= NOTIFY_NEW_SCAN;
        }

        if (!(notification_bits & NOTIFY_NEW_SCAN)) continue;

        // Measure Queue Load roughly
        UBaseType_t msgs_waiting = uxQueueMessagesWaiting(Lidar_Pose_Queue_Ready);
        sys_health.queue_load_percent = (msgs_waiting * 100) / 2; // Size is 2
        // Process all available scans in queue
        while (xQueueReceive(Lidar_Pose_Queue_Ready, &incoming_data_ptr, 0) == pdPASS) {
            
            // Update Map (Read directly from Heap)
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                uint32_t t0 = micros(); // START TIMER
                
                TheMap->update_map(*incoming_data_ptr);
                
                sys_health.map_update_time_us = micros() - t0; // STOP TIMER
                sys_health.map_frames_processed++;
                
                xSemaphoreGive(Bayesian_Grid_Mutex);
                
                // Mark first map as ready for mission planner
                if (!first_map_ready) {
                    first_map_ready = true;
                    // Notify mission planner it can start
                    if (MPlan_Task_Handle != NULL) {
                        xTaskNotify(MPlan_Task_Handle, NOTIFY_FIRST_MAP_READY, eSetBits);
                    }
                }
            }
            
            // Update Pose Snapshot
            if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                last_known_pose = incoming_data_ptr->pose;
                xSemaphoreGive(Pose_Mutex);
            }

            // Recycle buffer
            xQueueSend(Lidar_Pose_Queue_Free, &incoming_data_ptr, 0);
        }
    }
}

// ============================================================================
// TASK 5: GOAL VALIDATOR (Low Priority, Core 0)
// Periodically checks if current goal is still valid
// Woken up by pose updates or timer
// ============================================================================
bool is_current_goal_valid(const Pose2D& robot_pose, const BayesianOccupancyGrid& grid, 
    const MissionGoal& current_target_, bool path_found_flag) 
{
    if (!path_found_flag) 
        return false;

    // Proximity Check
    float dx = robot_pose.x - current_target_.target_pose.x;
    float dy = robot_pose.y - current_target_.target_pose.y;
    if ((dx*dx + dy*dy) < GOAL_REACHED * GOAL_REACHED) 
        return false;
    
    // Goal is in free/unknown space ?
    int gx, gy;
    world_to_grid(current_target_.target_pose.x, current_target_.target_pose.y, 
                  gx, gy, grid.grid_resolution, grid.grid_size_x, grid.grid_size_y);

    if (grid.get_cell_probability(gx, gy) > OCC_BOUND_PROB) 
        return false;

    // Still a frontier ?
    if (!is_frontier_cell(grid, gx, gy)) 
        return false;
    
    return true;
}

void Goal_Validator_Task(void* parameter) {
    const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(500); // Check every 0.5s
    uint32_t notification_bits;

    while (1) {
        // Wait for notification OR timeout
        BaseType_t ret = xTaskNotifyWait(
            0x00,                    // Don't clear bits on entry
            NOTIFY_POSE_UPDATED,     // Clear POSE_UPDATED bit on exit
            &notification_bits,      // Store notification value
            CHECK_PERIOD             // Timeout
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE) {
            notification_bits |= NOTIFY_POSE_UPDATED;
        }

        if (!(notification_bits & NOTIFY_POSE_UPDATED)) continue;
        
        Pose2D current_p;
        bool path_found_local;

        // Read current state
        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else
            continue;
        
        if (xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            path_found_local = path_found;
            xSemaphoreGive(Path_Mutex);
        } else
            continue;
        
        // Check if goal is still valid
        if (!is_current_goal_valid(current_p, *TheMap, current_mission_goal, path_found_local)) {
            // Goal invalid - notify mission planner to compute new one
            if (MPlan_Task_Handle != NULL) {
                xTaskNotify(MPlan_Task_Handle, NOTIFY_GOAL_INVALID, eSetBits);
            }
        }
    }
}

// ============================================================================
// TASK 6: MISSION PLANNER (Low Priority, Core 0)
// EVENT-DRIVEN: Triggered when goal becomes invalid or first map ready
// ============================================================================
void Mission_Planner_Task(void* parameter) {
    MissionGoal new_goal;
    uint32_t notification_bits;

    while (1) {
        // WAIT for FIRST_MAP_READY or GOAL_INVALID bits
        xTaskNotifyWait(
            0x00,                                              // Don't clear on entry
            NOTIFY_FIRST_MAP_READY | NOTIFY_GOAL_INVALID,     // Clear these bits on exit
            &notification_bits,                                // Store value
            portMAX_DELAY                                      // Block forever
        );

        // On first map ready, wait a bit for map to stabilize
        if (notification_bits & NOTIFY_FIRST_MAP_READY)
            vTaskDelay(pdMS_TO_TICKS(2000));
        else if (!(notification_bits & NOTIFY_GOAL_INVALID)) continue;

        // --- CALCULATION START ---
        uint32_t t0 = micros(); // Start Timer

        Pose2D current_p;
        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else
            continue;

        if(xSemaphoreTake(Invalid_Goals_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            new_goal = mission_planner->update_goal(current_p, *TheMap, invalid_goals);
            xSemaphoreGive(Invalid_Goals_Mutex);
        }

        // --- CALCULATION END ---
        sys_health.mission_plan_time_us = micros() - t0; // Write result to global struct

        if (xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            current_mission_goal = new_goal;
            xSemaphoreGive(Goal_Mutex);
        }

        // NOTIFY Global Planner: New goal ready
        if (GPlan_Task_Handle != NULL)
            xTaskNotify(GPlan_Task_Handle, NOTIFY_NEW_GOAL, eSetBits);
    }
}

// ============================================================================
// TASK 7: GLOBAL PLANNER (Medium Priority, Core 0)
// EVENT-DRIVEN: Triggered by Mission_Planner_Task when new goal arrives
// ============================================================================
void Global_Planner_Task(void* parameter) {
    GlobalPlanner planner; 
    Pose2D local_pose;
    MissionGoal local_goal;
    PathMessage pathMsg = {{{0}}, 0, 0, 0};
    uint32_t notification_bits;
    const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(5000); // Check every 0.5s

    while (1) {
        // WAIT for NOTIFY_NEW_GOAL bit
        BaseType_t ret = xTaskNotifyWait(
            0x00,               // Don't clear on entry
            NOTIFY_NEW_GOAL,    // Clear NEW_GOAL bit on exit
            &notification_bits, // Store value
            CHECK_PERIOD
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE) {
            notification_bits |= NOTIFY_NEW_GOAL;
        }

        if (!(notification_bits & NOTIFY_NEW_GOAL)) continue;

        // --- CALCULATION START ---
        uint32_t t0 = micros();
        sys_health.last_planner_status = PLANNER_RUNNING_MY_TYPE;

        // if (TheMap->get_cell_probability(sx, sy) > 0.5f) {
        //     sys_health.last_planner_status = ERR_START_BLOCKED_MY_TYPE;
        //     sys_health.planner_fail_count++;
        //     // Skip A*
        // } 
        // else if (TheMap->get_cell_probability(gx, gy) > 0.5f) {
        //     sys_health.last_planner_status = ERR_GOAL_BLOCKED_MY_TYPE;
        //     sys_health.planner_fail_count++;
        //     // Skip A*
        // }

        // Read current state
        if(xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            local_goal = current_mission_goal;
            xSemaphoreGive(Goal_Mutex); 
        } else 
            continue;

        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            local_pose = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else 
            continue;
        
        // Generate path (expensive operation ~300ms)
        pathMsg = planner.generate_path(local_pose, local_goal, *TheMap, gp_workspace);
        
        // --- CALCULATION END ---
        sys_health.global_plan_time_us = micros() - t0; 

        bool new_path_found = (pathMsg.current_length != 0);

        if (new_path_found) {
            // Update global path
            if(xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                current_global_path = pathMsg;
                path_found = true;
                xSemaphoreGive(Path_Mutex);
            }

            sys_health.last_planner_status = PLANNER_SUCCESS_MY_TYPE;

            // Send to ESP2
            esp_link.sendPath(current_global_path);

        } else {
            sys_health.last_planner_status = ERR_NO_PATH_FOUND_MY_TYPE;
            sys_health.planner_fail_count++;
            // Add to invalid goals
            if(xSemaphoreTake(Invalid_Goals_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (invalid_goals.size >= MAX_INVALID_GOALS) 
                    invalid_goals.size = 0;
                invalid_goals.lasts[invalid_goals.size++] = local_goal.target_pose;
                xSemaphoreGive(Invalid_Goals_Mutex);
            }

            if(xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                path_found = false; 
                xSemaphoreGive(Path_Mutex);
            }

            // Immediately request new goal
            if (MPlan_Task_Handle != NULL) {
                xTaskNotify(MPlan_Task_Handle, NOTIFY_GOAL_INVALID, eSetBits);
            }
        }
    }
}

// ============================================================================
// TASK 8: TCP TRANSMIT (Low Priority, Core 0)
// Runs at fixed frequency, sends telemetry and map
// ============================================================================
static uint8_t rle_buffer[10000];

void TCP_Transmit_Task(void* parameter) {
    WiFiClient tcpClient;
    uint8_t header[TCP_HEADER_SIZE]; 
    
    const TickType_t TelemetryFreq = pdMS_TO_TICKS(100); 
    const TickType_t MapFreq = pdMS_TO_TICKS(3000); // 3 Seconds
    
    TickType_t lastMapTime = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    struct StateSnapshot {
        Pose2D pose;
        MissionGoal goal;
        PathMessage global_path;
        uint8_t state;
    } Tx_Snapshot;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, TelemetryFreq);

        // Check TCP connection
        if (!tcpClient || !tcpClient.connected()) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
            } else
                continue;
        }

        // Quick snapshot ot state
        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Tx_Snapshot.pose = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        }
        
        if (xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Tx_Snapshot.global_path = current_global_path;
            xSemaphoreGive(Path_Mutex);
        }
        
        if (xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Tx_Snapshot.goal = current_mission_goal;
            Tx_Snapshot.state = (uint8_t)current_mission_goal.type;
            xSemaphoreGive(Goal_Mutex);
        }

        // RLE compress map (lock-free read)
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

        // UPDATE HEALTH METRICS
        sys_health.free_heap = esp_get_free_heap_size();
        sys_health.min_free_heap = esp_get_minimum_free_heap_size();
        sys_health.stack_min_tcp = uxTaskGetStackHighWaterMark(NULL) * 4;
        if(GPlan_Task_Handle) sys_health.stack_min_gplan = uxTaskGetStackHighWaterMark(GPlan_Task_Handle) * 4;
        if(MPlan_Task_Handle) sys_health.stack_min_mplan = uxTaskGetStackHighWaterMark(MPlan_Task_Handle) * 4;
        sys_health.last_esp2_packet_ms = millis() - last_ipc_activity_ts;
        
        memcpy(&header[33], &sys_health, sizeof(SystemHealth));

        // Transmit
        if (tcpClient.connected()) {
            tcpClient.write(header, TCP_HEADER_SIZE);
            if(gplen > 0) 
                tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            
            if (payload_map_sz > 0) {
                tcpClient.write(rle_buffer, payload_map_sz);
                lastMapTime = xTaskGetTickCount();
            }
        }
    }
}

// SETUP
void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial Monitor to catch up
    Serial.println("--- BOOT START ---");

    // --- MEMORY ALLOCATION ---
    Scan_Buffer_1 = new SyncedScan();
    Scan_Buffer_2 = new SyncedScan();
    gp_workspace = new GlobalPlannerWorkspace();
    if (!gp_workspace) while(1);

    // --- QUEUES ---
    Lidar_Buffer_Queue = xQueueCreate(3, sizeof(LiDARScan));
    Lidar_Pose_Queue_Ready = xQueueCreate(2, sizeof(SyncedScan*));
    Lidar_Pose_Queue_Free  = xQueueCreate(2, sizeof(SyncedScan*));
    
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_1, 0);
    xQueueSend(Lidar_Pose_Queue_Free, &Scan_Buffer_2, 0);

    // --- MUTEXES & OBJECTS ---
    Bayesian_Grid_Mutex = xSemaphoreCreateMutex();
    Pose_Mutex = xSemaphoreCreateMutex();
    Goal_Mutex = xSemaphoreCreateMutex();
    Path_Mutex = xSemaphoreCreateMutex();
    Invalid_Goals_Mutex = xSemaphoreCreateMutex();

    TheMap = new BayesianOccupancyGrid(RESOLUTION, GRID_SIZE_X, GRID_SIZE_Y);
    mission_planner = new MissionPlanner({0.0f, 0.0f, 0.0f, 0});

    // --- WIFI ---
    Serial.println("Starting WiFi...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    delay(1000);

    // --- TCP SERVER SETUP ---
    tcpServer.begin();
    tcpServer.setNoDelay(true);

    // --- Hardware Init ---
    esp_link.begin(); 
    lidar.start(); 
    delay(5000);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED
    delay(500);
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); // Turn off built-in LED
    delay(200);
    digitalWrite(LED_BUILTIN, LOW); // Turn off built-in LED

    // ========================================================================
    // TASK CREATION - Optimized Priority & Core Assignment
    // ========================================================================
    // Priority Guidelines:
    // 5-6: Critical real-time (Lidar, IPC)
    // 3-4: Important computation (Mapping, Planning)
    // 1-2: Background (TCP, Mission Planning)

    // CORE 1 (Sensor Processing)
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar", 3072, NULL, 6, &Lidar_Task_Handle, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Sync", 4096, NULL, 5, &Sync_Task_Handle, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map", 4096, NULL, 4, &Map_Task_Handle, 1);

    // CORE 0 (Communication & Planning)
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC", 2048, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(Global_Planner_Task, "GPlanner", 8192, NULL, 3, &GPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(Goal_Validator_Task, "Validator", 3072, NULL, 4, &Goal_Validator_Handle, 0);
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 3, &MPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP", 4096, NULL, 1, &TCP_Task_Handle, 0);
}

void loop() {}
