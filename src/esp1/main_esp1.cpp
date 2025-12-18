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

// Double Buffering Queues
QueueHandle_t Lidar_Buffer_Queue;
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

InvalidGoals invalid_goals = {{}, 0};
OccupancyGridSnapshot grid_snapshot_mission;
OccupancyGridSnapshot grid_snapshot_global;
OccupancyGridSnapshot grid_snapshot_goal_check;

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


// --- PROFILER / TRACING BUFFER ---
// Simple circular buffer to store task events
TaskEvent trace_buffer[MAX_TRACE_EVENTS];
volatile uint16_t trace_head = 0;
volatile uint16_t trace_wrap_count = 0;  // Track number of buffer wraps
portMUX_TYPE trace_mux = portMUX_INITIALIZER_UNLOCKED;

// FIXED: Use circular buffer with proper wraparound
void log_event(uint8_t task_id, uint8_t type) {
    portENTER_CRITICAL(&trace_mux);
    // Use modulo to create circular buffer
    uint16_t idx = trace_head % MAX_TRACE_EVENTS;
    trace_buffer[idx].timestamp_us = micros();
    trace_buffer[idx].task_id = task_id;
    trace_buffer[idx].type = type;
    trace_buffer[idx].core_id = xPortGetCoreID();
    trace_head++;
    // Track wraps for debugging
    if (trace_head >= MAX_TRACE_EVENTS && (trace_head % MAX_TRACE_EVENTS) == 0) {
        trace_wrap_count++;
    }
    if (trace_head >= 65000) trace_head = 0;
    portEXIT_CRITICAL(&trace_mux);
}

// ============================================================================
// TASK 1: LIDAR READ (High Priority, Core 1)
// Runs continuously at high frequency, sends scan to sync task via queue
// ============================================================================
void Lidar_Read_Task(void* parameter) {
    static LiDARScan scan; 
    float lastAngleESP = 0.0f;
    bool scanComplete;

    while (1) {
        scanComplete = false;
        lidar.build_scan(&scan, scanComplete, lastAngleESP);

        if (scanComplete) {
            log_event(ID_LIDAR, EVENT_START);
            if (scan.count > 10) {
                // Send to sync task (notify handled by queue reception)
                xQueueSend(Lidar_Buffer_Queue, &scan, 0);
                sys_health.lidar_frames_processed++; // DEBUG
            }
            scan.count = 0;
            lastAngleESP = 0.0f; 
            log_event(ID_LIDAR, EVENT_END);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// TASK 2: IPC RECEIVE (High Priority, Core 0)
// Continuously polls for pose updates
// ============================================================================
void IPC_Receive_Task(void* parameter) {
    Pose2D incoming_pose; 
    bool skip;

    while (1) {
        skip = false;
        log_event(ID_IPC, EVENT_START);
        esp_link.poll();

        if (!esp_link.get_pos(incoming_pose)) continue;

        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float x_diff = incoming_pose.x -  last_known_pose.x;
            float y_diff = incoming_pose.y -  last_known_pose.y;
            
            if (((x_diff) * (x_diff) + (y_diff) * (y_diff)) > 0.7f) {
                skip = true;
                // Discard obviously wrong pose
            } else {
                last_known_pose = incoming_pose;
                sys_health.last_esp2_packet_ms = millis(); 
                last_ipc_activity_ts = millis();
            }

            xSemaphoreGive(Pose_Mutex);
        } else continue;

        if (skip) continue;

        // Notify goal validator that pose changed
        if(Goal_Validator_Handle != NULL)
            xTaskNotify(Goal_Validator_Handle, NOTIFY_POSE_UPDATED, eSetBits);

        log_event(ID_IPC, EVENT_END);
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
            
            log_event(ID_SYNC, EVENT_START);

            // Downsample
            local_counter++;
            if (local_counter % 5 == 0) {
                // Get free buffer (non-blocking, drop frame if none available)
                if (xQueueReceive(Lidar_Pose_Queue_Free, &current_work_buffer, 0) == pdPASS) {
                    
                    // 3. Fill the buffer (Direct write to Heap, NO STACK usage)
                    current_work_buffer->scan = *scan_buffer;
                    
                    if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        current_work_buffer->pose = last_known_pose;
                        xSemaphoreGive(Pose_Mutex);
                    }

                    // Send to map task
                    xQueueSend(Lidar_Pose_Queue_Ready, &current_work_buffer, 0);
                    
                    if (Map_Task_Handle != NULL)
                        xTaskNotify(Map_Task_Handle, NOTIFY_NEW_SCAN, eSetBits);
                }
            }
            log_event(ID_SYNC, EVENT_END);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================================================
// TASK 4: BAYESIAN MAPPING (Medium Priority, Core 1)
// EVENT-DRIVEN: Triggered by Lidar_Sync_Map_Task notification
// ============================================================================
void Bayesian_Grid_Task(void* parameter) {
    SyncedScan* incoming_data_ptr = nullptr;
    uint32_t notification_bits;
    bool first_map_ready = false;
    // const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(10000); // Check every 2s

    while (1) {
        // WAIT for NOTIFY_NEW_SCAN bit
        BaseType_t ret = xTaskNotifyWait(
            0x00,                   // Don't clear bits on entry
            NOTIFY_NEW_SCAN,        // Clear POSE_UPDATED bit on exit
            &notification_bits,     // Store notification value
            portMAX_DELAY            // Timeout
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE) notification_bits |= NOTIFY_NEW_SCAN;

        if (!(notification_bits & NOTIFY_NEW_SCAN)) continue;

        // Measure Queue Load roughly
        UBaseType_t msgs_waiting = uxQueueMessagesWaiting(Lidar_Pose_Queue_Ready);
        sys_health.queue_load_percent = (msgs_waiting * 100) / 2; // Size is 2
        // Process all available scans in queue
        while (xQueueReceive(Lidar_Pose_Queue_Ready, &incoming_data_ptr, 0) == pdPASS) {
            
            log_event(ID_MAP, EVENT_START); // Start Profiling

            // Update Map (Read directly from Heap)
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                uint32_t t0 = micros(); // START TIMER

                TheMap->update_map(*incoming_data_ptr);

                sys_health.map_update_time_us = micros() - t0; // STOP TIMER
                sys_health.map_frames_processed++;
                xSemaphoreGive(Bayesian_Grid_Mutex);
            } else continue;

            // Mark first map as ready for mission planner
            if (!first_map_ready) {
                first_map_ready = true;
                // Notify mission planner it can start
                if (MPlan_Task_Handle != NULL)
                    xTaskNotify(MPlan_Task_Handle, NOTIFY_FIRST_MAP_READY, eSetBits);
            }

            // Recycle buffer
            xQueueSend(Lidar_Pose_Queue_Free, &incoming_data_ptr, 0);

            log_event(ID_MAP, EVENT_END); // End Profiling
        }
    }
}

// ============================================================================
// TASK 5: GOAL VALIDATOR (Low Priority, Core 0)
// Periodically checks if current goal is still valid
// Woken up by pose updates or timer
// ============================================================================
bool is_current_goal_valid(const Pose2D& robot_pose, const OccupancyGridSnapshot& grid, 
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

    if (get_cell_probability_snapshot(grid, gx, gy) > OCC_BOUND_PROB) 
        return false;

    // Still a frontier ?
    if (!is_frontier_cell_snapshot(grid, gx, gy)) 
        return false;
    
    return true;
}

void Goal_Validator_Task(void* parameter) {
    const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(500); // Check every 0.5s
    uint32_t notification_bits;
    Pose2D current_p;
    bool path_found_local;
    MissionGoal current_goal_local = {{0.0f, 0.f, 0.f, 0}, EXPLORATION_MODE};

    while (1) {
        // Wait for notification OR timeout
        BaseType_t ret = xTaskNotifyWait(
            0x00,                    // Don't clear bits on entry
            NOTIFY_POSE_UPDATED,     // Clear POSE_UPDATED bit on exit
            &notification_bits,      // Store notification value
            CHECK_PERIOD             // Timeout
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE)
            notification_bits |= NOTIFY_POSE_UPDATED;

        if (!(notification_bits & NOTIFY_POSE_UPDATED)) continue;
        
        log_event(ID_VALIDATOR, EVENT_START);

        // Read current state
        if (xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            log_event(ID_VALIDATOR, EVENT_END);
            continue;
        }
        
        if (xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            path_found_local = path_found;
            xSemaphoreGive(Path_Mutex);
        } else {
            log_event(ID_VALIDATOR, EVENT_END);
            continue;
        }

        if (xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            current_goal_local = current_mission_goal;
            xSemaphoreGive(Goal_Mutex);
        } else {
            log_event(ID_VALIDATOR, EVENT_END);
            continue;
        }
        
        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(
                grid_snapshot_goal_check.log_odds,
                TheMap->get_raw_data_pointer(),
                grid_snapshot_goal_check.grid_size_x * grid_snapshot_goal_check.grid_size_y
            );
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } else {
            log_event(ID_VALIDATOR, EVENT_END);
            continue;
        }

        // Check if goal is still valid
        if (!is_current_goal_valid(current_p, grid_snapshot_goal_check, current_goal_local, path_found_local)) {
            // Goal invalid - notify mission planner to compute new one
            if (MPlan_Task_Handle != NULL)
                xTaskNotify(MPlan_Task_Handle, NOTIFY_GOAL_INVALID, eSetBits);
        }

        log_event(ID_VALIDATOR, EVENT_END);
    }
}

// ============================================================================
// TASK 6: MISSION PLANNER (Low Priority, Core 0)
// EVENT-DRIVEN: Triggered when goal becomes invalid or first map ready
// ============================================================================
void Mission_Planner_Task(void* parameter) {
    MissionGoal new_goal;
    uint32_t notification_bits;
    Pose2D current_p;
    InvalidGoals invalid_goals_local = {{}, 0};

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

        log_event(ID_MPLAN, EVENT_START); // Start Trace

        // --- CALCULATION START ---
        uint32_t t0 = micros(); // Start Timer

        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_p = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            sys_health.mission_plan_time_us = micros() - t0;
            log_event(ID_MPLAN, EVENT_END);
            continue;
        }

        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(
                grid_snapshot_mission.log_odds,
                TheMap->get_raw_data_pointer(),
                grid_snapshot_mission.grid_size_x * grid_snapshot_mission.grid_size_y
            );
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } else {
            sys_health.mission_plan_time_us = micros() - t0;
            log_event(ID_MPLAN, EVENT_END);
            continue;
        }

        if(xSemaphoreTake(Invalid_Goals_Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            invalid_goals_local = invalid_goals;
            xSemaphoreGive(Invalid_Goals_Mutex);
        } else {
            sys_health.mission_plan_time_us = micros() - t0;
            log_event(ID_MPLAN, EVENT_END);
            continue;
        }

        new_goal = mission_planner->update_goal(current_p, &grid_snapshot_mission, invalid_goals_local);

        // --- CALCULATION END ---
        sys_health.mission_plan_time_us = micros() - t0; // Write result to global struct

        if (xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            current_mission_goal = new_goal;
            xSemaphoreGive(Goal_Mutex);
        } else {
            sys_health.mission_plan_time_us = micros() - t0;
            log_event(ID_MPLAN, EVENT_END);
            continue;
        }

        // NOTIFY Global Planner: New goal ready
        if (GPlan_Task_Handle != NULL)
            xTaskNotify(GPlan_Task_Handle, NOTIFY_NEW_GOAL, eSetBits);

        log_event(ID_MPLAN, EVENT_END); // End Trace
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
    PathMessage pathMsg = {{}, 0, 0, 0};
    uint32_t notification_bits;
    // const TickType_t CHECK_PERIOD = pdMS_TO_TICKS(10000); // Check every 0.5s

    while (1) {
        // WAIT for NOTIFY_NEW_GOAL bit
        BaseType_t ret = xTaskNotifyWait(
            0x00,               // Don't clear on entry
            NOTIFY_NEW_GOAL,    // Clear NEW_GOAL bit on exit
            &notification_bits, // Store value
            portMAX_DELAY
        );

        // Si timeout, on FORCE le bit
        if (ret == pdFALSE)
            notification_bits |= NOTIFY_NEW_GOAL;

        if (!(notification_bits & NOTIFY_NEW_GOAL)) continue;

        log_event(ID_GPLAN, EVENT_START); // Start Trace

        if(xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            path_found = false;
            xSemaphoreGive(Path_Mutex);
        }

        // --- CALCULATION START ---
        uint32_t t0 = micros();
        sys_health.last_planner_status = PLANNER_RUNNING_MY_TYPE;

        // Read current state
        if(xSemaphoreTake(Goal_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            local_goal = current_mission_goal;
            xSemaphoreGive(Goal_Mutex); 
        } else {
            sys_health.global_plan_time_us = micros() - t0; 
            log_event(ID_GPLAN, EVENT_END);
            continue;
        }

        if(xSemaphoreTake(Pose_Mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            local_pose = last_known_pose;
            xSemaphoreGive(Pose_Mutex);
        } else {
            sys_health.global_plan_time_us = micros() - t0; 
            log_event(ID_GPLAN, EVENT_END);
            continue;
        }

        if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(
                grid_snapshot_global.log_odds,
                TheMap->get_raw_data_pointer(),
                grid_snapshot_global.grid_size_x * grid_snapshot_global.grid_size_y
            );
            xSemaphoreGive(Bayesian_Grid_Mutex);
        } else {
            sys_health.global_plan_time_us = micros() - t0; 
            log_event(ID_GPLAN, EVENT_END);
            continue;
        }
        
        // Generate path (expensive operation ~300ms)
        pathMsg = planner.generate_path(local_pose, local_goal, grid_snapshot_global, gp_workspace);
        
        // --- CALCULATION END ---
        sys_health.global_plan_time_us = micros() - t0; 

        bool new_path_found = (pathMsg.current_length != 0);

        if (new_path_found) {
            // Update global path
            if(xSemaphoreTake(Path_Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                current_global_path = pathMsg;
                path_found = true;
                xSemaphoreGive(Path_Mutex);
            }

            sys_health.last_planner_status = PLANNER_SUCCESS_MY_TYPE;

            // Send to ESP2
            esp_link.sendPath(pathMsg);

        } else {
            sys_health.last_planner_status = ERR_NO_PATH_FOUND_MY_TYPE;
            sys_health.planner_fail_count++;
            // Add to invalid goals
            if(xSemaphoreTake(Invalid_Goals_Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
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
        log_event(ID_GPLAN, EVENT_END); // End Trace
    }
}

// ============================================================================
// TASK 8: TCP TRANSMIT (Low Priority, Core 0)
// Runs at fixed frequency, sends telemetry and map
// ============================================================================
static uint8_t rle_buffer[10000];
static TaskEvent trace_buffer_copy[MAX_TRACE_EVENTS]; // Temp buffer for sending

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
        log_event(ID_TCP, EVENT_START);

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

        bool skip = false;
        if (send_map_now) { // ------------------------- caca
            const int8_t* raw_map;
            uint32_t total_cells;
            if (xSemaphoreTake(Bayesian_Grid_Mutex, pdMS_TO_TICKS(30)) == pdTRUE) {
                raw_map = TheMap->get_map_data();
                total_cells = TheMap->get_total_size();
                xSemaphoreGive(Bayesian_Grid_Mutex);
            } else {
                skip = true;
                continue;
            }

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

        if (skip) continue;

        // Trace Data Extraction (Atomic Copy) - FIXED
        uint16_t events_to_send = 0;
        uint16_t current_head = 0;
        portENTER_CRITICAL(&trace_mux);
        current_head = trace_head;
        // Only send events written since last transmission
        // Cap at MAX_TRACE_EVENTS to prevent overflow
        events_to_send = (current_head < MAX_TRACE_EVENTS) ? current_head : MAX_TRACE_EVENTS;
        
        // Copy only the valid events
        if (events_to_send > 0) {
            // If we wrapped, copy the most recent events
            if (current_head >= MAX_TRACE_EVENTS) {
                // Calculate start position in circular buffer
                uint16_t start_idx = current_head % MAX_TRACE_EVENTS;
                // Copy in two parts if wrapped
                if (start_idx + events_to_send > MAX_TRACE_EVENTS) {
                    uint16_t first_part = MAX_TRACE_EVENTS - start_idx;
                    memcpy(trace_buffer_copy, &trace_buffer[start_idx], first_part * sizeof(TaskEvent));
                    memcpy(&trace_buffer_copy[first_part], trace_buffer, (events_to_send - first_part) * sizeof(TaskEvent));
                } else {
                    memcpy(trace_buffer_copy, &trace_buffer[start_idx], events_to_send * sizeof(TaskEvent));
                }
            } else {
                // No wrap, simple copy
                memcpy(trace_buffer_copy, trace_buffer, events_to_send * sizeof(TaskEvent));
            }
        }
        trace_head = 0; // Reset counter
        portEXIT_CRITICAL(&trace_mux);

        // Header Construction
        memset(header, 0, TCP_HEADER_SIZE);
        header[0] = 0xBE; header[1] = 0xEF;
        memcpy(&header[2], &payload_map_sz, 4);
        
        uint16_t gplen = Tx_Snapshot.global_path.current_length;
        if(gplen > MAX_PATH_LENGTH) gplen = 0;
        uint16_t gplen_pack = gplen;
        memcpy(&header[6], &gplen_pack, 2);

        memcpy(&header[8], &Tx_Snapshot.pose, 12);
        memcpy(&header[20], &Tx_Snapshot.goal.target_pose, 12);
        header[32] = Tx_Snapshot.state;

        // Health
        sys_health.free_heap = esp_get_free_heap_size();
        sys_health.min_free_heap = esp_get_minimum_free_heap_size();
        memcpy(&header[33], &sys_health, sizeof(SystemHealth));

        // 5. SEND (Header + TraceCount + Path + Map + TraceEvents)
        if (tcpClient.connected()) {
            tcpClient.write(header, TCP_HEADER_SIZE);
            
            // Send trace count (Append before path)
            tcpClient.write((uint8_t*)&events_to_send, 2);

            if(gplen > 0) 
                tcpClient.write((uint8_t*)Tx_Snapshot.global_path.path, gplen * sizeof(Waypoint));
            
            if (payload_map_sz > 0) {
                tcpClient.write(rle_buffer, payload_map_sz);
                lastMapTime = xTaskGetTickCount();
            }

            // Send Trace Events (Variable Size)
            if (events_to_send > 0) {
                tcpClient.write((uint8_t*)trace_buffer_copy, events_to_send * sizeof(TaskEvent));
            }
        }
        log_event(ID_TCP, EVENT_END);
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

    size_t total = GRID_SIZE_X * GRID_SIZE_Y;
    
    grid_snapshot_mission.grid_size_x = GRID_SIZE_X;
    grid_snapshot_mission.grid_size_y = GRID_SIZE_Y;
    grid_snapshot_mission.grid_resolution = RESOLUTION;
    grid_snapshot_mission.log_odds = (int8_t*) heap_caps_malloc(
        grid_snapshot_mission.grid_size_x * grid_snapshot_mission.grid_size_y * sizeof(int8_t),
        MALLOC_CAP_SPIRAM
    );
    memset(grid_snapshot_mission.log_odds, 0, grid_snapshot_mission.grid_size_x * grid_snapshot_mission.grid_size_y);

    grid_snapshot_global.grid_size_x = GRID_SIZE_X;
    grid_snapshot_global.grid_size_y = GRID_SIZE_Y;
    grid_snapshot_global.grid_resolution = RESOLUTION;
    grid_snapshot_global.log_odds = (int8_t*) heap_caps_malloc(
        grid_snapshot_global.grid_size_x * grid_snapshot_global.grid_size_y * sizeof(int8_t),
        MALLOC_CAP_SPIRAM
    );
    memset(grid_snapshot_global.log_odds, 0, grid_snapshot_global.grid_size_x * grid_snapshot_global.grid_size_y);

    grid_snapshot_goal_check.grid_size_x = GRID_SIZE_X;
    grid_snapshot_goal_check.grid_size_y = GRID_SIZE_Y;
    grid_snapshot_goal_check.grid_resolution = RESOLUTION;
    grid_snapshot_goal_check.log_odds = (int8_t*) heap_caps_malloc(
        grid_snapshot_goal_check.grid_size_x * grid_snapshot_goal_check.grid_size_y * sizeof(int8_t),
        MALLOC_CAP_SPIRAM
    );
    memset(grid_snapshot_goal_check.log_odds, 0, grid_snapshot_goal_check.grid_size_x * grid_snapshot_goal_check.grid_size_y);

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
    delay(1000);
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
    xTaskCreatePinnedToCore(Lidar_Read_Task, "Lidar", 3072, NULL, 5, &Lidar_Task_Handle, 1);
    xTaskCreatePinnedToCore(Lidar_Sync_Map_Task, "Sync", 4096, NULL, 4, &Sync_Task_Handle, 1);
    xTaskCreatePinnedToCore(Bayesian_Grid_Task, "Map", 4096, NULL, 3, &Map_Task_Handle, 1);
    xTaskCreatePinnedToCore(Goal_Validator_Task, "Validator", 3072, NULL, 2, &Goal_Validator_Handle, 1);

    // CORE 0 (Communication & Planning)
    xTaskCreatePinnedToCore(IPC_Receive_Task, "IPC", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(Global_Planner_Task, "GPlanner", 8192, NULL, 4, &GPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(Mission_Planner_Task, "Mission", 4096, NULL, 3, &MPlan_Task_Handle, 0);
    xTaskCreatePinnedToCore(TCP_Transmit_Task, "TCP", 4096, NULL, 1, &TCP_Task_Handle, 0);
}

void loop() {}