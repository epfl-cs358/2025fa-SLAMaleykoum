// Filename: common/data_types.h
// Description: Defines shared data structures for pose,
//          sensors, and inter-processor communication (IPC).

#pragma once

#ifdef ARDUINO
    #include <Arduino.h>
#endif
  

// Define the maximum number of waypoints the controller can handle.
#define MAX_PATH_LENGTH 15
#define MAX_LOCAL_PATH_LENGTH 10
#define MAX_LIDAR_POINTS 500
#define MAX_INVALID_GOALS 10
#define ROBOT_RADIUS 0.3f
#define SEARCH_BOUND_M 3.0f
#define GOAL_REACHED 0.4f

// Free and Occupied probability thresholds
#define FREE_BOUND_PROB 0.45 // = -6 en log_odds
#define OCC_BOUND_PROB 0.6  // = 4 en log_odds

#define GP_MAX_W  70     // largeur max de la carte coarse (utilisé dans generate path)
#define GP_MAX_H  70     // hauteur max
#define GP_MAX_CELLS (GP_MAX_W * GP_MAX_H)
#define GP_PQ_SIZE   (GP_MAX_CELLS / 3) // Queue doesn't need to hold the whole map, just the frontier

// --- Debugging & Profiling ---
#define MAX_TRACE_EVENTS 200
#define EVENT_START 1
#define EVENT_END   0

enum TaskID : uint8_t {
    ID_IDLE = 0,
    ID_LIDAR = 1,     // Core 1
    ID_SYNC = 2,      // Core 1
    ID_MAP = 3,       // Core 1
    ID_IPC = 4,       // Core 0
    ID_GPLAN = 5,     // Core 0
    ID_MPLAN = 6,     // Core 0
    ID_TCP = 7,       // Core 0
    ID_VALIDATOR = 8  // Core 0
};

#pragma pack(push, 4)

// Represents one "mark" on the timeline
struct TaskEvent {
    uint32_t timestamp_us;  // When it happened
    uint8_t task_id;        // Which task
    uint8_t type;           // Start or End
    uint8_t core_id;        // 0 or 1
    uint8_t padding;        // Alignment
} __attribute__((packed));

enum PlannerStatus : int32_t { // Changed to int32_t for alignment
    PLANNER_IDLE_MY_TYPE = 0,
    PLANNER_RUNNING_MY_TYPE = 1,
    PLANNER_SUCCESS_MY_TYPE = 2,
    ERR_START_BLOCKED_MY_TYPE = 3,
    ERR_GOAL_BLOCKED_MY_TYPE = 4,
    ERR_NO_PATH_FOUND_MY_TYPE = 5,
    ERR_TIMEOUT_MY_TYPE = 6
};

struct SystemHealth {
    uint32_t free_heap;
    uint32_t min_free_heap;
    
    uint32_t map_update_time_us;
    uint32_t global_plan_time_us;
    uint32_t mission_plan_time_us;
    
    uint32_t stack_min_tcp;
    uint32_t stack_min_gplan;
    uint32_t stack_min_mplan;
    uint32_t stack_min_lidar;
    
    // Connectivity
    uint32_t last_esp2_packet_ms; // Time since last packet from ESP2

    // Diag counts
    uint32_t lidar_frames_processed;
    uint32_t map_frames_processed;
    uint32_t planner_fail_count;
    uint32_t last_planner_status;
};

#pragma pack(pop)

// --- Core Geometric Structures ---
/**
 * @brief 2D pose structure (Position and Orientation).
 */
struct Pose2D {
    float x;     // Global X position (m)
    float y;     // Global Y position (m)
    float theta; // Global Yaw angle (rad)
    uint64_t timestamp_ms;
};

/**
 * @brief Velocity state.
 */
struct Velocity {
    float v_linear;  // Linear speed (m/s)
    float v_angular; // Angular speed (rad/s)
};

/**
 * @brief High-level motion command for local control.
 */
struct Waypoint {
    float x; // Target x-coordinate (m)
    float y; // Target y-coordinate (m)
};


// --- Sensor Data Structures ---

/**
 * @brief Raw data from wheel encoders.
 */
struct OdometryData {
    float delta_distance;  // Change in distance for rear wheels (m)
    uint32_t timestamp_ms;      // Timestamp of the measurement
};

/**
 * @brief Raw data from Inertial Measurement Unit (IMU).
 * Updated to include full 3-axis gyroscope and accelerometer data
 * for robust EKF prediction and non-holonomic constraint enforcement.
 */

#ifdef ARDUINO
struct IMUData {
    float qx;   // quaternion (absolute orientation)
    float qy;
    float qz;
    float qw;
    
    uint32_t timestamp_ms; // Timestamp of the measurement
};
#endif

typedef float MotorOutputs; // PWM = pulse duration in microseconds 
// (1000 µs = full reverse, 1500 µs = neutral, 2000 µs = full forward)

// --- LIDAR DATA ---
/**
 * @brief Single LiDAR scan (360°).
 */
typedef struct {
    uint16_t count;                         // number of valid points
    float angles[MAX_LIDAR_POINTS];         // angle in degrees
    float distances[MAX_LIDAR_POINTS];      // distance in millimeters
    uint8_t qualities[MAX_LIDAR_POINTS];    // quality [0-255]
    uint32_t timestamp_ms;                  // end-of-scan timestamp
} LiDARScan;

/**
 * @brief Synchronized LiDAR scan and robot pose.
 */
struct SyncedScan {
    LiDARScan scan;
    Pose2D pose;
};

/**
 * @brief Represents a single raw point from the LiDAR sensor in standard mode.
 */
typedef struct rawScanDataPoint {
	uint8_t quality;
	uint8_t angle_low;
	uint8_t angle_high;
	uint8_t distance_low;
	uint8_t distance_high;
} rawScanDataPoint_t;

// Descriptor and packet types for LiDAR communication
typedef uint8_t rp_descriptor_t[7];
typedef uint8_t rq_message_t[2];

// --- LiDAR Communication Enums ---
enum enDescriptor {
		legacyVersion,   // Legacy scan version
		extendedVersion, // Extendet scan version
		denseVersion,	 // Dense scan version
		startScan		 // start scan
};

// --- LiDAR Request Enums ---
enum enRequest {
	rq_stop,
	rq_reset,
	rq_scan
};

// --- Inter-Processor Communication (IPC) Structures ---

/**
 * @brief Command from Local Planner (Pure Pursuit) to PID Controller.
 */
struct MotionCommand {
    float v_target;  // Target linear velocity (m/s)
    float delta_target; // Target steering angle (rad)
};

/**
 * @brief Message containing a fixed-size array of waypoints from the Global Planner.
 * The path is constantly updated as new waypoints are generated. The current length
 * indicates how many waypoints in the array are valid, it is also constantly updated.
 * current_length <= MAX_PATH_LENGTH
 */
struct PathMessage {
    Waypoint path[MAX_PATH_LENGTH]; // Fixed-size array
    uint8_t current_length;        // Actual number of valid waypoints in the array
    uint16_t path_id; 
    uint64_t timestamp_ms;
};

// --- MISSION DATA ---

/**
 * @brief High-level goal state for the robot.
 */
enum MissionGoalType {
    EXPLORATION_MODE,
    RETURN_HOME,
};

/**
 * @brief The current mission goal chosen by the Goal Manager.
 */
struct MissionGoal {
    Pose2D target_pose;
    MissionGoalType type;
};

// --- ESPS COMMUNICATION ---
/**
 * @enum MsgID : The IDs of the messages
 * @brief Each message sent begins by a unique id. When receiving, the esp reads the ID 
 * and knos what to do with the following message.
 */
enum MsgId : uint8_t {
  MSG_PATH = 1,       // PathMessage ESP1->ESP2
  MSG_POSE = 2,       // Pose2D ESP2->ESP1
};

struct InvalidGoals {
    uint8_t size = 0;
    Pose2D lasts[MAX_INVALID_GOALS];
};