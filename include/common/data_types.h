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
struct SystemHealth {
    uint32_t free_heap;
    uint32_t min_free_heap;
    
    // Execution Times
    uint32_t map_update_time_us;
    uint32_t global_plan_time_us;
    uint32_t mission_plan_time_us;
    
    // Stack Monitoring (Lowest amount of bytes ever left in the stack)
    // If these get near 0, the CPU will reset.
    uint16_t stack_min_tcp;
    uint16_t stack_min_gplan;
    uint16_t stack_min_mplan;
    uint16_t stack_min_lidar;
    
    // Connectivity
    uint32_t last_esp2_packet_ms; // Time since last packet from ESP2
};

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
struct IMUData : public Printable {
    // Gyroscope data (Angular Velocity)
    // float omega_x;  // Angular velocity around X-axis (rad/s) - typically roll
    // float omega_y;  // Angular velocity around Y-axis (rad/s) - typically pitch
    // float omega_z;  // Angular velocity around Z-axis (rad/s) - typically yaw
    
    // Accelerometer data (Linear Acceleration)
    float acc_x;    // Acceleration along X-axis (m/s^2) - typically forward
    float acc_y;    // Acceleration along Y-axis (m/s^2) - typically lateral
    float acc_z;    // Acceleration along Z-axis (m/s^2) - typically gravity/vertical

    float qx;   // quaternion (absolute orientation)
    float qy;
    float qz;
    float qw;
    
    uint32_t timestamp_ms; // Timestamp of the measurement

    // Fonction used by Serial.print()
    size_t printTo(Print& p) const {
        size_t n = 0;
        n += p.print(F("IMUData { "));
        n += p.print(F("omega=("));
        // n += p.print(omega_x); n += p.print(F(", "));
        // n += p.print(omega_y); n += p.print(F(", "));
        // n += p.print(omega_z); n += p.print(F("), "));

        n += p.print(F("acc=("));
        n += p.print(acc_x); n += p.print(F(", "));
        n += p.print(acc_y); n += p.print(F(", "));
        n += p.print(acc_z); n += p.print(F("), "));

        n += p.print(F("quat=("));
        n += p.print(qx); n += p.print(F(", "));
        n += p.print(qy); n += p.print(F(", "));
        n += p.print(qz); n += p.print(F("), "));

        n += p.print(F("timestamp="));
        n += p.print(timestamp_ms);
        n += p.print(F(" }"));
        return n;
    }

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
    IDLE
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
