// Filename: common/data_types.h
// Description: Defines shared data structures for pose,
//          sensors, and inter-processor communication (IPC).

#pragma once

#include <vector>
#include <cstdint>

#ifdef ARDUINO
    #include <Arduino.h>
#endif
  


// Define the maximum number of waypoints the controller can handle.
#define MAX_PATH_LENGTH 50
#define MAX_LIDAR_POINTS 3000

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

// --- MAPPING AND PLANNING DATA ---
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
 * @brief Represents a single raw point from the LiDAR sensor in standard mode.
 */
typedef struct rawScanDataPoint
{
	uint8_t quality;
	uint8_t angle_low;
	uint8_t angle_high;
	uint8_t distance_low;
	uint8_t distance_high;
} rawScanDataPoint_t;

// Descriptor and packet types for LiDAR communication
typedef uint8_t rp_descriptor_t[7];
typedef uint8_t rq_Packet_t[9];
typedef uint8_t rq_message_t[2];

// --- LiDAR Communication Enums ---
enum enDescriptor
{
		legacyVersion,  ///< Legacy scan version
		extendedVersion, ///< Extendet scan version
		denseVersion,	 ///< Dense scan version
		startScan		 ///< start scan
};

// --- LiDAR Request Enums ---
enum enRequest
{
	rq_stop,
	rq_reset,
	rq_scan,
	rq_scanExpress
};

/**
 * @brief Detected landmark from LiDAR scan processing.
 */
struct LiDARLandmark {
    float range;   // Distance to the landmark (m)
    float angle;   // Angle to the landmark (rad)
    float quality; // Confidence/strength of the feature detection (0.0 to 1.0)
};

/**
 * @brief Full LiDAR scan structure.
 */
/*struct LiDARScan {
    std::vector<LiDARLandmark> landmarks; // The processed list of features
    uint32_t timestamp_ms;
};*/ // c'est quoi ça -----------------------------------------------------------------------------------


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
struct GlobalPathMessage {
    Waypoint path[MAX_PATH_LENGTH]; // Fixed-size array
    uint16_t current_length;        // Actual number of valid waypoints in the array
    uint32_t path_id; 
    uint32_t timestamp_ms;
};

/**
 * @brief Pose correction calculated by the Loop Closure module.
 * Sent from ESP-1 to ESP-2 to correct local pose drift.
 */
struct LoopClosureCorrection {
    float delta_x;    // Correction needed in X (m)
    float delta_y;    // Correction needed in Y (m)
    float delta_theta; // Correction needed in Yaw (rad)
    uint32_t slam_time; // Timestamp of the correction event
};

// --- MISSION DATA ---

/**
 * @brief High-level goal state for the robot.
 */
enum MissionGoalType {
    EXPLORATION_NODE,
    NAVIGATION_TO_WAYPOINT,
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
  MSG_PATH = 1,       // GloablPathMessage ESP1->ESP2
  MSG_CORR = 2,       // LoopClosureCorrection ESP1->ESP2
  MSG_POSE = 3,       // Pose2D ESP2->ESP1
  MSG_TXT = 4
  // maybe more to add
};
