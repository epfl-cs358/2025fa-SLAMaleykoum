// Filename: common/data_types.h
// Description: Defines shared data structures for pose,
//          sensors, and inter-processor communication (IPC).

#pragma once

#include <vector>
#include <cstdint>

// --- Core Geometric Structures ---
/**
 * @brief 2D pose structure (Position and Orientation).
 */
struct Pose2D {
    float x;     // Global X position (m)
    float y;     // Global Y position (m)
    float theta; // Global Yaw angle (rad)
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
    float delta_distance_left;  // Change in distance for left wheel (m)
    float delta_distance_right; // Change in distance for right wheel (m)
    uint32_t timestamp_ms;      // Timestamp of the measurement
};

/**
 * @brief Raw data from Inertial Measurement Unit (IMU).
 * Updated to include full 3-axis gyroscope and accelerometer data
 * for robust EKF prediction and non-holonomic constraint enforcement.
 */
struct IMUData {
    // Gyroscope data (Angular Velocity)
    float omega_x;  // Angular velocity around X-axis (rad/s) - typically roll
    float omega_y;  // Angular velocity around Y-axis (rad/s) - typically pitch
    float omega_z;  // Angular velocity around Z-axis (rad/s) - typically yaw
    
    // Accelerometer data (Linear Acceleration)
    float acc_x;    // Acceleration along X-axis (m/s^2) - typically forward
    float acc_y;    // Acceleration along Y-axis (m/s^2) - typically lateral
    float acc_z;    // Acceleration along Z-axis (m/s^2) - typically gravity/vertical
    
    uint32_t timestamp_ms; // Timestamp of the measurement
};

// --- MAPPING AND PLANNING DATA ---

/**
 * @brief Represents a distinctive feature/landmark extracted from LiDAR data.
 */
struct LiDARLandmark {
    float range;   // Distance to the landmark (m)
    float angle;   // Angle to the landmark (rad)
    float quality; // Confidence/strength of the feature detection (0.0 to 1.0)
};

/**
 * @brief Full LiDAR scan structure.
 */
struct LiDARScan {
    std::vector<LiDARLandmark> landmarks; // The processed list of features
    uint32_t timestamp_ms;
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
 * @brief Message containing a vector of waypoints from the Global Planner.
 */
//TODO: This is probably going to change!
struct GlobalPathMessage {
    std::vector<Waypoint> path;
    uint32_t path_id; // Identifier for the path
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
