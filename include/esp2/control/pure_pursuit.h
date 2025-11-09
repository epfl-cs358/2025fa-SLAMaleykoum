// Filename: esp2/control/pure_pursuit.h
// Description: Contract for the Pure Pursuit local path tracking algorithm.

#pragma once

#include "common/data_types.h"

/**
 * @brief Implements the Pure Pursuit algorithm for path tracking.
 * Translates a sequence of Waypoints into a MotionCommand.
 */
class PurePursuit {
public:
    PurePursuit(float wheelbase_m);

    /**
     * @brief Updates the path to track.
     * Copies the data from the message into the internal array.
     * Since the path is constantly updated from our current position
     * we can simply overwrite the previous one.
     */
    void set_path(const GlobalPathMessage& path_msg);

    /**
     * @brief Main control loop function.
     * Computes the required steering and velocity based on the current pose and the path.
     * @param current_pose The robot's current local pose from the EKFLocalizer.
     * @return The required MotionCommand (target velocity and steering angle).
     */
    MotionCommand compute_command(const Pose2D& current_pose);

    // TODO: Do we actually need this???? Seems useless since we shouldn't ever "finish" a path...
    /**
     * @brief Check if the end of the path has been reached.
     */
    bool is_path_complete() const;

private:
    // Internal path storage (fixed size, same as in GlobalPathMessage)
    Waypoint current_path_[MAX_PATH_LENGTH];
    uint16_t path_length_ = 0;      // Actual number of valid points in current_path_
    
    size_t last_target_index_ = 0;  // Index of the last waypoint found as a target
    const float L_ = 0.26;          // Vehicle wheelbase (m)

    // Configuration parameters
    float K_dd_ = 0.5f;             // Gain for lookahead distance (Ld = K_dd_ * speed)
    float K_v_ = 0.5f;              // Speed gain. eg., K_v_ = 0.5 means at max steering angle, speed is halved. 
    float max_lookahead_dist_ = 3.0f;
    float min_lookahead_dist_ = 0.5f;
    // TODO: Check these speeds with @cléa
    float max_speed_ = 0.5f;
    float min_speed_ = 0.2f;

    // Helper functions
    float calculate_lookahead_distance(float current_speed) const;
    float calculate_target_speed(float steering_angle) const;

    // Returns the Waypoint struct of the chosen lookahead point
    Waypoint find_lookahead_point(const Pose2D& current_pose, float lookahead_dist); 
};