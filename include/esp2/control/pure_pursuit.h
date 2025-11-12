// Filename: esp2/control/pure_pursuit.h
// Description: Contract for the Pure Pursuit local path tracking algorithm.

#pragma once

#include "common/data_types.h"
#include <vector>

/**
 * @brief Implements the Pure Pursuit algorithm for path tracking.
 * Translates a sequence of Waypoints into a MotionCommand.
 */
class PurePursuit {
public:
    PurePursuit();

    /**
     * @brief Updates the path currently being tracked.
     */
    void set_path(const GlobalPathMessage& path_msg);

    /**
     * @brief Main control loop function.
     * Computes the required steering and velocity based on the current pose and the path.
     * @param current_pose The robot's current local pose from the EKFLocalizer.
     * @return The required MotionCommand (target velocity and steering angle).
     */
    MotionCommand compute_command(const Pose2D& current_pose);

    /**
     * @brief Check if the end of the path has been reached.
     */
    bool is_path_complete() const;

private:
    // std::vector<Waypoint> current_path_;
    // size_t lookahead_index_;
};
