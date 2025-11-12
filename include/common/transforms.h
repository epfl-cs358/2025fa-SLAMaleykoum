// This file contains functions to convert between global and robot-centric frames.
// NOTE: The robot's frame has X pointing to the right, Y pointing forward.

#pragma once
#include <cmath>
#include "common/data_types.h"

namespace Transforms { 
    /**
    * @brief Transforms a point from global coordinates to robot-centric coordinates.
    * @param robot_pose The current pose of the robot in global frame.
    * @param point The point in global coordinates to be transformed (will be modified in-place).
    *
    * Assumes:
    * - Global Frame: X-Right, Y-Forward (Origin at robot starting point)
    * - Robot Frame: X-Right, Y-Forward
    * - robot_pose.theta: CCW rotation angle from Global X to Robot X.
    */
    inline void to_robot_frame(const Pose2D& robot_pose, Waypoint& point) {
        // Translate point to robot origin
        float dx = point.x - robot_pose.x;
        float dy = point.y - robot_pose.y;

        // Rotate point by -theta to align with robot's frame
        float x_r = dx * std::cos(robot_pose.theta) + dy * std::sin(robot_pose.theta);
        float y_r = -dx * std::sin(robot_pose.theta) + dy * std::cos(robot_pose.theta);
        
        point.x = x_r;
        point.y = y_r;
    }

    /**
    * @brief Transforms a point from robot-centric coordinates to global coordinates.
    * @param robot_pose The current pose of the robot in global frame.
    * @param point The point in robot-centric coordinates to be transformed (will be modified in-place).
    */
    inline void to_global_frame(const Pose2D& robot_pose, Waypoint& point) {
        // Rotate point by +theta to align with global frame
        float x_g = point.x * std::cos(robot_pose.theta) - point.y * std::sin(robot_pose.theta);
        float y_g = point.x * std::sin(robot_pose.theta) + point.y * std::cos(robot_pose.theta);
        
        // Translate point to global origin
        point.x = x_g + robot_pose.x;
        point.y = y_g + robot_pose.y;
    }
}
