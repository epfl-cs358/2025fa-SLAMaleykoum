// Filename: esp1/mapping/loop_closure/loop_closure.h
// Description: Contract for detecting revisited locations and calculating global pose corrections.

#pragma once

#include "common/data_types.h"

/**
 * @brief Responsible for detecting if the robot has returned to a previously mapped area
 * and calculating a precise pose correction.
 */
class LoopClosure {
public:
    LoopClosure();

    /**
     * @brief Processes a new LiDAR scan and current SLAM pose to check for a loop closure.
     * @param scan The current LiDAR feature scan.
     * @param current_pose The robot's current estimated pose.
     * @return A valid LoopClosureCorrection if a closure is found, otherwise a default/null correction.
     */
    LoopClosureCorrection check_for_closure(const LiDARScan& scan, const Pose2D& current_pose);

private:
    // Stores historical scans and poses (e.g., keyframes)
};
