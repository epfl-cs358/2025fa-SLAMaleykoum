// Filename: esp1/mapping/slam/landmark_map.h
// Description: Utility for managing the sparse list of landmarks for EKF-P SLAM.
// This is the lightweight, probabilistic map for V1.

#pragma once

#include "common/data_types.h"
#include <vector>

/**
 * @brief Manages the list of landmarks currently tracked in the EKF-P state vector.
 * This class handles landmark association and initial addition to the EKF state.
 */
class LandmarkMap {
public:
    /**
     * @brief Looks up a detected landmark (e.g., using a KD-tree or simple search)
     * to determine if it's a known landmark.
     * @param observed_landmark Landmark data from LiDAR.
     * @param robot_pose Current pose from EKF.
     * @return Index of the known landmark, or -1 if it's a new observation.
     */
    int find_known_landmark(const LiDARLandmark& observed_landmark, const Pose2D& robot_pose);

    /**
     * @brief Adds a new, initial estimate of a landmark to the map.
     * @param initial_estimate The calculated global position of the new landmark.
     * @return The index of the newly added landmark.
     */
    int add_new_landmark(const Pose2D& initial_estimate);

    /**
     * @brief Retrieves the full list of estimated landmark poses (for planning/visualization).
     */
    std::vector<Pose2D> get_all_landmark_poses() const;

private:
    std::vector<Pose2D> landmark_list_; // Stores the latest estimated pose of each landmark
};
