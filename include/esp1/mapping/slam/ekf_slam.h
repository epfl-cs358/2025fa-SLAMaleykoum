// Filename: esp1/mapping/slam/ekf_slam.h
// Description: Contract for the EKF-based SLAM algorithm on ESP_1.

// NOTE: This might be too big of a chunk, might refactor into smaller pieces.

#pragma once

#include "common/data_types.h"
#include <vector>

/**
 * @brief Implements the Extended Kalman Filter (EKF) for Simultaneous
 *          Localization and Mapping (SLAM).
 * Fuses Odometry, IMU, and LiDAR landmark data to estimate global pose
 * and landmark positions.
 */
class EKF_SLAM {
public:
    /**
     * @brief Initializes the SLAM EKF with an initial pose and covariance.
     */
    EKF_SLAM(const Pose2D& initial_pose);

    /**
     * @brief Prediction step of the EKF, driven by motion model (odometry/IMU).
     * @param odometry The latest wheel odometry reading.
     * @param imu The latest IMU reading.
     */
    // TODO: Double check with assistants/coaches if it would be corret to give
    //      it the post ekf data directly. Apparently not because it includes
    //      possibility of cascading errors. BUT STILL WORTH ASKING TO CLARIFY.
    void predict(const OdometryData& odometry, const IMUData& imu);

    /**
     * @brief Update step of the EKF, driven by LiDAR landmark observations.
     * @param landmarks A list of newly observed LiDAR landmarks.
     */
    void update(const std::vector<LiDARLandmark>& landmarks);

    /**
     * @brief Applies a global correction calculated during a loop closure event.
     */
    void apply_global_correction(const LoopClosureCorrection& correction);

    /**
     * @brief Retrieves the current best estimate of the global robot pose.
     */
    Pose2D get_current_global_pose() const;

private:
    Pose2D current_pose_;
    // Internal state management variables (e.g., Eigen matrices for state and covariance)
    // The landmark states are managed internally within the EKF state vector.
};
