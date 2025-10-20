// Filename: esp2/localization/ekf_localizer.h
// Description: Contract for the local, high-frequency EKF for pose estimation.

#pragma once

#include "common/data_types.h"
#include <Eigen/Dense>

// NOTE: The raw accelerometer data should not go directly to the PID.
//      Instead, it is fed into the EKFLocalizer where it performs its most valuable task:
//      Non-Holonomic Constraint Correction: If the robot is driving straight, the
//      accelerometer should report zero lateral acceleration (a_y​≈0). If a_y suddenly
//      spikes, it means the wheel has slipped sideways.
//      The EKF uses this information to decrease its confidence in the odometry reading,
//      leading to a more accurate and robust final pose and velocity estimate for the PID.

/**
 * @brief Implements an EKF (running at 50-100Hz ?) to fuse wheel odometry and IMU data.
 * Provides the fast, smooth local pose estimate.
 */
class EKFLocalizer {
public:
    EKFLocalizer(const Pose2D& initial_pose);

    /**
     * @brief Performs the EKF Prediction and Correction cycle.
     * @param odometry New wheel odometry readings.
     * @param imu New IMU readings.
     */
    void update_pose(const OdometryData& odometry, const IMUData& imu);

    /**
     * @brief Applies a global pose adjustment received from ESP_1's Loop Closure module.
     */
    void apply_global_correction(const LoopClosureCorrection& correction);

    /**
     * @brief Gets the latest estimated local pose (used by Pure Pursuit).
     */
    Pose2D get_current_pose() const;

    /**
     * @brief Gets the latest estimated velocity (used by PID Control and for telemetry).
     */
    Velocity get_current_velocity() const;

private:
    // Eigen::Vector3d state_vector_; // x, y, theta
    // Eigen::Matrix3d covariance_matrix_; // P
};
