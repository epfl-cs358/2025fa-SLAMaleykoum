// Filename: esp2/localization/ekf_localizer.h

#pragma once

#include "common/data_types.h"
#include <eigen3/Eigen/Dense>
#include <memory>

class EKFLocalizerImpl;  // forward declaration

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
    std::shared_ptr<EKFLocalizerImpl> impl_;
    // Old idea:
    // Eigen::Vector3d state_vector_; // x, y, theta
    // Eigen::Matrix3d covariance_matrix_; // P
};
