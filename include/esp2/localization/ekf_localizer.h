#pragma once

#include "common/data_types.h"
#include "mini_mat.h"
#include <stdint.h>

/**
 * @brief Implements an EKF to fuse wheel odometry and IMU data.
 * State: [x, y, theta, v_linear]^T
 *  - X to the right, Y forward, theta=0 facing +Y.
 */
class EKFLocalizer {
public:
    EKFLocalizer(const Pose2D& initial_pose);

    void update_pose(const OdometryData& odometry, const IMUData& imu);
    void apply_global_correction(const LoopClosureCorrection& correction);

    Pose2D get_current_pose() const;
    Velocity get_current_velocity() const;

private:
    // Old idea:
    // Eigen::Vector3d state_vector_; // x, y, theta
    // Eigen::Matrix3d covariance_matrix_; // P

    // New idea: use mini matrix library
    // State vector: [x, y, theta, v_linear]
    Vec4 state_;
    // Covariance and process noise
    Mat4 P_;
    Mat4 Q_;

    // Measurement noise for encoder velocity
    float R_v_;              // current
    float base_R_v_;         // baseline
    float slip_R_v_factor_;  // multiplier on slip
    float slip_a_lat_thresh_; // lateral accel threshold

    bool initialized_;
    uint32_t last_imu_timestamp_ms_;
    uint32_t last_odom_timestamp_ms_;

    Velocity current_velocity_;
    float last_theta_imu;

    // Internal steps
    void predict(const IMUData& imu);
    void updateMeasurementNoiseFromIMU(const IMUData& imu);
    void updateWithOdometry(const OdometryData& odom);

    static float wrapAngle(float a);
};

   
