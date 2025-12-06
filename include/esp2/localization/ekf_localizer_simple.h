#pragma once

#include "common/data_types.h"
#include <stdint.h>

/**
 * @brief Simple slip-aware localizer (no EKF, no covariance).
 * 
 * Functionality:
 *  - Integrate IMU acceleration -> IMU velocity
 *  - Compare IMU velocity with encoder velocity to detect forward slip
 *  - Use lateral accel (acc_x) to detect sideways slip
 *  - Choose trusted velocity (IMU or encoder)
 *  - Integrate position using trusted velocity + yaw from IMU
 * 
 * State:
 *   x, y, theta, v_linear
 */

class SlipLocalizer
{
public:
    SlipLocalizer(const Pose2D& initial_pose);

    /// Update localizer using encoder + IMU data.
    void update(float encoder_velocity, const IMUData& imu, float dt);

    /// Get estimated current pose
    Pose2D get_pose() const;

    /// Get estimated current velocity (linear + angular)
    Velocity get_velocity() const;

private:
    // --- Internal state ---
    float x_;
    float y_;
    float theta_;

    float v_enc_;       // encoder velocity
    float v_imu_;       // imu integrated velocity
    float v_trusted_;   // fused velocity (output)

    float omega_;       // orientation

    // --- previous values for integration ---
    float last_theta_;
    bool theta_initialized_;

    // --- slip detection thresholds ---
    float lateral_slip_threshold_;   // acc_x threshold
    float forward_slip_threshold_;   // |v_imu - v_enc|

    // --- Utility ---
    static float wrapAngle(float a);
};
