#include "slip_localizer2.h"
#include <math.h>

// -------------------------------------------------------------
// Utility
// -------------------------------------------------------------
float SlipLocalizer::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.f * M_PI;
    while (a < -M_PI) a += 2.f * M_PI;
    return a;
}

// -------------------------------------------------------------
// Constructor
// -------------------------------------------------------------
SlipLocalizer::SlipLocalizer(const Pose2D& initial_pose)
    : x_(initial_pose.x),
      y_(initial_pose.y),
      theta_(initial_pose.theta),
      v_enc_(0.0f),
      v_imu_(0.0f),
      v_trusted_(0.0f),
      lateral_slip_threshold_(1.5f),
      forward_slip_threshold_(0.20f)
{
}

// -------------------------------------------------------------
// Update step: fuse encoder + IMU using slip logic
// -------------------------------------------------------------
void SlipLocalizer::update(float encoder_velocity, const IMUData& imu, float dt)
{
    if (dt <= 0.0f) return;

    // ---------------------------------------------------------
    // 1) Yaw from IMU quaternion
    // ---------------------------------------------------------
    float qw = imu.qw, qx = imu.qx, qy = imu.qy, qz = imu.qz;
    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    theta_ = wrapAngle(atan2f(siny, cosy));

    // ---------------------------------------------------------
    // 2) Encoder-based forward velocity
    // ---------------------------------------------------------
    v_enc_ = encoder_velocity;

    // ---------------------------------------------------------
    // 3) IMU integrated forward velocity
    //    acc_y = forward acceleration on your robot
    // ---------------------------------------------------------
    v_imu_ += imu.acc_y * dt;

    

    // ---------------------------------------------------------
    // 4) Slip detection
    // ---------------------------------------------------------
    bool lateral_slip  = fabsf(imu.acc_x) > lateral_slip_threshold_;
    bool forward_slip  = fabsf(v_imu_ - v_enc_) > forward_slip_threshold_;

    // ---------------------------------------------------------
    // 5) Choose trusted velocity
    // ---------------------------------------------------------
    if (lateral_slip || forward_slip) {
        v_trusted_ = v_imu_;  // trust IMU
    } else {
        v_trusted_ = v_enc_;  // trust encoder

        // softly realign IMU velocity to avoid drift
        //v_imu_ = 0.9f * v_imu_ + 0.1f * v_enc_;
    }

    // ---------------------------------------------------------
    // 6) Update global position
    // ---------------------------------------------------------
    float s = sinf(theta_);
    float c = cosf(theta_);

    x_ -= v_trusted_ * s * dt;
    y_ += v_trusted_ * c * dt;
}

// -------------------------------------------------------------
// Getters
// -------------------------------------------------------------
Pose2D SlipLocalizer::get_pose() const {
    Pose2D p;
    p.x = x_;
    p.y = y_;
    p.theta = theta_;
    return p;
}

Velocity SlipLocalizer::get_velocity() const {
    Velocity v;
    v.v_linear  = v_trusted_;
    return v;
}
