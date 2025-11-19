// Filename: esp2/localization/ekf_localizer.cpp

#include "ekf_localizer.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

// =======================
// Internal EKF implementation
// =======================
//
// Coordinate convention used here:
//  - Global frame: X to the right, Y forward.
//  - Body frame:   X to the right, Y forward.
//  - Theta (Pose2D::theta): yaw, 0 rad means facing +Y (forward).
//
// Therefore, for forward speed v in body frame:
//   x_dot = v * sin(theta)
//   y_dot = v * cos(theta)
//
// ALSO: although IMUData comments say acc_x is "typically forward",
//       in THIS car:   acc_y = forward, acc_x = lateral.

class EKFLocalizerImpl {
public:
    // State: [x, y, theta, v_linear]^T
    Eigen::Vector4d x_;   // state
    Eigen::Matrix4d P_;   // covariance

    Eigen::Matrix4d Q_;                   // process noise
    Eigen::Matrix<double, 1, 1> R_v_;     // measurement noise for encoder velocity

    bool initialized_ = false;
    uint32_t last_imu_timestamp_ms_  = 0;
    uint32_t last_odom_timestamp_ms_ = 0;

    Velocity current_velocity_{};         // for get_current_velocity()
    double last_omega_z_ = 0.0;           // store last yaw rate from IMU

    // Slip handling parameters (tune these!)
    double base_R_v_          = 0.02; // baseline noise on encoder velocity [ (m/s)^2 ]
    double slip_R_v_factor_   = 6.0;  // multiply R when lateral acceleration is large
    double slip_ay_threshold_ = 1.5;  // [m/s^2] threshold for "suspicious" lateral accel

    EKFLocalizerImpl(const Pose2D& initial_pose) {
        // Initialize state from given pose, with zero initial speed
        x_ << initial_pose.x,
              initial_pose.y,
              initial_pose.theta,
              0.0;

        // Initial covariance: fairly confident in pose, less in speed.
        P_.setZero();
        P_(0,0) = 0.01;   // x
        P_(1,1) = 0.01;   // y
        P_(2,2) = 0.01;   // theta
        P_(3,3) = 0.10;   // v_linear

        // Process noise Q (tune these values experimentally!)
        Q_.setZero();
        Q_(0,0) = 1e-3;   // x
        Q_(1,1) = 1e-3;   // y
        Q_(2,2) = 5e-4;   // theta
        Q_(3,3) = 5e-2;   // v_linear

        // Measurement noise R for encoder-derived velocity
        R_v_(0,0) = base_R_v_;

        current_velocity_.v_linear  = 0.0f;
        current_velocity_.v_angular = 0.0f;
    }

    static double wrapAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // ------------------------
    // Prediction step (IMU)
    // ------------------------
    void predict(const IMUData& imu) {
        const uint32_t ts = imu.timestamp_ms;

        if (!initialized_) {
            last_imu_timestamp_ms_ = ts;
            initialized_ = true;
            return;
        }

        double dt = (ts - last_imu_timestamp_ms_) / 1000.0;
        if (dt <= 0.0) {
            dt = 1e-3; // fallback small dt
        }
        last_imu_timestamp_ms_ = ts;

        // Unpack state
        double x     = x_(0);
        double y     = x_(1);
        double theta = x_(2);
        double v     = x_(3);

        // Inputs from IMU
        const double omega_z = imu.omega_z;   // yaw rate [rad/s]
        const double acc_f   = imu.acc_y;     // forward acceleration [m/s^2] (Y-forward)
        // const double acc_lat = imu.acc_x;  // lateral acceleration (X-right), used only for slip

        last_omega_z_ = omega_z;

        // --- Nonlinear motion model ---
        // v_{k+1} = v_k + a_forward * dt
        double v_pred = v + acc_f * dt;

        // theta_{k+1} = theta_k + omega_z * dt
        double theta_pred = theta + omega_z * dt;
        theta_pred = wrapAngle(theta_pred);

        // x_{k+1} = x_k + v_pred * sin(theta_pred) * dt
        // y_{k+1} = y_k + v_pred * cos(theta_pred) * dt
        const double sin_t = std::sin(theta_pred);
        const double cos_t = std::cos(theta_pred);

        double x_pred = x + v_pred * sin_t * dt;
        double y_pred = y + v_pred * cos_t * dt;

        x_ << x_pred, y_pred, theta_pred, v_pred;

        // --- Jacobian F = ∂f/∂x ---
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();

        // With x = v * sin(theta), y = v * cos(theta):
        //
        // ∂x/∂theta =  v_pred * cos(theta_pred) * dt
        // ∂x/∂v     =  sin(theta_pred) * dt
        F(0,2) =  v_pred * cos_t * dt;
        F(0,3) =  sin_t * dt;

        // ∂y/∂theta = -v_pred * sin(theta_pred) * dt
        // ∂y/∂v     =  cos(theta_pred) * dt
        F(1,2) = -v_pred * sin_t * dt;
        F(1,3) =  cos_t * dt;

        // theta and v have identity derivatives wrt themselves (already set)

        // Covariance prediction
        P_ = F * P_ * F.transpose() + Q_;
    }

    // ------------------------
    // Update step (Odometry)
    // ------------------------
    void update_with_odometry(const OdometryData& odom) {
        // Convert delta_distance to velocity using odometry timestamps
        const uint32_t ts = odom.timestamp_ms;

        if (!initialized_) {
            // If predict() was never called, we don't know dt yet
            last_odom_timestamp_ms_ = ts;
            return;
        }

        double dt_odom = (ts - last_odom_timestamp_ms_) / 1000.0;
        if (dt_odom <= 0.0) {
            dt_odom = 0.0;
        }
        last_odom_timestamp_ms_ = ts;

        if (dt_odom <= 0.0f) {
            // No time passed, can't form a velocity measurement
            return;
        }

        // Measurement: encoder-derived forward velocity [m/s]
        const double v_meas = static_cast<double>(odom.delta_distance) / dt_odom;

        // Expected measurement h(x) = v_linear = x_(3)
        double v_pred = x_(3);

        // Innovation / residual
        double y = v_meas - v_pred;

        // Measurement Jacobian H: dh/dx = [0 0 0 1]
        Eigen::Matrix<double, 1, 4> H;
        H << 0.0, 0.0, 0.0, 1.0;

        // Innovation covariance S = H P H^T + R
        Eigen::Matrix<double, 1, 1> S = H * P_ * H.transpose() + R_v_;

        // Kalman gain K = P H^T S^{-1}
        Eigen::Matrix<double, 4, 1> K = P_ * H.transpose() * S.inverse();

        // State update
        x_ = x_ + K * y;

        // Covariance update: P = (I - K H) P
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P_ = (I - K * H) * P_;

        // Update cached velocity for external access
        current_velocity_.v_linear  = static_cast<float>(x_(3));
        current_velocity_.v_angular = static_cast<float>(last_omega_z_);
    }

    // ------------------------
    // Slip-sensitive R update (non-holonomic constraint)
    // ------------------------
    void update_R_from_IMU(const IMUData& imu) {
        // Lateral acceleration is along X (right) in this car
        const double a_lat = imu.acc_x;

        if (std::fabs(a_lat) > slip_ay_threshold_) {
            // Suspect slip -> odometry unreliable -> increase measurement noise
            R_v_(0,0) = base_R_v_ * slip_R_v_factor_;
        } else {
            // Normal driving
            R_v_(0,0) = base_R_v_;
        }
    }

    // ------------------------
    // Global correction (loop closure)
    // ------------------------
    void apply_global_correction(const LoopClosureCorrection& c) {
        x_(0) += c.delta_x;
        x_(1) += c.delta_y;
        x_(2) = wrapAngle(x_(2) + c.delta_theta);
        // You can optionally shrink P_ here if you want.
    }

    // ------------------------
    // Accessors
    // ------------------------
    Pose2D current_pose() const {
        Pose2D pose;
        pose.x     = static_cast<float>(x_(0));
        pose.y     = static_cast<float>(x_(1));
        pose.theta = static_cast<float>(x_(2));
        return pose;
    }

    Velocity current_velocity() const {
        return current_velocity_;
    }
};

// =======================
// EKFLocalizer interface
// =======================

EKFLocalizer::EKFLocalizer(const Pose2D& initial_pose)
    : impl_(std::make_shared<EKFLocalizerImpl>(initial_pose)) {}

// Main update: prediction (IMU) + R update (slip) + correction (odometry)
void EKFLocalizer::update_pose(const OdometryData& odometry, const IMUData& imu) {
    // 1) Prediction using IMU
    impl_->predict(imu);

    // 2) Adjust measurement noise based on lateral acceleration (non-holonomic constraint)
    impl_->update_R_from_IMU(imu);

    // 3) Correction using encoder odometry
    impl_->update_with_odometry(odometry);
}

void EKFLocalizer::apply_global_correction(const LoopClosureCorrection& correction) {
    impl_->apply_global_correction(correction);
}

Pose2D EKFLocalizer::get_current_pose() const {
    return impl_->current_pose();
}

Velocity EKFLocalizer::get_current_velocity() const {
    return impl_->current_velocity();
}
