#include "esp2/localization/ekf_localizer.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================================================================
// Angle wrapping helper
// ================================================================

float EKFLocalizer::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.f * M_PI;
    while (a < -M_PI) a += 2.f * M_PI;
    return a;
}
// ================================================================
// Compute yaw for imu
// ================================================================
float computeYawFromIMU(const IMUData& imu) {
    float qw = imu.qw;
    float qx = imu.qx;
    float qy = imu.qy;
    float qz = imu.qz;

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    return atan2f(siny_cosp, cosy_cosp);
}

// ================================================================
// Constructor
// ================================================================
EKFLocalizer::EKFLocalizer(const Pose2D& initial_pose)
    : last_imu_timestamp_ms_(0),
      last_odom_timestamp_ms_(0),
      last_theta_imu(0.0f)
{
    // -------------------------
    // State initialization
    // state = [x, y, theta, v_forward]
    // -------------------------
    state_.v[0] = initial_pose.x;
    state_.v[1] = initial_pose.y;
    state_.v[2] = initial_pose.theta;
    state_.v[3] = 0.0f;   // start with zero forward speed

    // -------------------------
    // Initial covariance P
    // -------------------------
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            P_.m[i][j] = 0.0f;

    P_.m[0][0] = 0.01f;   // x
    P_.m[1][1] = 0.01f;   // y
    P_.m[2][2] = 0.01f;   // theta
    P_.m[3][3] = 0.10f;   // v_forward

    // -------------------------
    // Process noise Q  (tunable)
    // -------------------------
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Q_.m[i][j] = 0.0f;

    Q_.m[0][0] = 1e-3f;  // x
    Q_.m[1][1] = 1e-3f;  // y
    Q_.m[2][2] = 5e-4f;  // theta
    Q_.m[3][3] = 5e-2f;  // v_forward

    // -------------------------
    // Measurement noise (encoder velocity)
    // -------------------------
    base_R_v_        = 0.02f; // (m/s)^2
    slip_R_v_factor_ = 6.0f;
    slip_a_lat_thresh_ = 1.5f;
    R_v_ = base_R_v_;

    current_velocity_.v_linear  = 0.f;
    current_velocity_.v_angular = 0.f;
}

// ================================================================
// Public update interface
// ================================================================
void EKFLocalizer::update_pose(const OdometryData& odom, const IMUData& imu) {
    // 1) Predict from IMU
    predict(imu);

    // 2) Use IMU lateral accel to detect slip → adjust noise
    updateMeasurementNoiseFromIMU(imu);

    // 3) Correct using wheel encoder
    updateWithOdometry(odom);
}

// ================================================================

Pose2D EKFLocalizer::get_current_pose() const {
    return { state_.v[0], state_.v[1], state_.v[2] };
}

Velocity EKFLocalizer::get_current_velocity() const {
    return current_velocity_;
}

// ================================================================
// PREDICTION STEP — replaces original Eigen-based version
// ================================================================
void EKFLocalizer::predict(const IMUData& imu)
{
    // =======================
    // Coordinate convention:
    //
    //  Global frame: X = right,   Y = forward
    //  Body frame:   X = right,   Y = forward
    //  Theta = yaw, 0 rad = facing +Y
    //
    // Motion model:
    //   x_dot = v * sin(theta)
    //   y_dot = v * cos(theta)
    //   v_dot = acc_y   (because acc_y = forward on THIS robot)
    // =======================

    uint32_t ts = imu.timestamp_ms;

   
    // -------------------------
    // 1) dt computation
    // -------------------------
    float dt = (ts - last_imu_timestamp_ms_) / 1000000.0f;
    if (dt <= 0.f) dt = 1e-6f;
    last_imu_timestamp_ms_ = ts;

    // -------------------------
    // 2) State unpacking
    // -------------------------
    float x     = state_.v[0];
    float y     = state_.v[1];
    float theta = state_.v[2];
    float v     = state_.v[3];

    // -------------------------
    // 3) IMU inputs
    // -------------------------
    /*
    // in predict():
static bool first = true;
static float last_theta = 0.0f;

float theta_imu = computeYawFromIMU(imu);
theta_imu = wrapAngle(theta_imu);

float omega;
if (first) {
    omega = 0.0f;
    first = false;
} else {
    float dtheta = wrapAngle(theta_imu - last_theta);
    omega = dtheta / dt;
}
last_theta = theta_imu;
last_omega_z_ = omega;
    */
   ////////////////////////////////////////change this according to imu_calibration.cpp test 3
    float theta_imu = computeYawFromIMU(imu);   // yaw rate
    float acc_f   = imu.acc_y;     // forward acceleration (Y-forward!!)
    last_theta_imu = theta_imu;  // cache for velocity output

    // -------------------------
    // 4) Predict velocity + heading
    // -------------------------
    float v_pred     = v + acc_f * dt;

    // -------------------------
    // 5) Predict position
    // -------------------------
    float s = sinf(theta_imu);
    float c = cosf(theta_imu);

    float x_pred = x - v_pred * s * dt;
    float y_pred = y + v_pred * c * dt;
    

    // -------------------------
    // 6) Update state vector
    // -------------------------
    state_.v[0] = x_pred;
    state_.v[1] = y_pred;
    state_.v[2] = theta_imu;
    state_.v[3] = v_pred;

    // -------------------------
    // 7) Build Jacobian F
    // -------------------------
    Mat4 F = Mat4::identity();

    F.m[0][2] =  v_pred * c * dt;  // ∂x/∂theta
    F.m[0][3] =  s * dt;           // ∂x/∂v
    F.m[1][2] = -v_pred * s * dt;  // ∂y/∂theta
    F.m[1][3] =  c * dt;           // ∂y/∂v

    // -------------------------
    // 8) Covariance update:
    //     P = F · P · Fᵀ + Q
    // -------------------------
    Mat4 Ft   = transpose(F);
    Mat4 FP   = mul(F, P_);
    Mat4 FPFt = mul(FP, Ft);

    P_ = FPFt + Q_;
}

// ================================================================
// SLIP DETECTION → measurement noise adjustment
// ================================================================
void EKFLocalizer::updateMeasurementNoiseFromIMU(const IMUData& imu) {
    float a_lat = imu.acc_x;   // X-axis = lateral

    if (fabs(a_lat) > slip_a_lat_thresh_) {
        R_v_ = base_R_v_ * slip_R_v_factor_;
    } else {
        R_v_ = base_R_v_;
    }
}

// ================================================================
// UPDATE STEP — replaces original Eigen-based version
// ================================================================
void EKFLocalizer::updateWithOdometry(const OdometryData& odom)
{
    uint32_t ts = odom.timestamp_ms;

    if (!enc_initialized) {
        last_odom_timestamp_ms_ = ts;
        enc_initialized = true;
        return;
    }

    // -------------------------
    // 1) compute dt for odometry
    // -------------------------
    float dt = (ts - last_odom_timestamp_ms_) / 1000000.0f;
    if (dt <= 0.f) {
        last_odom_timestamp_ms_ = ts;
        return;
    }
    last_odom_timestamp_ms_ = ts;

    // -------------------------
    // 2) Measured forward velocity
    // -------------------------
    float v_meas = odom.delta_distance / dt;

    // -------------------------
    // 3) Predicted forward velocity (from state)
    // -------------------------
    float v_pred = state_.v[3];

    // Innovation y = z - h(x)
    float y = v_meas - v_pred;

    // -------------------------
    // 4) H = [0 0 0 1]  (we only measure forward velocity)
    // -------------------------
    Row4 H{};
    H.v[0] = 0.f;
    H.v[1] = 0.f;
    H.v[2] = 0.f;
    H.v[3] = 1.f;

    // -------------------------
    // 5) Innovation covariance:
    //     S = H · P · Hᵀ + R
    // -------------------------
    Row4 HP = mul(H, P_);  // 1x4
    float S = dot(HP, transpose(H)) + R_v_;
    if (S < 1e-6f) return;

    float Sinv = 1.f / S;

    // -------------------------
    // 6) Kalman gain:
    //     K = P · Hᵀ * (1/S)
    // -------------------------
    Vec4 Ht{};
    for (int i = 0; i < 4; i++) Ht.v[i] = H.v[i];
    Vec4 K = mul(P_, Ht) * Sinv;

    // -------------------------
    // 7) State update:
    //     x ← x + K*y
    // -------------------------
    state_ = state_ + (K * y);

    // -------------------------
    // 8) Covariance update:
    //     P ← (I - K·H) P
    // -------------------------
    Mat4 I = Mat4::identity();
    Mat4 KH = outerMul(K, H);
    Mat4 M  = I - KH;
    P_ = mul(M, P_);

    // -------------------------
    // 9) Cache velocity for consumers (PID)
    // -------------------------
    current_velocity_.v_linear  = state_.v[3];
    current_velocity_.v_angular = last_theta_imu;
}
