/**
 * @file sensor_data_types.h
 * @brief Shared sensor data structures. No Ros, no comunication code.
 */

#pragma once
#include <array>
#include <cstdint>
#include <chrono>

namespace SLAMaleykoum {
namespace sensor_interface {

/**
 * @brief 2D pose structure (x, y, theta).
 */
struct Pose {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};
};

/**
 * @brief Incremental pose change since last update.
 */
struct PoseDelta {
    float dx{0.0f};
    float dy{0.0f};
    float dtheta{0.0f};
};

/**
 * @brief Linear and angular velocity estimates.
 */
struct Velocity {
    float linear{0.0f};   // m/s
    float angular{0.0f};  // rad/s
};

/**
 * @brief IMU measurement container (2D use but 3D fields for completeness).
 */
struct ImuData {
    // TODO: Should we use 2d only to save memory ?
    std::array<float, 3> accel{0.0f, 0.0f, 0.0f};  // m/s^2
    std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};   // rad/s
    std::chrono::steady_clock::time_point timestamp =
        std::chrono::steady_clock::now();
};

/**
 * @brief LiDAR scan data container (2D planar scan).
 */
struct LidarScan {
    std::array<float, 360> ranges{};  // meters, one per degree (example)
    std::chrono::steady_clock::time_point timestamp =
        std::chrono::steady_clock::now();
};

/**
 * @brief Aggregated sensor snapshot used by the rest of the system.
 */
struct SensorData {
    ImuData imu;
    LidarScan lidar;
    Pose odom_pose;
    Velocity odom_velocity;
};

}  // namespace sensor_interface
}  // namespace SLAMaleykoum
