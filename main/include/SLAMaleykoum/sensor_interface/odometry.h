/**
 * @file odometry.h
 * @brief Encoder-based odometry subsystem.
 */

#pragma once
#include "sensor_data_types.h"

namespace SLAMaleykoum {
namespace sensor_interface {

class Odometry {
public:
    Odometry() = default;

    void initialize(float wheel_radius_m,
                    float wheel_base_m,
                    int ticks_per_rev);

    bool isInitialized() const { return initialized_; }

    void reset();

    /// Update from new encoder counts and optional IMU yaw rate.
    void update(int left_ticks, int right_ticks, float imu_gyro_z = 0.0f);

    Pose getPose() const { return current_pose_; }
    Velocity getVelocity() const { return velocity_; }
    PoseDelta getDelta() const { return delta_; }

private:
    bool initialized_{false};
    int last_left_ticks_{0};
    int last_right_ticks_{0};

    float wheel_radius_{0.0f};
    float wheel_base_{0.0f};
    int ticks_per_rev_{0};

    Pose current_pose_{};
    PoseDelta delta_{};
    Velocity velocity_{};
};

}  // namespace sensor_interface
}  // namespace SLAMaleykoum
