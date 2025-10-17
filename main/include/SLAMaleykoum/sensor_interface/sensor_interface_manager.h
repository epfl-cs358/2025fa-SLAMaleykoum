/**
 * @file sensor_interface_manager.h
 * @brief Unified sensor manager — IMU, LiDAR, and Odometry.
 */

#pragma once
#include "imu.h"
#include "lidar.h"
#include "odometry.h"
#include "sensor_data_types.h"

namespace SLAMaleykoum {
namespace sensor_interface {

class SensorInterfaceManager {
public:
    SensorInterfaceManager() = default;

    /// Initialize all sensors
    bool initialize();

    /// Poll all sensors and fill a SensorData struct.
    /// Returns true if all sensors produced fresh data.
    bool pollAll(SensorData& out);

    /// Shutdown all sensors
    void shutdown();

    /// Access individual sensors if needed
    ImuSensor& imu() { return imu_; }
    LidarSensor& lidar() { return lidar_; }
    Odometry& odometry() { return odometry_; }

private:
    ImuSensor imu_;
    LidarSensor lidar_;
    Odometry odometry_;
    bool initialized_{false};
};

}  // namespace sensor_interface
}  // namespace SLAMaleykoum
