/**
 * @file lidar.h
 * @brief Hardware abstraction for RPLiDAR-C1.
 */

#pragma once
#include "sensor_data_types.h"

namespace SLAMaleykoum {
namespace sensor_interface {

class LidarSensor {
public:
    LidarSensor() = default;

    bool initialize();          ///< Connect to LiDAR and start scanning.
    void shutdown();            ///< Stop scanning, close connection.
    bool isInitialized() const { return initialized_; }

    bool poll(LidarScan& out);  ///< Acquire a new scan frame (blocking or async).
    const LidarScan& latest() const { return last_scan_; }

private:
    bool initialized_{false};
    LidarScan last_scan_;
};

}  // namespace sensor_interface
}  // namespace SLAMaleykoum
