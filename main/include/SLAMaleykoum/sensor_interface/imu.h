/**
 * @file imu.h
 * @brief Hardware abstraction for the IMU sensor.
 */

#pragma once
#include "sensor_data_types.h"

namespace SLAMaleykoum {
namespace sensor_interface {

class ImuSensor {
public:
    ImuSensor() = default;

    bool initialize();       ///< Initialize I2C/SPI and configure IMU.
    void shutdown();         ///< Graceful shutdown.
    bool isInitialized() const { return initialized_; }

    bool poll(ImuData& out); ///< Read new IMU data (accel + gyro).

    const ImuData& latest() const { return last_sample_; }

    void calibrate(const std::array<float, 3>& accel_bias,
                   const std::array<float, 3>& gyro_bias);

private:
    bool initialized_{false};
    ImuData last_sample_;
    std::array<float, 3> accel_bias_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> gyro_bias_{0.0f, 0.0f, 0.0f};
};

}  // namespace sensor_interface
}  // namespace SLAMaleykoum
