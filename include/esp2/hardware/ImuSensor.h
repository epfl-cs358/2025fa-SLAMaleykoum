/**
 * @file ImuSensor.h
 * @brief Header file defining the ImuSensor class to be used to measure the movement of the car
 * 
 * @author SLAMaleykoum & TurboSLAM : Same file as the TurboSLAM but removed all code related to ROS messages
 * @date Oct 2025
 */

#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Adafruit_BNO08x.h>
#include <stdint.h>
#include "../common/data_types.h"

/**
 * @class ImuSensor
 * @brief reads IMU data and gives a compact IMUData view
 */
class ImuSensor {
private:
    Adafruit_BNO08x bno086; // handles all the communication with the sensor (I2C addr, SH2 protocol..)
    sh2_SensorValue_t sensorValue; // temp structure to keep the current mesure
    const uint8_t IMU_ADDR = 0x4B;
  

    /**
     * @brief Enable desired BNO08x reports at a given period.
     * @param period_us Reporting period in microseconds
     * @return true if all requested reports enabled. False if at least one report failed to enable.
     */
    bool configureSensor(uint32_t period_us = 400); // 400 Hz by default

public:
    IMUData imu_data;
    /**
     * @brief Construct an ImuSensor object
     * Calls the constructor of bno086 and nitializes the sensorValue and all the data components to 0
     */
    ImuSensor();

    /**
     * @brief Initialize I2C and the BNO08x, then enable sensor reports.
     * @return true if the initialization succeeded. False if I2C not found or BNO08x not responding.
     */
    bool begin();

    /**
     * @brief Poll and process all pending BNO08x events, updating the internal IMUData.
     */
    void readAndUpdate();

    /**
     * @brief Read-only access to the latest IMU measurements.
     * @return const IMUData& Reference to the last updated data.
     */
    const IMUData& data() {return imu_data;}
};

#endif