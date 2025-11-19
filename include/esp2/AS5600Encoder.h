/**
 * @file AS5600Encoder.h
 * @brief Header file defining the AS5600Encoder class to be used to measure the absolute position
 * of the motor rotation.
 * 
 * @author SLAMaleykoum & TurboSLAM : Same file as the TurboSLAM but removed all code related to ROS messages
 * @date Oct 2025
 */

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

/**
 * @class AS5600Encoder
 * @brief reads motor angle
 */
class AS5600Encoder {
private:
    int _sdaPin;
    int _sclPin;
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t RAW_ANGLE_REG = 0x0C;

    /**
     * @brief Get the actual angle
     * @return the angle if available. 0xFFFF if bus occupied or miscommunication.
     */
    uint16_t getRawAngle();

public:
    /**
     * @brief Construct an AS5600Encoder object
     */
    AS5600Encoder(int sdaPin, int sclPin);

    /**
     * @brief Initialize I2C and test the sensor
     * @return true if the initialization succeeded. False if I2C not found or angle incorrect.
     */
    bool begin();

    /**
     * @brief gets the new angle.
     * @return the angle in degrees.
     */
    float update();
};

#endif
