/**
 * @file MotorManager.h 
 * @brief High-level interface for controlling the DC motor through the Tamiya brushed ESC.
 * 
 * @author SLAMaleykoum 
 * @date Oct 2025
 */
#pragma once
#include "MotorController.h"

/**
 * @class MotorManager
 * @brief High-level methods to control the car's motor
*/

class MotorManager {
public:
    /**
     * @brief Construct a MotorManager object
     * @param escPin pin connected to the ESC signal input
     */

    MotorManager(int pwmPin);

    /**
     * @brief Initializes the ESC
     * @return true if the esc is connected and initializes, otherwise false if disconnected
     */
     bool begin();

    /**
     * @brief Moves the motor forward, given the amount of power in percent
     * @param percent Power level between 0,0f and 1,0f
     */
    void forward(float percent = 1.0f);

    /**
     * @brief Moves the motor backwards, given the amount of power in percent
     * @param percent Power level between 0,0f and 1,0f
     */
    void backward(float percent = 1.0f);

    /**
     * @brief Stops the motor
     */
    void stop();

    /**
     * @brief Stops the motor when obstacle is detected by the Ultra Sonic Sensor
     */
    void emergencyStop();

    /**
     * @brief Updates the internal control loop of the motor
     */
    void update();

private:
    MotorController motor; //motor control instance
};
