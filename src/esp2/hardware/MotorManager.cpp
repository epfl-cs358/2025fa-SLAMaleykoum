/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 * 
 * @author SLAMaleykoum 
 * @date Oct 2025
 */
#include "MotorManager.h"

MotorManager::MotorManager(int pwmPin) : motor(pwmPin) {}

/**
 * @brief Initializes the motor
 */
bool MotorManager::begin() {
    return motor.begin();
}

/**
 * @brief Moves the motor forward
 */
void MotorManager::forward(float percent) {
    motor.setTargetPercent(percent);
}

/**
 * @brief Moves the motor backwards
 */
void MotorManager::backward(float percent) {
    motor.setTargetPercent(-percent);
}

/**
 * @brief Stops the motor
 */
void MotorManager::stop() {
    motor.setTargetPercent(0.0f);
}

/**
 * @brief Stops the motor when an obstacle is detected
 */
void MotorManager::emergencyStop() {
    motor.emergencyStop();
}

/**
 * @brief Gradually ramp motor to target speed
*/
void MotorManager::update() {
    motor.update();
}
