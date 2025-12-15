/**
 * @file MotorManager.cpp
 * @brief Implementation of the MotorManager class
 * 
 * @author SLAMaleykoum 
 * @date Oct 2025
 */
#include "hardware/MotorManager.h"

MotorManager::MotorManager(int pwmPin) : motor(pwmPin) {}

bool MotorManager::begin() {return motor.begin();}

void MotorManager::forward(float percent) {motor.setTargetPercent(percent);}

void MotorManager::backward(float percent) {motor.setTargetPercent(-percent);}

void MotorManager::stop() {motor.setTargetPercent(0.0f);}

void MotorManager::emergencyStop() {motor.emergencyStop();}

void MotorManager::update() {motor.update();}
