#pragma once
#include <Arduino.h>

/**
 * @brief FreeRTOS task handling motor control and recovery maneuvers.
 *
 * This task is responsible for:
 * - Driving the motor during normal operation
 * - Monitoring global state flags (emergency stop, path completion)
 * - Triggering a recovery maneuver when an emergency stop occurs
 * - Stopping the motor and centering the steering when required
 *
 * The task runs continuously and synchronizes access to shared state
 * using a mutex.
 *
 * @param pvParameters Pointer to task parameters (unused)
 */

void TaskMotor(void *pvParameters);
