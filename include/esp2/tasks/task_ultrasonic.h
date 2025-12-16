#pragma once
#include <Arduino.h>

/**
 * @brief FreeRTOS task monitoring ultrasonic distance for safety.
 *
 * This task periodically reads the ultrasonic sensor to detect
 * obstacles in front of the vehicle.
 *
 * When an obstacle is detected within a configurable emergency
 * distance, the task triggers a global emergency stop unless
 * a recovery maneuver is currently being executed.
 *
 * @param pvParameters Pointer to task parameters (unused)
 */

void TaskUltrasonic(void *pvParameters);
