#pragma once
#include <Arduino.h>

/**
 * @brief FreeRTOS task providing stable unified odometry estimation.
 *
 * This task computes the robot pose by fusing:
 * - IMU yaw (quaternion-based)
 * - Wheel encoder linear velocity
 *
 * It includes basic glitch rejection on yaw changes to improve
 * robustness against IMU spikes and sensor noise.
 *
 * The resulting pose is:
 * - Integrated over time
 * - Stored in shared global state (protected by a mutex)
 * - Periodically transmitted to the other ESP via esp_link
 *
 * @param pvParameters Pointer to task parameters (unused)
 */

void TaskOdometryUnified_Stable(void *pvParameters);
