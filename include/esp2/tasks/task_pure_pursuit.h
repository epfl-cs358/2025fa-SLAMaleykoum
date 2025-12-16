#pragma once
#include <Arduino.h>

/**
 * @brief FreeRTOS task implementing Pure Pursuit path tracking.
 *
 * This task:
 * - Waits for an initial path to be received
 * - Updates the Pure Pursuit controller when new paths arrive
 * - Computes steering (and velocity commands from the current pose only if you use pid
 *      look at archive)
 * - Stops command generation during emergency conditions
 * - Detects path completion and signals it to the global state
 *
 * The task operates asynchronously with odometry and motor tasks
 * and relies on mutex-protected shared state.
 *
 * @param pvParameters Pointer to task parameters (unused)
 */

void TaskPurePursuit(void *pvParameters);
