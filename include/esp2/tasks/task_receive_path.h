#pragma once
#include <Arduino.h>

/**
 * @brief FreeRTOS task responsible for receiving path messages.
 *
 * This task continuously polls the communication link (esp_link)
 * to receive new path messages from the companion system.
 *
 * When a new path is received:
 * - The path is copied into shared global storage
 * - A flag is raised to notify other tasks of the update
 * - The first-path reception state is initialized
 *
 * Access to shared data is protected using mutexes.
 *
 * @param pvParameters Pointer to task parameters (unused)
 */

void TaskReceivePath(void *pvParameters);
