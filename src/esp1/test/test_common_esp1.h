/**
 * @file test_common_esp1.h
 * @brief Declares global hardware objects, pin mappings, and shared test function prototypes.
 *
 * This header centralizes all common definitions, hardware object declarations, 
 * and test function prototypes used across the various test modules.
 *
 * @details
 * - Defines all pin mappings for the ESP32-S3 1 board used in the project.  
 * - Declares all global hardware interfaces (lidar).  
 * - Declares the global `Connection` object for Wi-Fi and MQTT communication.  
 * - Provides prototypes for each `setup_*()` and `loop_*()` function, 
 *   allowing `test_main_esp1.cpp` to call the corresponding tests dynamically.
 *
 * @note
 * The global objects are **defined** once in `test_globals.cpp`.  
 * This file only provides `extern` declarations to make them accessible 
 * across all test files.
 */
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_connection.h"

// Pins used by the hardware components on the esp 2
#define LIDAR_PIN 4 // TO CHANGE!!!

// Harwdare objects
//extern LiDAR lidar;

extern Connection connection;

// Prototypes of the functions
//void setup_all_together();          void loop_all_together();
void setup_test_lidar_basic();      void loop_test_lidar_basic();
void setup_test_connection();       void loop_test_connection();
