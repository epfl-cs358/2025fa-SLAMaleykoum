/**
 * @file test_main_esp1.cpp
 * @brief Central test runner for all hardware and communication modules of the SLAMaleykoum project.
 *
 * This file acts as the main test entry point for the ESP32 firmware.
 * It allows us to easily switch between different hardware or software
 * component tests (motor, servo, IMU, Wi-Fi/MQTT, etc.) by changing a single
 * compile-time constant (`TEST_ID`).
 *
 * @details
 * - Each test scenario has its own `setup_*()` and `loop_*()` functions 
 *   defined in separate files under the `/test/` directory.
 * - Only one test is executed at a time based on the selected `TEST_ID`.
 * - The standard Arduino `setup()` and `loop()` functions act as dispatchers.
 *
 * @note
 * Update the `TEST_ID` macro before building.
 *
 * @author SLAMaleykoum
 * @date October 2025
 */
#include "test_common_esp1.h"

// Change the id depending on the test to run
#ifndef TEST_ID
#define TEST_ID 3
#endif

void setup() {
    initGlobals();
    #if TEST_ID == 1
        setup_test_lidar_basic();
    #elif TEST_ID == 2
        setup_test_connection();
    #elif TEST_ID == 3
        setup_test_lidar_express();
    #elif TEST_ID == 4
        setup_test_lidar_serial();
    #elif TEST_ID == 5
        setup_test_read_lidar();
    #elif TEST_ID == 6
        setup_servo_lidar();
    #elif TEST_ID == 7
        setup_test_lidar_express_get_info();
    #elif TEST_ID == 8
        setup_test_lidar_standard();
    #endif
}

void loop() {
    #if TEST_ID == 1
        loop_test_lidar_basic();
    #elif TEST_ID == 2
        loop_test_connection();
    #elif TEST_ID == 3
        loop_test_lidar_express();
    #elif TEST_ID == 4
        loop_test_lidar_serial();
    #elif TEST_ID == 5
        loop_test_read_lidar();
    #elif TEST_ID == 6
        loop_servo_lidar();
    #elif TEST_ID == 7
        loop_test_lidar_express_get_info();
    #elif TEST_ID == 8
        loop_test_lidar_standard();
    #endif
}
