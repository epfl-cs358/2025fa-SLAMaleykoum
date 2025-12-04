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
#define TEST_ID 16
#endif

void setup() {
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
    #elif TEST_ID == 9
        setup_test_lidar_tcp();
    #elif TEST_ID == 10
        setup_esps_comm_esp1();
    #elif TEST_ID == 11
        setup_bayesian_dynamic_tcp();
    #elif TEST_ID == 12
        setup_clock_esp1();
    #elif TEST_ID == 13
        setup_led_basic();
    #elif TEST_ID == 14
        setup_clock_esp1_AP();
    #elif TEST_ID == 15
        setup_mission_planner_basic();
    #elif TEST_ID == 16
        setup_test_lidar_udp();
    #elif TEST_ID == 17
        setup_test_bayesian_udp();
    #elif TEST_ID == 18
        setup_send_simple_path();
    #elif TEST_ID == 19
        setup_send_dynamic_path();
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
    #elif TEST_ID == 9
        loop_test_lidar_tcp();
    #elif TEST_ID == 10
        loop_esps_comm_esp1();
    #elif TEST_ID == 11
        loop_bayesian_dynamic_tcp();
    #elif TEST_ID == 12
        loop_clock_esp1();
    #elif TEST_ID == 13
        loop_led_basic();
    #elif TEST_ID == 14
        loop_clock_esp1_AP();
    #elif TEST_ID == 15
        loop_mission_planner_basic();
    #elif TEST_ID == 16
        loop_test_lidar_udp();
    #elif TEST_ID == 17
        loop_test_bayesian_udp();
    #elif TEST_ID == 18
        loop_send_simple_path();
    #elif TEST_ID == 19
        loop_send_dynamic_path();
    #endif
}
