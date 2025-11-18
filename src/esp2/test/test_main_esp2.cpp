/**
 * @file test_main.cpp
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
#include "test_common_esp2.h"

// Change the id depending on the test to run
#ifndef TEST_ID
#define TEST_ID 11
#endif

void setup() {
    #if TEST_ID == 1
        setup_servo_right_left();
    #elif TEST_ID == 2
        setup_motor_start_stop();
    #elif TEST_ID == 3
        setup_imu_stop_z();
    #elif TEST_ID == 4
        setup_all_together();
    #elif TEST_ID == 5
        setup_motor_runs();
    #elif TEST_ID == 6
        setup_test_connection();
    #elif TEST_ID == 7
        setup_imu_calibration();
    #elif TEST_ID == 8
        setup_encoder();
    #elif TEST_ID == 9
        setup_pid_velocity();   
    #elif TEST_ID == 10
        setup_path_pid();
    #elif TEST_ID == 11
        setup_esps_comm_esp2();
    #endif
}

void loop() {
    #if TEST_ID == 1
        loop_servo_right_left();
    #elif TEST_ID == 2
        loop_motor_start_stop();
    #elif TEST_ID == 3
        loop_imu_stop_z();
    #elif TEST_ID == 4
        loop_all_together();
    #elif TEST_ID == 5
        loop_motor_runs();
    #elif TEST_ID == 6
        loop_test_connection();
    #elif TEST_ID == 7
        loop_imu_calibration();
    #elif TEST_ID == 8
        loop_encoder();
    #elif TEST_ID == 9
        loop_pid_velocity();
    #elif TEST_ID == 10
        loop_path_pid();
    #elif TEST_ID == 11
        loop_esps_comm_esp2();
    #endif
}
