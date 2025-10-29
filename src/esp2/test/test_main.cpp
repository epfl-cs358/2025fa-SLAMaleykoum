/**
 * @file test_main.cpp
 * @brief Main program for testing the code
 * 
 * @author SLAMaleykoum
 * @date October 2025
 */
#include "test_common.h"

// Change the id depending on the test we want to run
#ifndef TEST_ID
#define TEST_ID 5
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
    #endif
}
