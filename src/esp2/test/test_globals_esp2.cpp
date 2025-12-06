/**
 * @file test_globals.cpp
 * @brief Defines global hardware and communication objects shared across all test modules.
 *
 * This file centralizes the instantiation of all hardware interface objects 
 * as well as the network connection manager.  
 * These global instances are declared in `test_common.h` and used across 
 * different test files to ensure consistent hardware initialization and 
 * shared access during experiments.
 *
 * @note
 * Only one instance of each hardware component exists globally.  
 * Do **not** redefine them elsewhere to avoid multiple-definition errors.
 */
#include "test_common_esp2.h"
/*
MotorManager motor(ESC_PIN);
//MotorController motor(ESC_PIN);
DMS15 servo_dir(SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(5,19);//US_TRIG_PIN, US_ECHO_PIN);
ImuSensor imu;
//AS5600Encoder encoder(SDA_PIN, SCL_PIN);
EncoderCarVelocity encoder;
Connection connection;
MotorPID pid(0.9f, 0.1f, 0.0f);
