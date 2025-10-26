/**
 * @file test_globals.cpp
 * @brief The global variables used in all the tests
 */
#include "test_common.h"

MotorManager motor(ESC_PIN);
DMS15 servo_dir(SERVO_DIR_PIN);
UltraSonicSensor ultrasonic(US_TRIG_PIN, US_ECHO_PIN);
ImuSensor imu(SDA_PIN, SCL_PIN);
AS5600Encoder encoder(SDA_PIN, SCL_PIN);