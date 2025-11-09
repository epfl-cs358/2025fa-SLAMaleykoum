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
#include "test_common_esp1.h"

//const HardwareSerial LIDAR(2);
rpLidar* lidar = new rpLidar(&Serial2, LIDAR_BAUDRATE, 16, 17);
DMS15 servo_dir(SERVO_DIR_PIN);

Connection connection;
