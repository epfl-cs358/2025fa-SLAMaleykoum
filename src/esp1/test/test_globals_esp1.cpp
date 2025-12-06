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

HardwareSerial ESPS(1);
Connection connection("esp1");

HardwareSerial& LIDAR_SERIAL = Serial2;
Lidar lidar(LIDAR_SERIAL);
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
LiDARScan scan;
bool scanComplete = false;
uint16_t lastSendTime = 0;
