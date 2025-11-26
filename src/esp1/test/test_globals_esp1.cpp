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

HardwareSerial LIDAR_SERIAL(2);
HardwareSerial ESPS(1);
Esp_link esp_link(ESPS);
Lidar* lidar = nullptr;

Connection connection("esp1");

void initGlobals() {
    // Initialize the material serial port of the LIDAR
    LIDAR_SERIAL.setRxBufferSize(LIDAR_SERIAL_BUFFER_SIZE);
    LIDAR_SERIAL.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    delay(100);

    // Create LIDAR object
    lidar = new Lidar(&LIDAR_SERIAL);
}
