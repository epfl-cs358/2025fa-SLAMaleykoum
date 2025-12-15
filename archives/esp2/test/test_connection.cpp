/**
 * @file test_connection.cpp
 * @brief Test script for verifying Wi-Fi and MQTT connectivity on the ESP32.
 *
 * This file provides a simple test routine to validate that the ESP32 
 * can connect to the configured Wi-Fi network and communicate with 
 * the MQTT broker defined in `wifi_connection.h`.
 *
 * @details
 * - The `setup_test_connection()` function initializes serial communication 
 *   and connects the board to Wi-Fi and the MQTT server.
 * - The `loop_test_connection()` function periodically checks the MQTT 
 *   connection and publishes a test message (`"I am connected!!!!"`) 
 *   every second.
 *
 * @note 
 * This test is blocking and runs continuously; it should only be used 
 * for debugging and connectivity validation, not in the main production firmware.
 *
 * Typical output on Serial Monitor:
 * @code
 * Connecting to PC-Alex
 * -----
 * Connected to PC-Alex
 * Published: I am connected!!!!
 * Published: I am connected!!!!
 * ...
 * @endcode
 *
 * @see wifi_connection.h
 */
#include "test_common_esp2.h"

const char* mqtt_topic_connection2 = "slamaleykoum77/print";

void setup_test_connection() {
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();
}

void loop_test_connection() {
    connection.check_connection();

    char msg[50];
    snprintf(msg, sizeof(msg), "I am connected!!!!");
    // Publish data to MQTT
    connection.publish(mqtt_topic_connection2, msg);

    delay(1000);
}
