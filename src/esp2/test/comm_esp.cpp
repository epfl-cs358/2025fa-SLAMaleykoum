/**
 * @file main_esp2.cpp
 * @brief Minimal receiver test for ESP-to-ESP UART link
 *
 * ESP2 receives a GlobalPathMessage from ESP1 over UART
 * and prints its contents to Serial.
 *
 * No motors, no sensors, no WiFi — pure communication test.
 */

#include <Arduino.h>
#include "test_common_esp2.h"
#include "common/esp_link.h"
#include "common/data_types.h"

#include "common/wifi_connection.h"


const char* mqtt_topic_connection_esp2_comm_setup = "slamaleykoum77/setupesp2";
const char* mqtt_topic_connection_esp2_comm_receivewaypoints = "slamaleykoum77/receivewaypoints";

char bufreceive[100];

// ------------------------------------------------------------
// UART link (must match ESP1)
// ------------------------------------------------------------
Esp_link esp__link(Serial1);

// Storage for received path
PathMessage received__Path;

void setup_test_comm() {
    Serial.begin(115200);
    delay(1500);

    // Wifi connection 
    connection.setupWifi();
    connection.check_connection();

    snprintf(bufreceive, sizeof(bufreceive), "ESP2: PATH RECEIVER TEST");
    connection.publish(mqtt_topic_connection_esp2_comm_setup, bufreceive);

    snprintf(bufreceive, sizeof(bufreceive), "Initializing UART link...");
    connection.publish(mqtt_topic_connection_esp2_comm_setup, bufreceive);


    
    esp__link.begin();   // 2 Mbaud, correct pins, etc.


    snprintf(bufreceive, sizeof(bufreceive), "UART initialized.");
    connection.publish(mqtt_topic_connection_esp2_comm_setup, bufreceive);

     snprintf(bufreceive, sizeof(bufreceive), "Waiting for path from ESP1...\n");
    connection.publish(mqtt_topic_connection_esp2_comm_setup, bufreceive);
    
}

void loop_test_comm() {
    // Poll UART for incoming messages
    esp__link.poll();

    // Check if a new path was received
    if (esp__link.get_path(received__Path)) {

        snprintf(bufreceive, sizeof(bufreceive), "\n>>> RECEIVED PATH FROM ESP1!");
        connection.publish(mqtt_topic_connection_esp2_comm_receivewaypoints, bufreceive);

        snprintf(bufreceive, sizeof(bufreceive), "Path ID: %lu\n", received__Path.path_id);
        connection.publish(mqtt_topic_connection_esp2_comm_receivewaypoints, bufreceive);

        snprintf(bufreceive, sizeof(bufreceive), "Timestamp: %lu ms\n", received__Path.timestamp_ms);
        connection.publish(mqtt_topic_connection_esp2_comm_receivewaypoints, bufreceive);

        snprintf(bufreceive, sizeof(bufreceive), "Number of waypoints: %u\n\n", received__Path.current_length);
        connection.publish(mqtt_topic_connection_esp2_comm_receivewaypoints, bufreceive);


        

        for (int i = 0; i < received__Path.current_length; i++) {

            snprintf(bufreceive, sizeof(bufreceive), "WP %02d: (%.2f, %.2f)\n",
                          i,
                          received__Path.path[i].x,
                          received__Path.path[i].y);
        connection.publish(mqtt_topic_connection_esp2_comm_receivewaypoints, bufreceive);

        }
    }

    // For now, no other logic
    delay(10);
}
