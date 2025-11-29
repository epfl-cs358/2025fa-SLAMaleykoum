/**
 * @file main_esp1.cpp
 * @brief Minimal path sender for ESP2 testing
 *
 * This code ONLY sends a path to ESP2 via UART.
 * No WiFi, no MQTT, no position receiving - just path transmission.
 *
 * @author SLAMaleykoum
 * @date November 2025
 */

#include <Arduino.h>
#include "common/esp_link.h"
#include "common/data_types.h"

#include "common/data_types.h"
#include "common/wifi_connection.h"

Connection connection;

const char* mqtt_topic_connection_esp1_comm_setup = "slamaleykoum77/setupesp1";
const char* mqtt_topic_connection_esp1_comm_sendwaypoints = "slamaleykoum77/sendwaypoints";

char buf[100];


// ===============================================================
// Hardware Configuration
// ===============================================================
Esp_link esp_link(Serial1);  // UART on Serial1 (TX=12, RX=13)

// ===============================================================
// Path Definition - Same as your ESP2 test path
// ===============================================================
float pathX[] = {0,0}; /*{
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 4.00, 4.00, 4.00, 4.00, 4.00, 3.50, 3.00, 2.50, 2.00, 1.50, 1.00, 0.50, 0.00, -0.20
};*/
float pathY[] ={0,1.0};/* {
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.50, 1.00, 1.50, 2.00, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40
};*/

const int pathSize = sizeof(pathX) / sizeof(pathX[0]);

// ===============================================================
// SETUP
// ===============================================================
void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(2000);

    // Wifi connection 
    connection.setupWifi();
    connection.check_connection();

    snprintf(buf, sizeof(buf), "ESP1: Path Sender (Minimal Test)");
    connection.publish(mqtt_topic_connection_esp1_comm_setup, buf);
    

    // Initialize UART link to ESP2
    esp_link.begin();
    snprintf(buf, sizeof(buf), "UART initialized (TX=12, RX=13, 2Mbaud)");
    connection.publish(mqtt_topic_connection_esp1_comm_setup, buf);
    
    // Print path info
    snprintf(buf, sizeof(buf), "Path to send: %d waypoints\n", pathSize);
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);
    snprintf(buf, sizeof(buf), "First: (%.2f, %.2f)\n", pathX[0], pathY[0]);
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);
    snprintf(buf, sizeof(buf), "Last:  (%.2f, %.2f)\n", pathX[pathSize-1], pathY[pathSize-1]);
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);


    
    // Wait 5 seconds for ESP2 to boot and initialize
    snprintf(buf, sizeof(buf), "\n Waiting 5 seconds for ESP2 to be ready...");
    connection.publish(mqtt_topic_connection_esp1_comm_setup, buf);
    delay(5000);
    
    // Create path message
    GlobalPathMessage pathMsg;
    pathMsg.current_length = pathSize;
    pathMsg.path_id = 1;
    pathMsg.timestamp_ms = millis();
    
    // Copy waypoints
    for (int i = 0; i < pathSize; i++) {
        pathMsg.path[i].x = pathX[i];
        pathMsg.path[i].y = pathY[i];
    }
    
    // Send path to ESP2
    snprintf(buf, sizeof(buf), "Sending path to ESP2...");
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);
    esp_link.sendPath(pathMsg);
    snprintf(buf, sizeof(buf), "Path sent!");
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);

    snprintf(buf, sizeof(buf), "\n Sent %d waypoints with path_id=%lu\n", pathMsg.current_length, pathMsg.path_id);
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);

    snprintf(buf, sizeof(buf), "\nESP1: Done! ESP2 should now follow path.");
    connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buf);

}

// ===============================================================
// LOOP
// ===============================================================
void loop() {
    // Nothing to do - path was sent in setup()
    //delay(1000);
}