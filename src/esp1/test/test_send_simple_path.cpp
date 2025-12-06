/**
 * @file test_send_simple_path.cpp
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

//Connection connection;

const char* mqtt_topic_connection_esp1_comm_setup = "slamaleykoum77/setupesp1";
const char* mqtt_topic_connection_esp1_comm_sendwaypoints = "slamaleykoum77/sendwaypoints";

char buffer[100];


// ===============================================================
// Hardware Configuration
// ===============================================================
Esp_link esp_link_simple(Serial1);  // UART on Serial1 (TX=12, RX=13)

// ===============================================================
// Path Definition - Same as your ESP2 test path
// ===============================================================
float pathX[] = {
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 4.00, 4.00, 4.00, 4.00, 4.00, 3.50, 3.00, 2.50, 2.00, 1.50, 1.00, 0.50, 0.00, -0.20
};
float pathY[] ={
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.50, 1.00, 1.50, 2.00, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40, 2.40
};

const int pathSize = sizeof(pathX) / sizeof(pathX[0]);

// ===============================================================
// SETUP
// ===============================================================
void setup_send_simple_path() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(2000);

    // Wifi connection 
    //connection.setupWifi();
    //connection.check_connection();

    snprintf(buffer, sizeof(buffer), "ESP1: Path Sender (Minimal Test)");
    //connection.publish(mqtt_topic_connection_esp1_comm_setup, buffer);
    

    // Initialize UART link to ESP2
    esp_link_simple.begin();
    snprintf(buffer, sizeof(buffer), "UART initialized (TX=12, RX=13, 2Mbaud)");
    //connection.publish(mqtt_topic_connection_esp1_comm_setup, buffer);
    
    // Print path info
    snprintf(buffer, sizeof(buffer), "Path to send: %d waypoints\n", pathSize);
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);
    snprintf(buffer, sizeof(buffer), "First: (%.2f, %.2f)\n", pathX[0], pathY[0]);
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);
    snprintf(buffer, sizeof(buffer), "Last:  (%.2f, %.2f)\n", pathX[pathSize-1], pathY[pathSize-1]);
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);


    
    // Wait 5 seconds for ESP2 to boot and initialize
    snprintf(buffer, sizeof(buffer), "\n Waiting 5 seconds for ESP2 to be ready...");
    //connection.publish(mqtt_topic_connection_esp1_comm_setup, buffer);
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
    snprintf(buffer, sizeof(buffer), "Sending path to ESP2...");
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);
    esp_link_simple.sendPath(pathMsg);
    snprintf(buffer, sizeof(buffer), "Path sent!");
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);

    snprintf(buffer, sizeof(buffer), "\n Sent %d waypoints with path_id=%lu\n", pathMsg.current_length, pathMsg.path_id);
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);

    snprintf(buffer, sizeof(buffer), "\nESP1: Done! ESP2 should now follow path.");
    //connection.publish(mqtt_topic_connection_esp1_comm_sendwaypoints, buffer);

}

// ===============================================================
// LOOP
// ===============================================================
void loop_send_simple_path() {
    // Nothing to do - path was sent in setup()
    //delay(1000);
}