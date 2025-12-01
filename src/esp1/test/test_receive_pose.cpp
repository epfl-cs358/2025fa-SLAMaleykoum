/**
 * @file main_esp1_pose_receiver_test.cpp
 * @brief ESP1 - Sends paths to ESP2 and receives pose updates back
 * 
 * This test:
 * 1. Sends Path A on startup
 * 2. Continuously receives and displays pose updates from ESP2
 * 3. After 20 seconds, sends Path B (optional dynamic update)
 */

#include <Arduino.h>
#include "common/esp_link.h"
#include "common/data_types.h"
#include "common/wifi_connection.h"

const char* mqtt_topic_connection_receive_pos_setup = "slamaleykoum77/setupesp1";
const char* mqtt_topic_connection_receive_pos = "slamaleykoum77/sendwaypoints";


// UART link (TX=12, RX=13, 2Mbaud)
Esp_link esp___link(Serial1);

char bufrecpos[256];

Connection connection;
// ------------------------------------------
// PATH A (the original one)
// ------------------------------------------
float pathA__X[] = {
    0.00, 0.00, 0.50, 1.00, 1.50, 2.00,
    2.50, 3.00, 3.50, 4.00, 4.00, 4.00,
    4.00, 4.00, 4.00, 3.50, 3.00, 2.50,
    2.00, 1.50, 1.00, 0.50, 0.00, -0.20
};
float pathA__Y[] = {
    0.00, 0.20, 0.20, 0.20, 0.20, 0.20,
    0.20, 0.20, 0.20, 0.20, 0.50, 1.00,
    1.50, 2.00, 2.40, 2.40, 2.40, 2.40,
    2.40, 2.40, 2.40, 2.40, 2.40, 2.40
};
const int pathA_size = sizeof(pathA__X) / sizeof(pathA__X[0]);


// Timing variables
uint32_t pathSentTime = 0;
bool pathBSent = false;

// Pose statistics
uint32_t poseCount = 0;
uint32_t lastPoseTime = 0;

// ===============================================================
// SETUP
// ===============================================================
void setup_receive_pos() {
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();

    connection.check_connection();

    snprintf(bufrecpos, sizeof(bufrecpos), "ESP1 Bidirectional Communication Test: Sends Path → Receives Pose ");
    connection.publish( mqtt_topic_connection_receive_pos_setup, bufrecpos);

    esp___link.begin();

    snprintf(bufrecpos, sizeof(bufrecpos), "UART initialized at 2Mbaud");
    connection.publish( mqtt_topic_connection_receive_pos_setup, bufrecpos);


    // --- Send PATH A ---
    GlobalPathMessage pathA;
    pathA.current_length = pathA_size;
    pathA.path_id = 1;
    pathA.timestamp_ms = millis();

    for (int i = 0; i < pathA_size; i++) {
        pathA.path[i].x = pathA__X[i];
        pathA.path[i].y = pathA__Y[i];
    }
    
    snprintf(bufrecpos, sizeof(bufrecpos), "Sending Path A (initial path)...");
    connection.publish( mqtt_topic_connection_receive_pos_setup, bufrecpos);

    esp___link.sendPath(pathA);

    
    snprintf(bufrecpos, sizeof(bufrecpos), "Path A sent! [ID=%lu, Length=%u waypoints]\n\n", 
                  pathA.path_id, pathA.current_length);
    connection.publish( mqtt_topic_connection_receive_pos_setup, bufrecpos);


    pathSentTime = millis();
    lastPoseTime = millis();

    snprintf(bufrecpos, sizeof(bufrecpos), "Listening for pose updates from ESP2...\n");
    connection.publish( mqtt_topic_connection_receive_pos_setup, bufrecpos);

}

// ===============================================================
// LOOP
// ===============================================================
void loop_receive_pos() {
    // Poll for incoming messages
    esp___link.poll();

    // Check for pose updates
    Pose2D pose;
    if (esp___link.get_pos(pose)) {
        poseCount++;
        lastPoseTime = millis();

        // Display pose with formatting
        snprintf(bufrecpos, sizeof(bufrecpos), "Pose #%lu | X=% 7.3f | Y=% 7.3f | θ=% 6.3f rad | t=%lu ms\n",
                      poseCount, pose.x, pose.y, pose.theta, pose.timestamp_ms);
        connection.publish( mqtt_topic_connection_receive_pos, bufrecpos);

    }

    delay(10); 
}