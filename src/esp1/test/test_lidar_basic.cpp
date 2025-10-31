/**
 * @file test_lidar_basic.cpp
 * @brief Test to send fake data of the lidar to the MQTT broker
 * @result Create a fake map with the mqtt_client.py file
 */
#include "test_common_esp1.h"

void setup_test_lidar_basic() {
    
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();
}

// test loop
void loop_test_lidar_basic() {
    connection.check_connection();

    int num_points = 100;
    float distances[5] = {2.0, 2.5, 3.0, 5.0, 4.0};

    char msg[500];
    strcpy(msg, "[");  // start JSON array

    for (int i = 0; i < num_points; i++) {
        float angle_deg = i * 3.6;
        char point[50];
        snprintf(point, sizeof(point), "{\"angle\": %d, \"distance\": %.2f}", angle_deg, distances[i % 5]);
        strcat(msg, point);

        if (i < num_points - 1) strcat(msg, ",");  // add comma except after last element
    }

    strcat(msg, "]");  // close JSON array

    connection.publish(msg);

    delay(100000);
}
