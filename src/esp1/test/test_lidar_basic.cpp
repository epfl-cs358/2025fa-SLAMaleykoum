/**
 * @file test_lidar_basic.cpp
 * @brief Test to send fake data of the lidar to the MQTT broker
 * @result Create a fake map with the mqtt_client.py file
 */
#include "test_common_esp1.h"

const char* mqtt_topic_lidar_basic = "slamaleykoum77/lidar";

void setup_test_lidar_basic() {
    
    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();
}

// test loop
void loop_test_lidar_basic() {
    connection.check_connection();

    int num_points = 10;
    float distances[5] = {2.0, 2.5, 3.0, 5.0, 4.0};
    int quality[5] = {1, 4, 6, 3, 8};

    char msg[6000];
    strcpy(msg, "{\"type\": \"lidar\", \"points\": [");  // start JSON array

    for (int i = 0; i < num_points; i++) {
        float angle_deg = i * 3.6;
        char point[50];
        snprintf(point, sizeof(point), "{\"angle\": %.0f, \"distance\": %.2f, \"quality\": %u}", angle_deg, distances[i % 5], quality[i % 5]);
        strcat(msg, point);

        if (i < num_points - 1) strcat(msg, ",");  // add comma except after last element
    }

    strcat(msg, "]");  // close JSON array

    connection.publish(mqtt_topic_lidar_basic, msg);

    delay(1000);
}
