/**
 * @file test_lidar_basic.cpp
 * @brief Test to send fake data of the lidar to the MQTT broker (to test MQTT)
 * @result Create a fake map with the mqtt_client.py file
 */
#include "test_common_esp1.h"

const char* MQTT_TOPIC_LIDAR_BASIC = "slamaleykoum77/lidar";

// ---- Tunables ----
static const uint32_t PUBLISH_PERIOD_MS= 200;   // publish every 200 ms
static const uint16_t DOWNSAMPLE_N     = 8;     // keep every Nth point (bigger => fewer points)
static const uint16_t MAX_POINTS_OUT   = 180;   // hard cap to keep payload small

void setup_test_lidar_basic() {

    Serial.begin(115200);
    delay(2000);

    connection.setupWifi();

    lidar->resetDevice();
    stDeviceStatus_t sdst = lidar->getDeviceHealth();
    printf("sdst.errorCode_high=%d  sdst.errorCode_low=%d sdst.status=%d\r\n", sdst.errorCode_high, sdst.errorCode_low, sdst.status);

    lidar->setAngleOfInterest(LIDAR_ANGLE_OF_INTEREST_START, LIDAR_ANGLE_OF_INTEREST_END);

    connection.check_connection();

    bool ret = lidar->start(standard);
    if (ret) {
        char msg[6000];
        strcpy(msg, "🟢 Rplidar C1 started correctly!\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_BASIC, msg);
    } else {
        char msg[6000];
        strcpy(msg, "🔴 Error starting Rplidar C1\r\n");
        connection.publish(MQTT_TOPIC_LIDAR_BASIC, msg);
    }

    Serial.println("[LiDAR] STANDART mode started.");
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

    connection.publish(MQTT_TOPIC_LIDAR_BASIC, msg);

    delay(1000);
}
