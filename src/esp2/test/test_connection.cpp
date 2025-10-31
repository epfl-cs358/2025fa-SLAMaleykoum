#include "test_common.h"

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
    connection.publish(msg);

    delay(100);
}
