#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "common/wifi_connection.h"

Connection connection;

const char* mqtt_topic_esp1esp2 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esp1esp2, msg);
}

void setup() {
    connection.setupWifi();
    connection.check_connection();

    mqtt_print("[ESP1] Setup ok");

    Serial1.begin(2000000, SERIAL_8N1, 13, 12); // RX=13, TX=12
    delay(1000);

    mqtt_print("ESP1 ready");
}

void loop() {
    Serial1.println("PING from ESP1");

    if (Serial1.available()) {
        String msg = Serial1.readStringUntil('\n');
        mqtt_print("[ESP1] got: ");
        mqtt_print(msg.c_str());
    }

    delay(1000);
}