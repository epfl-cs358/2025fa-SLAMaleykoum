#include "test_common_esp2.h"

Connection connection;

HardwareSerial ESPS_LIDAR(1);
int RX_ESPS = 13;
int TX_ESPS = 12;
uint32_t ESPS_BAUDRATE = 2000000;

Esp_link link(ESPS_LIDAR, RX_ESPS, TX_ESPS);

const char* mqtt_topic_esp2esp1 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esp2esp1, msg);
}

void setup_esps_communication() {
    connection.setupWifi();
    connection.check_connection();

    mqtt_print("[ESP2] Setup ok");

    link.begin(ESPS_BAUDRATE);
    delay(1000);

    mqtt_print("ESP2 ready");
}

void loop_esps_communication() {
    link.poll();

}