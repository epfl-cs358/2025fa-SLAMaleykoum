#include "test_common_esp1.h"

Connection connection;

HardwareSerial ESPS_LIDAR(1);
int RX_ESPS = 13;
int TX_ESPS = 12;
uint32_t ESPS_BAUDRATE = 2000000;

Esp_link esp_link(ESPS_LIDAR, RX_ESPS, TX_ESPS);

const char* mqtt_topic_esps1 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps1, msg);
}

void setup_esps_comm_esp1() {
    connection.setupWifi();
    connection.check_connection();

    mqtt_print("[ESP1] Setup ok");

    esp_link.begin(ESPS_BAUDRATE);
    delay(1000);

    mqtt_print("ESP1 ready");
}

void loop_esps_comm_esp1() {
    esp_link.sendText("Ping from ESP1");
    
    esp_link.poll();

    char* out;
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        mqtt_print("out = ");
        mqtt_print(out);
        if (out == "Pong from EPS2"); 
            mqtt_print("[ESP1] got 'PONG' ");
    }

    delay(2000);
}