#include "test_common_esp2.h"

Connection connection;

HardwareSerial ESPS_LIDAR(1);
int RX_ESPS = 13;
int TX_ESPS = 12;
uint32_t ESPS_BAUDRATE = 2000000;

Esp_link esp_link(ESPS_LIDAR, RX_ESPS, TX_ESPS);

const char* mqtt_topic_esps2 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2, msg);
}

void setup_esps_comm_esp2() {
    connection.setupWifi();
    connection.check_connection();

    mqtt_print("[ESP2] Setup ok");

    esp_link.begin(ESPS_BAUDRATE);
    delay(1000);

    mqtt_print("ESP2 ready");
}

void loop_esps_comm_esp2() {
    esp_link.poll();
    
    char* out;
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        mqtt_print("out = ");
        mqtt_print(out);
        if (out == "Ping from EPS1"); 
            esp_link.sendText("Pong from EPS2");
    }

    delay(2000);
}