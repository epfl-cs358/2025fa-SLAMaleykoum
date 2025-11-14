#include "test_common_esp1.h"

Esp_link esp_link(ESPS);

const char* mqtt_topic_esps1 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps1, msg);
}

void setup_esps_comm_esp1() {
    delay(3000);
    Serial.begin(115200);
    delay(1000);

    connection.setupWifi();
    connection.check_connection();
    delay(500);

    mqtt_print("[ESP1] Setup ok");

    esp_link.begin();
    Serial.println("juste aprs begin");
    delay(1000);

    mqtt_print("ESP1 ready");
}

void loop_esps_comm_esp1() {
    connection.check_connection();

    mqtt_print("esp1 dans la boucle");
    delay(1000);

    esp_link.sendText("Ping from ESP1");
    
    esp_link.poll();

    char out[MAX_TXT_LEN];
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        mqtt_print("out = ");
        mqtt_print(out);
        if (strcmp(out, "Pong from EPS2") == 0) {
            mqtt_print("[ESP1] got 'PONG' ");
        }
    }

    delay(2000);
}