#include "test_common_esp2.h"

Esp_link esp_link(ESPS);

const char* mqtt_topic_esps2 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2, msg);
}

void setup_esps_comm_esp2() {
    Serial.begin(115200);
    delay(1000);

    connection.setupWifi();
    connection.check_connection();
    delay(500);

    mqtt_print("[ESP2] Setup ok");

    esp_link.begin();
    delay(1000);

    mqtt_print("ESP2 ready");
}

void loop_esps_comm_esp2() {
    mqtt_print("esp2 dans la boucle");
    delay(1000);

    esp_link.poll();
    
    char out[MAX_TXT_LEN];
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        mqtt_print("out = ");
        mqtt_print(out);
        if (out == "Ping from EPS1"); 
            esp_link.sendText("Pong from EPS2");
    }

    delay(2000);
}