#include "test_common_esp2.h"

Esp_link esp_link(ESPS);

const char* mqtt_topic_esps2 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2, msg);
}

inline bool operator==(const Pose2D& a, const Pose2D& b) {
    return
        a.x == b.x &&
        a.y == b.y &&
        a.theta == b.theta &&
        a.timestamp_ms == b.timestamp_ms;
}

void test_text();
void test_pos();

void setup_esps_comm_esp2() {
    delay(2000);
    Serial.begin(115200);
    delay(1000);

    connection.setupWifi();
    connection.check_connection();
    delay(500);

    esp_link.begin();
    delay(1000);

    mqtt_print("ESP2 ready");
}

void loop_esps_comm_esp2() {
    connection.check_connection();
    delay(1000);

    esp_link.poll();

    test_pos();

    delay(2000);
}

void test_text() {
    char out[MAX_TXT_LEN];
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        // mqtt_print("out len (censé etre 14) = ");
        // char len_str[12];
        // snprintf(len_str, sizeof(len_str), "%u", (unsigned)strlen(out));
        // mqtt_print(len_str);
        mqtt_print("out = ");
        mqtt_print(out);
        if (strcmp(out, "Ping from ESP1") == 0) {
            esp_link.sendText("Pong from EPS2");
        }
    }
}

void test_pos() {
    Pose2D out = {};
    Pose2D to_compare = {5, 5, 5, 0};
    if (esp_link.get_pos(out)) {
        mqtt_print("pos recevied on esp2");
        if (out == to_compare) {
            esp_link.sendPos({2, 2, 2, 2});
            mqtt_print("pos is exact on esp2 and sending the (2,2,2,2) to esp1");
        }
    }
}