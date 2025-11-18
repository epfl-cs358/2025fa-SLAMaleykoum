#include "test_common_esp1.h"

Esp_link esp_link(ESPS);
const Pose2D pos_eps1 = {5, 5, 5, 0};

const char* mqtt_topic_esps1 = "slamaleykoum77/esps";

void mqtt_print(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps1, msg);
}

inline bool operator==(const Pose2D& a, const Pose2D& b) {
    return
        a.x == b.x &&
        a.y == b.y &&
        a.theta == b.theta &&
        a.timestamp_ms == b.timestamp_ms;
}

inline bool operator==(const Waypoint& a, const Waypoint& b) {
    return a.x == b.x && a.y == b.y;
}

inline bool operator==(const GlobalPathMessage& a, const GlobalPathMessage& b) {
    if (a.path_id        != b.path_id)        return false;
    if (a.timestamp_ms   != b.timestamp_ms)   return false;

    for (uint16_t i = 0; i < MAX_PATH_LENGTH; i++) {
        if (!(a.path[i] == b.path[i]))
            return false;
    }
    return true;
}

const char* pose_to_cstr(const Pose2D& p) {
    static char buf[64];   // buffer persistant
    snprintf(buf, sizeof(buf),
             "x=%.3f, y=%.3f, th=%.3f, t=%u",
             p.x, p.y, p.theta, p.timestamp_ms);
    return buf;
}

void test_text();
void test_pos();

void setup_esps_comm_esp1() {
    Serial.begin(115200);
    delay(1000);

    connection.setupWifi();
    connection.check_connection();
    delay(500);

    esp_link.begin();
    delay(1000);

    mqtt_print("ESP1 ready");
}

void loop_esps_comm_esp1() {
    connection.check_connection();
    delay(1000);

    esp_link.poll();

    test_pos();

    delay(2000);
}

void test_text() {
    esp_link.sendText("Ping from ESP1");
    delay(1000);

    char out[MAX_TXT_LEN];
    if (esp_link.get_txt(out)) {
        // to see what is happenning
        mqtt_print("out = ");
        mqtt_print(out);
        if (strcmp(out, "Pong from EPS2") == 0) {
            mqtt_print("[ESP1] got 'PONG' ");
        }
    }
}

void test_pos() {
    esp_link.sendPos(pos_eps1);
    delay(1000);

    Pose2D out = {};
    Pose2D to_compare = {2, 2, 2, 2};
    if (esp_link.get_pos(out)) {
        mqtt_print("pos recevied on esp1");
        mqtt_print(pose_to_cstr(out));
        if (out == to_compare) {
            mqtt_print("pos is exact on esp1");
        }
    }
}

void test_path() {
    esp_link.sendPath({});

    delay(1000);

    GlobalPathMessage out;
    GlobalPathMessage to_compare = {};
    if (esp_link.get_path(out)) {
        mqtt_print("path recevied on esp1");
        if (out == to_compare) {
            mqtt_print("path is exact on esp1");
        }
    }
}