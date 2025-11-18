#include "test_common_esp2.h"

Esp_link esp_link(ESPS);

const Pose2D pos_eps1 = {5, 5, 5, 0};
const Pose2D pos_eps2 = {2, 2, 2, 2};

const GlobalPathMessage path_esp1 = {
    .path = {
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {1.0f, 1.0f},
        {2.0f, 1.0f}
    },
    .current_length = 0,
    .path_id = 0,
    .timestamp_ms = 0
};
const GlobalPathMessage path_esp2 = {
    .path = {
        { 5.0f,  5.0f},
        { 6.0f,  5.0f},
        { 7.0f,  6.0f},
        { 7.5f, 7.2f},
        { 8.0f, 8.0f}
    },
    .current_length = 0,
    .path_id = 0,            // identifiant différent pour ESP2
    .timestamp_ms = 0     // timestamp de test
};

const char* mqtt_topic_esps2 = "slamaleykoum77/esps";
const char* mqtt_topic_esps2_pos = "slamaleykoum77/pos2";
const char* mqtt_topic_esps2_gpm = "slamaleykoum77/gpm2";

void mqtt_print(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2, msg);
}
void mqtt_print_pos(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2_pos, msg);
}

void mqtt_print_gpm(const char* str) {
    Serial.println(str);

    char msg[80];
    snprintf(msg, sizeof(msg), str);
    connection.publish(mqtt_topic_esps2_gpm, msg);
}

const char* pose_to_cstr(const Pose2D& p) {
    static char buf[64];   // buffer persistant
    snprintf(buf, sizeof(buf),
             "x=%.3f, y=%.3f, th=%.3f, t=%u",
             p.x, p.y, p.theta, p.timestamp_ms);
    return buf;
}

const char* gpm_to_cstr(const GlobalPathMessage& gpm) {
    static char buf[512];   // sufﬁsant pour ~20–30 waypoints
    size_t pos = 0;

    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "PathID=%u, time=%u, len=%u\n",
                    gpm.path_id, gpm.timestamp_ms, gpm.current_length);

    for (uint16_t i = 0; i < gpm.current_length && pos < sizeof(buf); i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "  [%u] (%.3f, %.3f)\n",
                        i,
                        gpm.path[i].x,
                        gpm.path[i].y);
    }

    return buf;
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

void test_pos();
void test_path();

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

    test_path();

    delay(2000);
}

void test_pos() {
    Pose2D out = {};
    Pose2D to_compare = pos_eps1;

    if (esp_link.get_pos(out)) {
        mqtt_print("pos recevied on esp2");
        mqtt_print_pos(pose_to_cstr(out));
        if (out == to_compare) {
            esp_link.sendPos(pos_eps2);
            mqtt_print("pos is exact on esp2");
        }
    }
}

void test_path() {
    GlobalPathMessage out;
    GlobalPathMessage to_compare = path_esp1;

    if (esp_link.get_path(out)) {
        mqtt_print("path recevied on esp2");
        mqtt_print_gpm(gpm_to_cstr(out));
        if (out == to_compare) {
            esp_link.sendPath(path_esp2);
            mqtt_print("path is exact on esp2");
        }
    }
}