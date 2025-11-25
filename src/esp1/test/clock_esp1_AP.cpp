#include "test_common_esp1.h"

#define ABS(a, b)  ((a > b) ? (a - b) : (b - a))

int j = 0;
uint64_t t0_AP = 0;

void setup_clock_esp1_AP() {
    pinMode(LED_PIN, OUTPUT);

    esp_link.begin();
    delay(1000);
    
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP("TSF_AP", "12345678");
    delay(1000);

    digitalWrite(LED_PIN, HIGH);

    t0_AP = esp_wifi_get_tsf_time(WIFI_IF_AP);
}

void loop_clock_esp1_AP() {
    esp_link.poll();

    delay(4000);

    Pose2D pos_i = {(float)j++, 0, 0, (uint64_t)esp_wifi_get_tsf_time(WIFI_IF_AP)-t0_AP};
    Pose2D to_compare;
    esp_link.get_pos(to_compare);

    if (ABS(pos_i.timestamp_ms, to_compare.timestamp_ms) < 500) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    
}
