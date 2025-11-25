#include "test_common_esp2.h"

int j = 0;
uint64_t t0_STA = 0;

void setup_clock_esp2_STA() {
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, LOW);

    esp_link.begin();
    delay(1000);

    WiFi.mode(WIFI_MODE_STA);
    WiFi.begin("TSF_AP", "12345678");
    delay(1000);

    if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_PIN, LOW);
        delay(40000); // retry with bigger delay for initialisation
    }

    digitalWrite(LED_PIN, HIGH);

    t0_STA = esp_wifi_get_tsf_time(WIFI_IF_STA);
}

void loop_clock_esp2_STA() {
    esp_link.poll();

    delay(4000);

    Pose2D pos_i = {(float)j++, 0, 0, (uint64_t)esp_wifi_get_tsf_time(WIFI_IF_STA)-t0_STA};
    esp_link.sendPos(pos_i);

}
