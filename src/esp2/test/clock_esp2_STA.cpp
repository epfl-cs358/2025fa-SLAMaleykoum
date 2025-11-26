#include "test_common_esp2.h"

int j2 = 0;
uint64_t t0_STA = 0;

WiFiClient tcpClient_clock;

void setup_clock_esp2_STA() {
    pinMode(LED_PIN, OUTPUT);
    delay(1000);

    esp_link.begin();
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.begin("TSF_AP", "12345678");
    delay(3000);
    
    t0_STA = esp_wifi_get_tsf_time(WIFI_IF_STA);
}

void loop_clock_esp2_STA() {
    esp_link.poll();

    delay(1000);

    Pose2D pos_i = {(float)++j2, 0, 0, (uint64_t)esp_wifi_get_tsf_time(WIFI_IF_STA)-t0_STA};
    esp_link.sendPos(pos_i);
    delay(2000);

    delay(6000);
}
