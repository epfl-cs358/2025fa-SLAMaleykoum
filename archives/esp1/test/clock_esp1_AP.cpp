#include "test_common_esp1.h"

#define ABS(a, b)  ((a > b) ? (a - b) : (b - a))

int j = 0;
uint64_t t0_AP = 0;

const uint16_t TCP_PORT_clock = 9000;

// TCP server and client
WiFiServer tcpServer_clock(TCP_PORT_clock);
WiFiClient tcpClient_clock;

void setup_clock_esp1_AP() {
    pinMode(LED_PIN, OUTPUT);
    delay(1000);

    esp_link.begin();
    delay(1000);
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP("TSF_AP", "12345678");
    delay(3000);

    // Start TCP server
    tcpServer_clock.begin();
    tcpServer_clock.setNoDelay(true);

    delay(1000);

    t0_AP = esp_wifi_get_tsf_time(WIFI_IF_AP);
}

void loop_clock_esp1_AP() {
    // Accept new client if none
    if (!tcpClient_clock || !tcpClient_clock.connected()) {
        WiFiClient newClient = tcpServer_clock.available();
        if (newClient) {
            tcpClient_clock = newClient;
            tcpClient_clock.setNoDelay(true);
        }
    }

    esp_link.poll();

    delay(1000);

    Pose2D pos_i = {(float)++j, 0, 0, (uint64_t)esp_wifi_get_tsf_time(WIFI_IF_AP)-t0_AP};
    delay(2000);
    Pose2D to_compare;
    if (esp_link.get_pos(to_compare)) {
        if (tcpClient_clock && tcpClient_clock.connected()) {
            uint8_t buffer[24];
            size_t offset = 0;

            memcpy(buffer + offset, &pos_i.x, sizeof(float));
            offset += sizeof(float);

            memcpy(buffer + offset, &pos_i.timestamp_ms, sizeof(uint64_t));
            offset += sizeof(uint64_t);

            memcpy(buffer + offset, &to_compare.x, sizeof(float));
            offset += sizeof(float);

            memcpy(buffer + offset, &to_compare.timestamp_ms, sizeof(uint64_t));

            tcpClient_clock.write(buffer, sizeof(buffer));
        }
        delay(2000);
        if ((pos_i.x == to_compare.x) && ABS(pos_i.timestamp_ms, to_compare.timestamp_ms) < 5000) {
            for(int i = 0; i < 10; ++i) {
                digitalWrite(LED_PIN, HIGH);
                delay(200);
                digitalWrite(LED_PIN, LOW);
                delay(200);
            }
        } else {
            digitalWrite(LED_PIN, LOW);
            delay(4000);
        }
    } else {
        digitalWrite(LED_PIN, LOW);
        delay(6000);
    }
}
