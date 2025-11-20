#include "test_common_esp2.h"
 
// Pour la plupart des ESP32-S3 DevKitC :
#define LED_PIN LED_BUILTIN   // sinon remplace par 48 si ça ne marche pas

void setup_led_basic() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    delay(1000);

    Serial.println("setup ok !");
}

void loop_led_basic() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}