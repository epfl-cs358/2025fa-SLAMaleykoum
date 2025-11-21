#include "test_common_esp2.h"

#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>
#include "I2C_wire.h"

char buff[100];
const char* mqtt_topic_connection_sensor = "slamaleykoum77/print";
#define LED_PIN LED_BUILTIN   // sinon remplace par 48 si ça ne marche pas


// === Global instances ===

const float EMERGENCY_DISTANCE = 0.25f;

void setup_test_sensor() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);


    
    if (!ultrasonic.begin()) {
        digitalWrite(LED_PIN, LOW);

        // char msgSonic[50];
        // snprintf(msgSonic, sizeof(msgSonic), "UltraSonic Sensor init failed!");
        // connection.publish(mqtt_topic_connection_sensor , msgSonic);
        while (true);
    }

    // snprintf(buff, sizeof(buff), "✅ Ultra Sonic initialized successfully");
    // connection.publish(mqtt_topic_connection_sensor, buff);


}

void loop_test_sensor() { 

    float dist = ultrasonic.readDistance();
    if (dist > 0 && dist <EMERGENCY_DISTANCE) {
        digitalWrite(LED_PIN, LOW);
        // snprintf(buff, sizeof(buff), "⚠️ Emergency stop! Distance=%.2f m", dist);
        // connection.publish(mqtt_topic_connection_sensor, buff);

    }else{
        digitalWrite(LED_PIN, HIGH);
    }

    delay(1000);
    
}
