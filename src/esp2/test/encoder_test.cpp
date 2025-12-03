#include "test_common_esp2.h"

#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>

char buffencoder[100];
const char* mqtt_topic_connection_encoder= "slamaleykoum77/print";


// === Global instances ===



void setup_test_encoder() {
    Serial.begin(115200);
    delay(2000);

    ///connection.setupWifi();


    // Wifi connection 
    //connection.check_connection();

    pinMode(LED_BUILTIN, OUTPUT);

    //digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);

    connection.setupWifi();

    I2C_wire.begin(8, 9);
    // I2C setup for IMU and Encoder
    i2cMutexInit();

    // Wifi connection 
    connection.check_connection();

   
    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_encoder,msgEncoder);
        while (true);
    }

   
    if (!motor.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Motor init failed!");
        connection.publish(mqtt_topic_connection_encoder,msgEncoder);
        while (true);
    }


}

void loop_test_encoder() { 

    
    motor.forward(0.17);
    motor.update();

    i2c_lock();
    float currVel = encoder.getWheelLinearVelocity();   // updates internal velocity state
    float newY = encoder.getDistance();
    i2c_unlock();

    char msgEncoder[50];
    snprintf(msgEncoder, sizeof(msgEncoder), "velocity = %0.3f , Y = %0.3f", currVel, newY);
    connection.publish(mqtt_topic_connection_encoder,msgEncoder);

    
}
