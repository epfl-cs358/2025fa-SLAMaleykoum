#include "test_common_esp2.h"

#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>
#include "esp2/localization/slip_localizer2.h"

char buffekf2[100];
const char* mqtt_topic_connection_encoder_ekf2= "slamaleykoum77/ekf2";
uint32_t prevTime; 


// === Global instances ===


void setup_test_ekf2() {
    Serial.begin(115200);
    delay(2000);

    pinMode(LED_BUILTIN, OUTPUT);

    delay(2000);

    connection.setupWifi();

    I2C_wire.begin(8, 9);
    i2cMutexInit();

    // Wifi connection 
    connection.check_connection();

   
    if (!encoder.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Encoder init failed!");
        connection.publish(mqtt_topic_connection_encoder_ekf2,msgEncoder);
        while (true);
    }

   
    if (!motor.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "Motor init failed!");
        connection.publish(mqtt_topic_connection_encoder_ekf2,msgEncoder);
        while (true);
    }
    if  (!imu.begin()) {
        char msgEncoder[50];
        snprintf(msgEncoder, sizeof(msgEncoder), "IMU init failed!");
        connection.publish(mqtt_topic_connection_encoder_ekf2,msgEncoder);
        while (true);
    }

    prevTime = micros();


}

void loop_test_ekf2() { 
    uint32_t now = micros();
    float dt = (now - prevTime) / 1000000.0f;
    prevTime = now;
    
    motor.forward(0.17);
    motor.update();

    i2c_lock();
    imu.readAndUpdate();
    IMUData imuData = imu.data();
    float currentVel = encoder.getWheelLinearVelocity();
    i2c_unlock();
    slipLocalizer.update(currentVel, imuData, dt);
    Pose2D pose = slipLocalizer.get_pose();
    float vel = slipLocalizer.get_velocity().v_linear;

    char msgekf2[50];
    snprintf(msgekf2, sizeof(msgekf2), "velocity_enc = %0.3f, x= %0.3f, y= %0.3f, θ= %0.3f", vel, pose.x, pose.y, pose.theta);
    connection.publish(mqtt_topic_connection_encoder_ekf2,msgekf2);

    
}
