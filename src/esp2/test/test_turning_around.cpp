#include "test_common_esp2.h"

#include "common/data_types.h"
#include <Arduino.h>
#include <cmath>

float yaw_radians_initial = 0.0f;

float getYaw_turn(const IMUData& imu_data) {
  // Extract quaternion components
  float qw = imu_data.qw;
  float qx = imu_data.qx;
  float qy = imu_data.qy;
  float qz = imu_data.qz;

  // Yaw (z-axis rotation)
  // This is the standard formula for quaternion-to-euler conversion
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  float yaw = atan2(siny_cosp, cosy_cosp);

  return yaw; // Yaw angle in radians
}

void setup_test_turning_around() {
    Serial.begin(115200);
    delay(2000);

    I2C_wire.begin(8, 9);
    i2cMutexInit();
 

    if (!servo_dir.begin()) { Serial.println("Servo init failed!"); while (true); }
    else { Serial.println("Servo initialized"); }
    servo_dir.setAngle(90);
    
    
    if (!imu.begin()) { Serial.println("IMU init failed!"); vTaskDelete(NULL); return; }
    else { Serial.println("IMU initialized"); }
    
    if (!encoder.begin()) { Serial.println("Encoder init failed!"); while (true); }
    else { Serial.println("Encoder initialized"); }
    
    if (!motor.begin()) { Serial.println("Motor init failed!"); while (true); }
    else { Serial.println("Motor initialized"); }

    imu.readAndUpdate();

    yaw_radians_initial= getYaw_turn(imu.data());
    //float yaw_degrees = yaw_radians * 180.0f / PI;
    imu.readAndUpdate();

   
    



    

   
    
}

float yaw__radians = 0.0f;
float yaw__degrees = 0.0f;

void loop_test_turning_around() {

    imu.readAndUpdate();
    yaw__radians = getYaw_turn(imu.data());
    yaw__degrees = yaw__radians * 180.0f / PI;

    for(int i = 0 ; i< 5; i++){
    //while(yaw__degrees < 180 || yaw__degrees> -178){
        
        servo_dir.setAngle(120);
        delay(500);
        for(int i = 0; i<6; i++){
            motor.backward(0.21f);
            motor.update();
        }

        delay(800);



        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }

        delay(1500);

        servo_dir.setAngle(60);
        delay(500);
        for(int i = 0; i<8; i++){
            motor.forward(0.17f);
            motor.update();
        }

        delay(800);



        for(int i = 0; i<3; i++){
            motor.stop();
            motor.update();
        }

        delay(1500);
        yaw__degrees = yaw__radians * 180.0f / PI;
    }

    servo_dir.setAngle(90);

    delay(1000000);
    


    
    

   
    
}
