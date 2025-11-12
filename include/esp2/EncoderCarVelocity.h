//#ifndef ENCODER_CAR_VELOCITY_H
//#define ENCODER_CAR_VELOCITY_H

//#include <Arduino.h>
/*class AS5600Encoder;

class EncoderCarVelocity {
public:
    EncoderCarVelocity(AS5600Encoder* encoder);

    float getMotorAngularVelocity();
    float getWheelAngularVelocity();
    float getWheelLinearVelocity();
    float getTimeElapsed();
    float getAngle1();
    float getAngle2();
    float getAverageMotorAngularVelocity();
 

private:
    AS5600Encoder* encoder;

    float timeElapsed;
    float lastAngle;          
    float unwrappedAngle;      
    unsigned long lastTime;
    float motorAngularVelocity;

    float angle_1;
    float angle_2;


    const unsigned long VELOCITY_INTERVAL_MS = 5; //fixed sampling interval in ms

    const float GEAR_RATIO = 10.0f; //it takes the motor 10 full rotations to make the wheel turn a full rotation
    const float WHEEL_RADIUS = 0.0485f; //wheel radius in meters
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t RAW_ANGLE_REG = 0x0C;
};

#endif
*/
#ifndef ENCODER_CAR_VELOCITY_H
#define ENCODER_CAR_VELOCITY_H

#include <AS5600.h>

#define GEAR_RATIO 10.0f      // Set your gear ratio
#define WHEEL_RADIUS 0.0495f   // meters, set your wheel radius

class EncoderCarVelocity {
public:
    EncoderCarVelocity();

   

    void begin();  // initialize I2C and AS5600

    // Motor / Wheel Velocities
    float getMotorAngularVelocity();   // rad/sec
    float getFilteredAngularVelocity();
    float getWheelAngularVelocity();   // rad/sec
    float getWheelLinearVelocity();    // m/s
    float getFilteredMotorAngularVelocity();

    // Optional cumulative position (revolutions)
    int32_t getCumulativePosition();

//private:
    AS5600 as5600;
};

#endif
