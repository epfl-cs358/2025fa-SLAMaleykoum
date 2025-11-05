#ifndef ENCODER_CAR_VELOCITY_H
#define ENCODER_CAR_VELOCITY_H

#include <Arduino.h>
class AS5600Encoder;

class EncoderCarVelocity {
public:
    EncoderCarVelocity(AS5600Encoder* encoder);

    void update(unsigned long currentMillis);

    float getMotorAngularVelocity() const;
    float getWheelAngularVelocity() const;
    float getWheelLinearVelocity() const;
 
    float getLastAngle() const;
 
    float getUnwrappedAngle() const { return unwrappedAngle; }

private:
    AS5600Encoder* encoder;

    float lastAngle;          
    float unwrappedAngle;      
    unsigned long lastTime;
    float motorAngularVelocity;


    
    const float GEAR_RATIO = 10.0f; //it takes the motor 10 full rotations to make the wheel turn a full rotation
    const float WHEEL_RADIUS = 0.045f; //wheel radius in meters
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t RAW_ANGLE_REG = 0x0C;
};

#endif
