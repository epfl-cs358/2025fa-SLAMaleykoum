
#ifndef ENCODER_CAR_VELOCITY_H
#define ENCODER_CAR_VELOCITY_H

#include <AS5600.h>

#define GEAR_RATIO 10.0f      
#define WHEEL_RADIUS 0.0495f //maybe needs to be adapted but 9.9 diametre  


class EncoderCarVelocity {
public:
    EncoderCarVelocity();

    void begin();  


    float getMotorAngularVelocity();  
    float getFilteredAngularVelocity();
    float getWheelAngularVelocity();   
    float getWheelLinearVelocity();    
    float getFilteredMotorAngularVelocity();
  
    int32_t getCumulativePosition();

private:
    AS5600 as5600;
};

#endif
