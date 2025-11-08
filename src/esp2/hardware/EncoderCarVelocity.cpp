/**
 * @file EncoderCarVelocity.cpp
 * @brief Implementation of the EncoderCarVelocity class to be used to calculate the car's velocity
 * 
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */
#include "EncoderCarVelocity.h"
#include "AS5600Encoder.h"
#include "I2C_mutex.h"
#include "I2C_wire.h"



EncoderCarVelocity::EncoderCarVelocity(AS5600Encoder* encoder)
    : encoder(encoder), timeElapsed(0.0f) {} //might not need later, for testing

float EncoderCarVelocity :: getMotorAngularVelocity(){
    unsigned long timeElapsed1 = micros();


    unsigned long time1 = micros();
    float angle1 = encoder->update();

    unsigned long time2 = micros();
    float angle2 = encoder->update();

    unsigned long timeElapsed2 = micros();

    timeElapsed = timeElapsed2 - timeElapsed1;
    
    unsigned long dt = time2 - time1; 
    float delta = angle2 - angle1;

    
  
    
    //Unwrap using 180° rule, if we measure 2 roations before the magnet was able to do more than half of a full revolution,
    //we will be able to tell if the rotation was forward or backwards because 0 to 170 degrees means only way was forward
    //because you need 190 degrees backwards otherwise
    // 0 to 0 didn't roate beacuse could'nt have don e a full revolution
    
    //adjust the degree depending on if the movement was backwards or forwards
    if (delta > 180.0f) { //could have only been a backwards rotation sicne over 180 degrees forward is impossible
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }
    
    // Calculate velocity
    float dt_seconds = dt / 1000000.0f;  // micros to seconds
    float motorAngularVelocity = (delta * DEG_TO_RAD) / dt_seconds;

    return motorAngularVelocity;

}




float EncoderCarVelocity::getWheelAngularVelocity() {
    return getMotorAngularVelocity() / GEAR_RATIO;
}

float EncoderCarVelocity::getWheelLinearVelocity(){
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}

