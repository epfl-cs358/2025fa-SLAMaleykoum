/**
 * @file EncoderCarVelocity.cpp
 * @brief Implementation of the EncoderCarVelocity class to be used to calculate the car's velocity
 * 
 * 
 * @author SLAMaleykoum
 * @date Nov 2025
 */
//#include "EncoderCarVelocity.h"
//#include "AS5600Encoder.h"
//#include "I2C_mutex.h"
//#include "I2C_wire.h"


/*
EncoderCarVelocity::EncoderCarVelocity(AS5600Encoder* encoder)
    : encoder(encoder), timeElapsed(0.0f), angle_1(0.0f), angle_2(0.0f) {} //might not need later, for testing

float EncoderCarVelocity::getMotorAngularVelocity(){

    
    float angle1 = encoder->update();
    unsigned long time1 = micros();
    
    // Delay to allow motor movement
    delay(5);  // 40ms //5 works 
    
  
    
    float angle2 = encoder->update();
    unsigned long time2 = micros();
    unsigned long timeE2 = micros();
 


    unsigned long dt = time2 - time1;
    timeElapsed = timeE2 - time1;
    angle_1 = angle1;
    angle_2 = angle2;

    timeElapsed = dt;

    
    float delta = angle2 - angle1;
    
    /*
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }*/

    //if abgle 2 is smaller then angle 1 then restarted loop
/*
    if(delta < 0.0f){
        delta += 360.0f;
    }
    

    
    float dt_seconds = dt / 1000000.0f;
    
    if (dt_seconds < 0.001f) {
        return 0.0f;
    }
    
    float motorAngularVelocity = (delta * DEG_TO_RAD) / dt_seconds;
    
    return motorAngularVelocity;
}*/

/*float EncoderCarVelocity::getAverageMotorAngularVelocity(){
    static float filteredVel = 0.0f;
    const float alpha = 0.3f;

    float newVel = getMotorAngularVelocity();
    filteredVel = alpha * newVel + (1 - alpha) * filteredVel;

    return filteredVel;
}*/
/*
const int VELOCITY_SAMPLES = 20; // number of samples to average

float EncoderCarVelocity::getAverageMotorAngularVelocity() {
    static float velBuffer[VELOCITY_SAMPLES] = {0};
    static int index = 0;
    static bool filled = false;

    float newVel = getMotorAngularVelocity();

    velBuffer[index] = newVel;
    index = (index + 1) % VELOCITY_SAMPLES;
    if(index == 0) filled = true;

    int count = filled ? VELOCITY_SAMPLES : index;

    float sum = 0;
    for(int i = 0; i < count; i++) sum += velBuffer[i];

    return sum / (float)count;
}



float EncoderCarVelocity::getWheelAngularVelocity() {
    //return getMotorAngularVelocity() / GEAR_RATIO;
    
    return getAverageMotorAngularVelocity() / GEAR_RATIO;
}

float EncoderCarVelocity::getWheelLinearVelocity(){
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}

float EncoderCarVelocity::getTimeElapsed(){
    return timeElapsed;
}


float EncoderCarVelocity::getAngle1(){
    return angle_1;
}

float EncoderCarVelocity::getAngle2(){
    return angle_2;
}
*/
#include "EncoderCarVelocity.h"
#include <Wire.h>

EncoderCarVelocity::EncoderCarVelocity() {}

// Initialize I2C and AS5600
void EncoderCarVelocity::begin() {
    Wire.begin();  // default SDA/SCL pins
    if (!as5600.begin()) {
        Serial.println("AS5600 not detected!");
        while (1);  // stop if not found
    }

    // Set clockwise rotation, small noise filter
    as5600.setDirection(AS5600_CLOCK_WISE);
    as5600.setHysteresis(2);     // suppress small noise
    as5600.setSlowFilter(2);     // optional, reduces jitter





    
}

// Returns motor angular velocity in rad/sec
float EncoderCarVelocity::getMotorAngularVelocity() {
    // library handles delta / dt internally
    return as5600.getAngularSpeed(AS5600_MODE_RADIANS, true);
}



float EncoderCarVelocity::getFilteredAngularVelocity() {
    /*static float filteredVel = 0.0f;
    const float alpha = 0.02f;

    float rawVel = getMotorAngularVelocity();
    filteredVel = alpha * rawVel + (1 - alpha) * filteredVel;

    return filteredVel;*/
     static float emaVel = 0.0f;
    static float buffer[5] = {0};
    static int idx = 0;

    const float alpha = 0.02f;

    // Update EMA
    float rawVel = getMotorAngularVelocity();
    emaVel = alpha * rawVel + (1 - alpha) * emaVel;

    // Update buffer
    buffer[idx] = emaVel;
    idx = (idx + 1) % 5;

    // Compute small moving average
    float sum = 0;
    for (int i = 0; i < 5; i++) sum += buffer[i];

    return sum / 5.0f;
}




/*

float EncoderCarVelocity::getMotorAngularVelocity(){

    
    float angle1 = as5600.readAngle();//encoder->update();
    unsigned long time1 = micros();
    
    // Delay to allow motor movement
    delay(5);  // 40ms //5 works 
    
  
    
    float angle2 = as5600.readAngle();//encoder->update();
    unsigned long time2 = micros();
    unsigned long timeE2 = micros();
 


    unsigned long dt = time2 - time1;
    //timeElapsed = timeE2 - time1;
    //angle_1 = angle1;
    //angle_2 = angle2;

    //timeElapsed = dt;

    
    float delta = angle2 - angle1;

    //transform into degrees
    delta = delta* AS5600_RAW_TO_RADIANS;
    
    /*
    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }*/

    //if abgle 2 is smaller then angle 1 then restarted loop

    /*if(delta < 0.0f){
        delta += 360.0f;
    }
    

    
    float dt_seconds = dt / 1000000.0f;
    
    if (dt_seconds < 0.001f) {
        return 0.0f;
    }
    
    float motorAngularVelocity = (delta) / dt_seconds;
    
    return motorAngularVelocity;
}
*/
float EncoderCarVelocity::getWheelAngularVelocity() {
    return getFilteredAngularVelocity() / GEAR_RATIO;
}





float EncoderCarVelocity::getWheelLinearVelocity() {
    return getWheelAngularVelocity() * WHEEL_RADIUS;
}


int32_t EncoderCarVelocity::getCumulativePosition() {
    return as5600.getCumulativePosition();
}
