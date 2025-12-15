# **Motor Velocity PID Controller**

A time-based PID controller designed for the closed-loop velocity control of the SLAM car's drive motor.
Its primary role is to take a **target velocity** (from the Pure Pursuit algorithm) and a **current velocity** (from odometry/sensor fusion) and compute the exact motor PWM signal required to make them match. (Adjust the current velocity to match the target velocity.)

## **Core Principles \- How This PID Works**

A PID controller works by calculating a "control signal" based on three terms, which correct for past, present, and future errors. The goal is to **minimize** the error between the target and current velocities.

### **1\. Proportional (P) \- The Present**

* **Concept**: This term provides a response that is proportional to the **current error**.  
* **Equation**: Error \= TargetVelocity \- CurrentVelocity  
* **Behavior**: If you are far from your target speed, the P-term gives a large correction. If you are close, it gives a small one. It's the main "driving" force of the controller.
* **Limitation**: By itself, a P-only controller often results in a **steady-state error**. For example, when going up an incline, the P-term alone might not be strong enough to ever reach the *full* target speed. Genreally speeking, it will often result in oscillation and *long* convergence time to it's target speed.

### **2\. Integral (I) \- The Past**

* **Concept**: This term looks at the **accumulation of past errors**. It sums up all previous errors and multiplies them by the time they persisted (dt).  
* **Behavior**: If the P-term consistently fails to reach the target (causing a small, persistent steady-state error), this integral\_ sum will grow over time. This growth adds more and more power until the error finally becomes zero. This is what defeats steady-state error (like on our incline example). It accelerates the convergance.
* **Key Feature (Anti-Windup)**: This implementation includes "integral anti-windup." The integral\_ term is "clamped" (limited) to a PID\_INTEGRAL\_MAX value. This prevents it from growing to a massive, unstable value if the car gets stuck, which would otherwise cause extreme overshooting. (e.g. wheel slipping, steep incline, human holding the car, etc.)

### **3\. Derivative (D) \- The Future**

* **Concept**: This term provides a response proportional to the **rate of change of the error**. It's essentially predicting the *future* error.  
* **Behavior**: The D-term acts as a damper, smoothing the controller's response.  
  * If the car is approaching the target speed *very quickly* (error is closing fast), the D-term will apply a "braking" force to prevent overshooting.  
  * If the car is suddenly knocked away from its target (like hitting a bump), the D-term provides a quick boost to counteract the disturbance.  
* It makes the controller more stable,  **less prone to oscillation**.

### **Final Output**

The final control signal is the sum of these three terms:  
$ControlSignal = (K_p * Error) + (K_i * Integral) + (K_d * Derivative)$
This ControlSignal (a float value, clamped between \-1.0 and 1.0) is then scaled by this class to the precise PWM pulse width required by the ESC (e.g., 1000µs for full-reverse, 1500µs for neutral, 2000µs for full-forward).

## **Implementation Details**

* **motor\_pid.h**: Defines the MotorPID class, its constructor, public methods (`compute_pwm_output`, `reset`), and private member variables (the gains `Kp_`, `Ki_`, `Kd_` and the state variables `integral_`, `prev_err_`).  
* **motor\_pid.cpp**: Implements the class logic.  
  * **Time-Based (dt)**: All integral and derivative calculations are multiplied or divided by `dt` (the time delta in seconds). This makes the controller's behavior consistent, regardless of the loop's frequency.
  * **Direct PWM Output**: This class handles all the math. You give it velocities, and it gives you back the exact MotorOutputs (`uint16_t`) PWM value to send to the MotorController.

## **API Usage**

Here is a basic example of how to integrate the MotorPID into the main control loop.  
```
#include "motor_pid.h"  
#include "MotorController.h"  
#include "common/data_types.h"

// 1. Define your tuned gains
// THESE GAINS MUST BE TUNED AND WILL TAKE A LOT OF TIME TO DO SO!
#define KP 1.5  
#define KI 0.8  
#define KD 0.2

// 2. Instantiate your PID controller and Motor  
MotorPID motor_pid(KP, KI, KD);  
MotorController motor_controller(MOTOR_PWM_PIN);

// 3. Keep track of time  
unsigned long last_loop_time = 0;

void setup() {  
    motor_controller.begin();  
    last_loop_time = micros();  
}

void loop() {  
    // --- 1. Timing ---  
    unsigned long now_us = micros();  
    float dt = (now_us - last_loop_time) / 1000000.0f; // dt in seconds  
    last_loop_time = now_us;

    // --- 2. Get Inputs ---  
    // (Get these from Pure Pursuit and Odometry/EKF)  
    Velocity target_vel;  
    target_vel.v_linear = 1.0; // Target 1.0 m/s

    Velocity current_vel;  
    current_vel.v_linear = 0.8; // Measured 0.8 m/s

    // --- 3. Compute ---  
    // The PID controller does all the hard work  
    MotorOutputs pwm_signal = motor_pid.compute_pwm_output(target_vel, current_vel, dt);

    // --- 4. Actuate ---  
    motor_controller.setTargetUs(pwm_signal);  
    motor_controller.update(); // Call this to apply the new PWM value  
}
```


### **Public Methods**

* MotorPID(float Kp, float Ki, float Kd)  
  * The constructor. Initializes the PID gains.  
* MotorOutputs compute\_pwm\_output(const Velocity& target\_vel, const Velocity& current\_velocity, float dt)  
  * The main computation function. Call this once per loop.  
* void reset()  
  * Resets the controller's internal state (integral\_ and prev\_err\_). Call this if the PID is disabled for a period to prevent it from acting on stale data.

## **Tuning**

The gains Kp, Ki, and Kd, as well as the integral clamp limits (PID\_INTEGRAL\_MIN/MAX), are critical to the car's performance. These values **must** be tuned experimentally for the specific physics (weight, friction, motor power) of your car.

## **Resources**

* Good intuition course on youtube: https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y  
* Aplication to a "wall following" controller in F1TENTH course: https://youtu.be/qIpiqhO3ITY?si=5K6qbe66yPaF3mj-
* 