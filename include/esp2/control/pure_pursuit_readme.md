# **Pure Pursuit: Path Tracker**

## **Overview**

This is a path-tracking controller that implements the Pure Pursuit algorithm. Its goal is to compute the necessary `MotionCommand` (target speed and steering angle) to make the car follow a predefined path.  
This controller relies on a **Bicycle Kinematic Model**. It simplifies the car into a bicycle with a wheelbase L\_, a single rear wheel, and a single steerable front wheel.

## **1\. Core Concept**

1. **Find Target Point:** The algorithm "looks ahead" on the path by a certain `lookahead_distance` (Ld). It finds a target waypoint on the path at this distance.  
2. **Transform Coordinates:** This global target point is transformed into the car's local coordinate frame (using the Transforms library). This gives a local target ($x_r$, $y_r$).  
3. **Calculate Steering:** Using the bicycle model, it calculates the circular arc needed to intercept this local target point. The steering angle $\\delta$ is calculated from this arc.  
   * The formula is: $\delta = \arctan(2 \cdot L \cdot x_{\text{lateral}} / Ld^2)$  
   * L is the car's wheelbase (`L_`).  
   * x\_lateral is the lateral (side-to-side) distance to the target in the car's frame.  
   * Ld is the lookahead distance. 
4. **Calculate Speed:** The target speed is adapted based on the required steering angle. Sharper turns (larger $\delta$) result in a lower target speed. We want to slow down for turns to avoid slipping.

## **2\. Calibration**

We **must** tune the following constants inside `pure_pursuit.h` for our car.

* **L\_ (float):** The wheelbase (distance in meters) between the front and rear axles. This is the most critical kinematic parameter.  
* **MAX\_STEERING\_ANGLE\_RAD\_ / MIN\_STEERING\_ANGLE\_RAD\_ (float):** The physical steering limits of the car in radians. (e.g., 25.0f \* M\_PI / 180.0f).  
* **min\_lookahead\_dist\_ / max\_lookahead\_dist\_ (float):** The clamping range for the lookahead distance.  
* **K\_dd\_ (float):** The lookahead gain.
* **min\_speed\_ / max\_speed\_ (float):** The clamping range for the output target speed.
* **K\_v\_ (float):** The velocity gain.  

## **4\. API Functions**

### **void PurePursuit::set\_path(const GlobalPathMessage& path\_msg)**

* **Purpose:** Clears the old path and loads a new one for the controller to follow.  
* **Input:** path\_msg: A message/struct containing an array of Waypoint objects and the path length.  
* **Note:** This also resets the path-following index, so the controller will start tracking from the beginning of the new path.

### **MotionCommand PurePursuit::compute\_command(const Pose2D& current\_pose, const Velocity& vel)**

* **Purpose:** This is the main update function. It performs the full Pure Pursuit calculation.  
* **Inputs:**  
  * **current\_pose**: The car's current position and heading from localization EKF.  
  * **vel**: The car's current velocity.  
* **Returns:** A MotionCommand struct containing:  
  * **target\_velocity**: The calculated adaptive speed.  
  * **steering\_angle**: The calculated steering angle (in radians), clamped to the vehicle's limits.

## **5\. Usage Example (Conceptual Control Loop)**

This example shows how to integrate the PurePursuit controller into a typical robot control loop.

```
#include "pure_pursuit.h"
#include "ekf_localizer.h"
#include "motor_controller.h"
#include "common/data_types.h"

// 1. Initialize hardware and controller
PurePursuit tracker;
EKFLocalizer ekf;
MotorController motors;

// 2. Create and set a path (Will be "automated" in final version of slamcar)
GlobalPathMessage my_path;
my_path.current_length = 3;
my_path.path[0] = {0.0f, 5.0f};  // Go 5m fwd
my_path.path[1] = {5.0f, 5.0f};  // Go 5m right
my_path.path[2] = {5.0f, 10.0f}; // Go 5m fwd
// ... (load a real path)

tracker.set_path(my_path);
bool is_following = true;

// 3. Start the main control loop
while (is_following) {
    // Get current state from sensors
    Pose2D current_pose = ekf.get_pose();
    Velocity current_vel = ekf.get_velocity(); // Or from odometry ?

    // Compute the control command
    MotionCommand cmd = tracker.compute_command(current_pose, current_vel);

    // Send commands to hardware
    motors.set_steering(cmd.steering_angle);   // Assumes radians
    motors.set_speed(cmd.target_velocity);     // Assumes m/s

    // Check if we reached the end of the path (add some logic)
    // if (tracker.is_path_complete(current_pose)) {
    //     is_following = false;
    //     motors.stop();
    // }

    // sleep_ms(20); // Wait for the next control cycle
}
```
