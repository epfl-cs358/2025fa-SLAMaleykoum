# **Transforms Library (transforms.h)**

## **Overview**

This is a lightweight, header-only utility library for performing 2D coordinate transformations. It is designed to convert points between the **global** coordinate frame and the **robot-centric** coordinate frame.  
**The functions in this library are inline for performance and modify the Waypoint object in-place.**

## **1\. Coordinate System Definition**

This library assumes a specific coordinate system for the robot based on our IMU orientation on the physical car.

* **Robot Frame:**  
  * **Y-axis:** Points **forward**.  
  * **X-axis:** Points to the **right**.  
  * **Origin:** Center of the car's rear axle.
* **Global Frame:**  
  * The X and Y axes are defined by the robot's orientation at startup (assumed known axis).  
* **Angle (Pose2D::theta):**  
  * Represents the robot's heading.  
  * It is the counter-clockwise (CCW) angle from the **Global X-axis** to the **Robot's X-axis**.  
  * A theta of 0 means the robot's X-axis is aligned with the Global X-axis (and Y-axis with Y-axis).  
  * A theta of 90 degrees (M\_PI / 2.0) means the robot is facing along the Global Y-axis.

## **2\. API Functions**

### **void Transforms::to\_robot\_frame(const Pose2D& robot\_pose, Waypoint& point)**

* **Purpose:** Converts a Waypoint from global coordinates to robot-centric coordinates.  
* **Inputs:**  
  * robot\_pose: The current Pose2D (x, y, theta) of the robot in the global frame.  
  * point: The Waypoint (x, y) in global coordinates.  
* **Output:** The point object is modified **in-place** to contain its (x, y) coordinates relative to the robot.

### **void Transforms::to\_global\_frame(const Pose2D& robot\_pose, Waypoint& point)**

* **Purpose:** Converts a Waypoint from robot-centric coordinates back to global coordinates.  
* **Inputs:**  
  * robot\_pose: The current Pose2D (x, y, theta) of the robot in the global frame.  
  * point: The Waypoint (x, y) in robot-centric coordinates.  
* **Output:** The point object is modified **in-place** to contain its (x, y) coordinates in the global frame.

## **3\. Usage Example**

This example shows how to take a global point, convert it to the robot's frame, and then convert it back to global.
```
#include "common/data_types.h"  
#include "common/transforms.h"  
#include <iostream>  
#include <cmath> // For M_PI

int main() {  
    // 1. Define the Robot's Pose  
    // Robot is at (x=10, y=5) in the global frame.  
    // It is rotated 90 degrees CCW (theta = PI/2).  
    // This means the robot's Y-axis (front) points along the global Y-axis.  
    Pose2D robot_pose = {10.0f, 5.0f, M_PI / 2.0f};

    // 2. Define a Global Point  
    // A point 3 meters in front of the robot.  
    Waypoint global_point = {10.0f, 8.0f};

    std::cout << "Original Global Point: ("   
              << global_point.x << ", " << global_point.y << ")" << std::endl;

    // 3. Convert to Robot Frame  
    Waypoint robot_point = global_point; // Make a copy  
    Transforms::to_robot_frame(robot_pose, robot_point);

    // With X-Right, Y-Fwd frame:  
    // The point is 0m to the right (x=0) and 3m in front (y=3).  
    std::cout << "Robot-Frame Point:     ("   
              << robot_point.x << ", " << robot_point.y << ")" << std::endl;  
    // Expected Output: (0, 3) (or very close due to float precision)

    // 4. Convert Back to Global Frame  
    Transforms::to_global_frame(robot_pose, robot_point);

    std::cout << "Reverted Global Point: ("
              << robot_point.x << ", " << robot_point.y << ")" << std::endl;
    // Expected Output: (10, 8)  

    return 0;  
}
```
