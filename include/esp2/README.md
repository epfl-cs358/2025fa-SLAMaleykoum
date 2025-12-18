# ESP-2: Real-Time Control & Actuation Module

ESP-2 is dedicated to real-time execution, low-latency control, and sensor management.
It operates at a high frequency (50-100 Hz) to ensure stable actuation and precise velocity tracking.

> **Core Mission**: Execute the global path plan stably, maintain smooth velocity control, and gather high-frequency sensor data for both local and global localization.


## Hardware

### Servo :
Control the steering angle of the car.

- **Input**: Desired steering angle (in degrees).
- **Process**: The servo is initialized and attached to the specified GPIO pin. At startup, it is set to a neutral position (110°). During operation, the target angle is sent directly to the servo, which updates its position accordingly. The current angle is stored internally for state tracking.
- **Output**: Steering angle applied to the servo (in degrees).

### Encoder :
Measure the car’s velocity and traveled distance using a magnetic encoder.

- **Input**: Raw angular position data from the AS5600 magnetic encoder via I2C.
- **Process**: The encoder provides motor shaft angular velocity and cumulative position. Motor speed is converted to wheel angular velocity using the gear ratio, then to linear velocity using the wheel radius. The angular velocity can be filtered before conversion to have more stability. The cumulative encoder position is tracked over time and converted into a total traveled distance by integrating incremental wheel rotations. Accurate distance estimation requires frequent polling.
- **Output**: Motor angular velocity, wheel angular velocity, linear velocity, and cumulative traveled distance (in meters).

### I2C Mutex and Wire :
Provide safe and shared access to the I2C bus across multiple components.

- **Input**: I2C read/write requests from multiple sensors and peripherals.
- **Process**: A global FreeRTOS mutex (i2c_mutex) is used to protect all I2C transactions and prevent concurrent access to the bus. Each component locks the mutex before performing I2C operations and releases it afterward.
A single shared TwoWire instance (I2C_wire) is used by all devices to ensure consistent I2C configuration and avoid multiple bus initializations.
- **Output**: Thread-safe and reliable I2C communication for all connected sensors.

### IMU :
Measure the orientation of the car to provide inertial information for state estimation and control.

- **Input**: Raw 9-Axis IMU Readings (Gyro, Accelerometer, Magnetometer).
- **Process**: The IMU is polled at a fixed rate and raw sensor reports are converted into physical quantities. Only the quaternion output is enabled and kept, as it is the only representation that provides reliable data in our setup. These values are stored in a compact IMUData structure and timestamped for later use. No full sensor fusion or localization is performed at this stage.
- **Output**: IMUData containing quaternion orientation and timestamps.

### Motor :
Control the car’s DC motor through the ESC, with low-level PWM handling and high-level motion commands.

- **Input**: Desired motor command (power level or direction).
- **Process**: The MotorController handles low-level ESC control by generating PWM signals. It converts target power levels into pulse widths and applies smooth ramping to avoid sudden changes. An emergency stop immediately sets the ESC to neutral.
The MotorManager provides a higher-level interface built on top of the controller, exposing simple commands such as forward, backward, stop, and emergency stop. It manages direction and power scaling and periodically updates the controller to gradually reach the target speed.
- **Output**: PWM signal sent to the ESC, resulting in controlled motor speed and direction.

### Ultrasonic : 
Sensor used to detect close obstacles in front of the car for safety and emergency handling.

- **Input**: Raw signals from the trigger and echo pins of the ultrasonic sensor.
- **Process**: A trigger pulse is sent, and the echo return time is measured to compute the distance to the obstacle. This distance is continuously monitored and compared against a safety threshold. When an obstacle is detected too close, an emergency stop is triggered and a recovery maneuver can be initiated.
- **Output**: Distance to the nearest obstacle.


## Control

### Pure Pursuit (`pure_pursuit.h`):

[Open Pure Pursuit README](/include/esp2/control/pure_pursuit_readme.md)

Local path-tracking controller responsible for converting global waypoints into low-level steering and speed commands.

- **Input**: `PathMessage` (waypoints) from ESP-1 and robot current pose
- **Process**: selects a lookahead point on the path and computes the steering command using the Pure Pursuit bicycle model
- **Output**: steering angle and target speed commands sent to the motor and steering hardware


### Recovery Maneuvers :

- **Input**: `Current vehicle pose` for yaw estimation.    Global motor and steering interfaces (`MotorManager`, `DMS15`).
- **Process**: Move backward briefly to create clearance from obstacles. Perform a sequence of alternating backward and forward motions : steer wheels to opposite angles (left/right) during each motion.  
Insert short pauses to stabilize the vehicle and reduce mechanical stress
Repeat the sequence a fixed number of times to achieve an approximate quarter-turn

- **Output**: Vehicle reoriented to face approximately an opposite direction.
Motors stopped and steering centered, ready to resume normal navigation.


---

## architecture diagram
# //TODO: UPDATE THIS DIAGRAM.
Here is the current architecture.
![alt text](/assets/docs/esp2/global-architecture-Page-2.png)