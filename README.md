
# SLAMaleykoum
---
Project proposal : https://www.overleaf.com/9942875199zgzbkrmgkkkj#3fbbb2

## Table of Contents
1. [Set it up](#set-it-up)
2. [Project Overview](#project-overview)
3. [Hardware Overview](#hardware-overview)
4. [System Architecture](#system-architecture)
5. [Hardware Platform](#hardware-platform)
6. [Software Components](#software-components)
7. [Data Flow and Communication](#data-flow-and-communication)
8. [Implementation Details](#implementation-details)
9. [Mission and Task Management](#mission-and-task-management)


---

## Set it up
We recommend you create a python [virtual environment](https://docs.python.org/3/library/venv.html) (venv) for the project (e.g. `python3 -m venv slamaleykoum_venv`).
Activate the venv (`source slamaleykoum_venv/bin/activate`) and install the requirements from our "requirements.txt" file: `pip install -r requirements.txt`
*You could alternatively manually install every requirement (which you might already have).* 

## Project Overview

### Vision
SLAMaleykoum is an autonomous robotics platform that transforms a standard RC car chassis into an intelligent mobile robot capable of mapping and navigating unknown environments. The system performs full Simultaneous Localization and Mapping (SLAM), enabling the robot to autonomously explore, build a map, localize itself with high accuracy in real time, navigate to target coordinates, dynamically avoid obstacles, and maintain a reliable estimate of its position throughout the mission.

### Technical Vocabulary
- Ground Station: **GS**  _refering to the laptop or some other computer external to the car._


### Key Objectives
- **Autonomous Mapping**: Create detailed environment maps using LiDAR sensor data
- **Robust Localization**: Maintain accurate pose estimation through sensor fusion
- **Intelligent Navigation**: Plan and execute collision-free paths to destinations
- **Real-time Control**: Execute precise motor control for path following
- **Remote Monitoring**: Provide ground station (GS) visualization and emergency override capabilities
> **Important**: The car should be able to opperate fully autonomously -> with **no** GS.
> No computations nor storage will be offloaded to the GS. 

### Core Technologies
- **Hardware**: Dual ESP32-S3 microcontrollers, LiDAR sensor, wheel encoders, IMU
- **Operating System**: FreeRTOS for real-time multitasking
- **Communication**: UART (inter-processor), MQTT (ground station)
- **Algorithms**: Extended Kalman Filter, A* pathfinding, Pure Pursuit control, Bayesian occupancy mapping

---

## Hardware Overview

## Component List

| Component                     |           Info                                                                                                                                                                                               |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Tamiya Blitzer Beetle         | [Manual](https://www.tamiyausa.com/media/files/58502ml-829-5367.pdf)                                                                                                                               |
| RPLIDAR C1                    | [Datasheet](https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf), [SDK](https://github.com/Slamtec/rplidar_sdk), [Wiki](https://www.waveshare.com/wiki/RPLIDAR_C1) |
| ESP32-S3 Microcontroller (x2) | [Datasheet](https://cdn-shop.adafruit.com/product-files/5477/esp32-s3_datasheet_en.pdf)                                                                                                            |
| DMS15 Servo                   | [Wiki](https://wiki.dfrobot.com/DSS-M15S_270%C2%B0_15KG_DF_Metal_Servo_with_Analog_Feedback_SKU__SER0044)                                                                                          |
| BNO086 IMU                    | [Datasheet](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080_085-Datasheet_v1.16.pdf)                                                         |
| AS5600 Encoder                | [Datasheet](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf)                             |
| HC-SR04 Ultrasonic sensor     | [Datasheet](https://handsontec.com/dataspecs/sensor/SR-04-Ultrasonic.pdf)                                                                                                                          |
| 540J Motor                    | [Datasheet](https://asset.conrad.com/media10/add/160267/c1/-/en/001385115DS01/adatlap-1385115-540-es-motor-reely-532114c.pdf)                                                                      |
| THW-1060-RTR ESC              | [Datasheet](https://www.hobbywing.com/en/uploads/file/20221015/f60b7ebe160a7b283927ae8916d36763.pdf)                                                                                               |
| LM2596 Buck converter         | [Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)                                                                                                                                          |
| 7.2V Battery                  | [Product Page](https://www.galaxus.ch/fr/s5/product/gens-ace-modelisme-dune-batterie-720-v-5000-mah-batterie-rc-9459930)                                                                           |


## System Architecture

### High-Level Design Philosophy

The project employs a **distributed computing architecture** that divides computational load across two ESP32-S3 microcontrollers. SLAM and path planning are computationally intensive; distributing tasks prevents processor saturation.

### ESP-1: Mapping & Planning

  👉 [Open ESP-1 README](include/esp1/README.md)

**Primary Mission**: Create and maintain a global understanding of the environment and plan high-level navigation strategies.

**Operating Frequency**: 1-10 Hz (depends on task)

**Core Responsibilities**:
- Execute EKF-SLAM algorithm for global pose estimation
- Generate and maintain occupancy grid maps
- Perform loop closure detection to correct accumulated drift
- Plan global paths using A* algorithm
- Manage mission goals and exploration strategies
- Share maps with ground station via MQTT

**Key Characteristics**:
- Operates at lower frequency but higher computational complexity
- Maintains "ground truth" global pose estimate
- Provides correction signals to ESP-2 when loop closures are detected

### ESP-2: Localization & Control

  👉 [Open ESP-2 README](include/esp2/README.md)

**Primary Mission**: Execute precise vehicle control and maintain high-frequency local pose tracking.

**Operating Frequency**: 50-100 Hz for control loops

**Core Responsibilities**:
- Perform local pose estimation using high-frequency EKF
- Execute Pure Pursuit path following algorithm
- Implement PID motor control for velocity and steering
- Aggregate sensor data (odometry, IMU) and forward to ESP-1
- Apply loop closure corrections from ESP-1
- Handle emergency stop commands

**Key Characteristics**:
- Operates at high frequency with lower latency
- Focused on reactive control rather than planning
- Maintains local pose estimate between global corrections

### Why This Architecture?

**Advantages**:
1. **Performance**: Each processor can focus on its task without competing for resources
2. **Modularity**: Components are well-isolated, making testing and debugging easier
3. **Reliability**: Control loop continues operating even if global planning encounters issues

**Trade-offs**:
1. **Synchronization**: Must manage time alignment between processors
2. **Communication Overhead**: Data must be serialized and transmitted between ESPs
3. **Pose Drift**: ESP-2's local pose accumulates drift between corrections from ESP-1

---

## Software Components

### Shared Data Structures (`common/data_types.h`)

Definitions of shared data structures that enable communication between components:

### ESP-1 Components

#### 1. **EKF_SLAM** (`mapping/slam/ekf_slam.h`)
**Purpose**: Global SLAM using Extended Kalman Filter with landmark-based mapping
**Algorithm**: Maintains state vector containing robot pose and landmark positions, with associated covariance matrix for uncertainty quantification.

#### 2. **LandmarkMap** (`mapping/slam/landmark_map.h`)
**Purpose**: Data association and landmark database management
**Data Association**: Will implement nearest-neighbor or JCBB (Joint Compatibility Branch and Bound) for matching.

#### 3. **BayesianOccupancyGrid** (`mapping/occupancy/bayesian_grid.h`)
**Purpose**: Probabilistic occupancy mapping for path planning
**Algorithm**: Log-odds representation with inverse sensor model updates. Handles sensor uncertainty and accumulates evidence over time.

#### 4. **LoopClosure** (`mapping/loop_closure/loop_closure.h`)
**Purpose**: Detect when robot revisits previous locations, based on that, corrects accumulated drift
**Approach**: Maintains keyframe database of historical scans and poses. When current scan matches a previous location, calculates correction to eliminate drift.

#### 5. **GlobalPlanner** (`planning/global_planner.h`)
**Purpose**: Compute collision-free paths using A* algorithm
**Algorithm**: A* search on occupancy grid with configurable heuristic. Produces waypoint sequence for ESP-2.

#### 6. **GoalManager** (`planning/goal_manager.h`)
**Purpose**: High-level mission coordination and goal selection
**Mission States**:
- `STATE_IDLE`: Waiting for commands
- `STATE_EXPLORING`: Autonomous frontier exploration
- `STATE_NAVIGATING`: Moving to user waypoint
- `STATE_RETURNING_HOME`: Return to start position
- `STATE_EMERGENCY_STOP`: Halt all operations

### ESP-2 Components

#### 1. **EKFLocalizer** (`localization/ekf_localizer.h`)
**Purpose**: High-frequency local pose estimation from, imu and odometry, for control

#### 2. **PurePursuit** (`control/pure_pursuit.h`)
**Purpose**: Path following algorithm for smooth trajectory tracking
**Algorithm**: Geometric path follower that computes steering angle to reach a "lookahead point" on the path. Inherently stable and provides smooth motion.

#### 3. **MotorPID** (`control/motor_pid.h`)
**Purpose**: Low-level velocity control using PID feedback
**Control Loops**: Separate PID controllers for linear and angular velocity. Converts high-level commands to PWM duty cycles.

#### 4. **Actuation** (`hardware/actuation.h`)
**Purpose**: Hardware abstraction layer for sensors and actuators
**Abstraction Benefits**: Isolates hardware-specific code, enabling testing with mocked sensors.

---

## Data Flow and Communication

### Inter-ESP Communication (UART)

**ESP-2 → ESP-1 (1-2 Hz)**:
- Aggregated odometry measurements
- IMU data samples
- Current local pose estimate
- Status and telemetry

**ESP-1 → ESP-2 (Event-driven + periodic)**:
- Global path updates (when new path computed)
- Loop closure corrections (when loop detected)
- Mission status updates

**Design Rationale**:
- Low-frequency communication reduces UART overhead
- ESP-2 aggregates high-frequency sensor data before transmission
- Event-driven updates for time-critical information

### Ground Station Communication (MQTT)

**ESP-1 → Ground Station**:
- Real-time map visualization data
- Robot pose and trajectory
- Landmark positions
- Mission status and telemetry
- Debug information

**Ground Station → ESP-1**:
- User-defined waypoints ("manual control")
- Mission mode commands
- Emergency stop override
- Parameter adjustments

**Benefits**:
- MQTT provides publish-subscribe pattern for efficient multi-subscriber support
- Quality of Service (QoS) levels ensure critical commands are delivered
- WiFi enables wireless operation without tether







## Software architecture
The computational load will be devided over the two ESP32-S3 microcontrollers. The first one taking care of the mapping & global planning, while the second one takes care of localization & control of the vehicle. We run **FreeRTOS** to take care of our parallelism and we are comunicating between our two ESPs via **UART** (via the serial ports).

#### Diagram
![alt text](/docs/global-architecture-2.png)


// TODO: FINISH README

#### File structure:
For now the file structure is still in development.
You can the things that WILL NOT CHANGE are mainly the hardware files, since they will "just" get/send data from/to the sensors/actuators and do so in a "nice" way to make it easily accessible from other modules. (aka integrate them with our custom api)

Parts like the `pid_controller` or `ekf_filter` in `odometry/` will not change either since regardless of how we link everyhting those are things we MUST do.



---
