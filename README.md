
# SLAMaleykoum
---
Project proposal : https://www.overleaf.com/9942875199zgzbkrmgkkkj#3fbbb2

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
3. [Hardware Platform](#hardware-platform)
4. [Software Components](#software-components)
5. [Data Flow and Communication](#data-flow-and-communication)
6. [Implementation Details](#implementation-details)
7. [Mission and Task Management](#mission-and-task-management)

---

## Project Overview

### Vision
SLAMaleykoum is an autonomous robotics project that transforms an RC car chassis into an intelligent mapping and navigation platform. The system performs **Simultaneous Localization and Mapping (SLAM)** to autonomously explore and map unknown environments while maintaining accurate position estimates.

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

## System Architecture

### High-Level Design Philosophy

The project employs a **distributed computing architecture** that divides computational load across two ESP32-S3 microcontrollers. SLAM and path planning are computationally intensive; distributing tasks prevents processor saturation.

### ESP-1: Mapping & Global Planning

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
