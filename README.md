# SLAMaleykoum

> An autonomous mobile robot that transforms an RC car into an intelligent mapping and navigation platform using dual ESP32-S3 microcontrollers.

![Alt](assets/Images/Design_after_left_side.jpeg)

**Live Demo - 19.12.2025**
<p align="center">
  <img src="assets/Demo/DLL-run.gif" width="54%" />
  <img src="assets/Demo/DLL-run-whatsapp-video.gif" width="30%" />
</p>

_Screen recording sped up x3, video in real speed (for reference)_

## Project Context

**Abstract**

SLAMaleykoum aims to implement **Simultaneous Localization and Mapping (SLAM)**, a robotics technique that enables a mobile robot to build a map of an unknown environment while simultaneously tracking its own location within that map. This chicken-and-egg problem requires three core capabilities working in concert:

1. **Mapping**: Building an accurate representation of the environment using LiDAR sensor data processed through a Bayesian occupancy grid with Bresenham ray tracing
2. **Localization**: Determining the robot's position by fusing wheel encoder odometry with IMU orientation data
3. **Navigation**: Planning collision-free paths using A* search on the occupancy grid and executing them with Pure Pursuit trajectory tracking

The system runs entirely on two ESP32-S3 microcontrollers with no external computation, using FreeRTOS for real-time task scheduling. The robot autonomously explores unknown environments, discovers frontiers (boundaries between known and unknown space), and navigates to them while avoiding static and dynamic obstacles detected.

**Course Context:**

This project was developed as part of the [**Making Intelligent Things** (CS-358)](https://edu.epfl.ch/coursebook/en/making-intelligent-things-a-CS-358-A) course at **EPFL** (École Polytechnique Fédérale de Lausanne).

**Acknowledgments:**
- Built upon the physical structrue of [**TurboSLAM**](https://github.com/epfl-cs358/2025sp-turboslam) (previous team)
- Course TAs and [DLL coaches](https://make.epfl.ch/coaches) for guidance and support

**Note on Continued Development:**

Although the CS-358 course has concluded, we continue to actively develop and improve SLAMaleykoum. For current work-in-progress features, planned enhancements, and future research directions, see [Ongoing Works & Next Steps](#ongoing-works--next-steps).

---

## Quick Jump To Detailed Documentation

For in-depth technical details, refer to the dedicated subsystem documentation:

<table>
<tr>
<td width="33%" align="center">

### 🔧 **[Hardware: Step by Step How to build](assets/docs/hardware/ASSEMBLE_GUIDE.md)**

Assembly instructions, wiring diagrams, CAD files, component specs

</td>
<td width="33%" align="center">

### 🗺️ **[ESP-1 Controller: Mapping & Planning](include/esp1/README.md)**

LiDAR processing, Bayesian mapping, frontier detection, A* path planning

</td>
<td width="33%" align="center">

### 🎮 **[ESP-2 Operator: Localization & Driving](include/esp2/README.md)**

Sensor fusion, Pure Pursuit control, motor management, safety systems

</td>
</tr>
</table>

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Quick Start](#quick-start)
3. [Hardware](#hardware)
4. [System Architecture](#system-architecture)
5. [Software Setup](#software-setup)
6. [Configuration & Tuning](#configuration--tuning)
7. [Documentation Index](#documentation-index)
8. [Archives](#archives)
9. [Ongoing Works & Next Steps](#ongoing-works--next-steps)
10. [Credits](#credits)
11. [Conclusion](#conclusion)

---

## Project Overview

### Vision

SLAMaleykoum is an autonomous robotics platform that transforms a standard RC car chassis into an intelligent mobile robot capable of mapping and navigating unknown environments. The system performs full Simultaneous Localization and Mapping (SLAM), enabling the robot to autonomously explore, build a map, localize itself with high accuracy in real time, navigate to target coordinates, dynamically avoid obstacles, and maintain a reliable estimate of its position throughout the mission.

Here is our original [Project proposal](https://www.overleaf.com/9942875199zgzbkrmgkkkj#3fbbb2) for our SLAM Car. Please note that it represents the initial version of the project. Since then, many aspects have evolved and changed throughout development, so the proposal should be considered an early-stage reference rather than the final design.

### How the Robot "Thinks"

The robot operates through a continuous perception-planning-action loop distributed across two processors:

**High-Level Decision Making (ESP-1 Controller):**
1. **Perceive** - Process LiDAR scans to update the occupancy grid map
2. **Analyze** - Identify frontiers (boundaries between explored and unexplored areas)
3. **Decide** - Select the closest explorable frontier as the next goal
4. **Plan** - Compute a collision-free path to the goal using A* search
5. **Command** - Send path waypoints to the Operator (ESP-2)

**Why this approach?** Frontier-based exploration naturally drives the robot to map the entire environment without human intervention. The robot is "curious" - it always seeks to explore what it doesn't know yet.

**Low-Level Execution (ESP-2 Operator):**
1. **Localize** - Continuously estimate position by fusing encoder and IMU data
2. **Track** - Follow the commanded path using Pure Pursuit controller
3. **React** - Monitor ultrasonic sensor for obstacles; emergency stop if needed
4. **Report** - Send updated position back to Controller for map correction

**Why this split?** The Controller handles computationally expensive tasks (mapping, pathfinding) at lower rates (1-10 Hz), while the Operator focuses on time-critical control loops at high rates (50-100 Hz). This separation ensures smooth, responsive driving even while planning is ongoing.

### Technical Vocabulary

- **ESP** (ESP32-S3): Microcontroller with dual-core processor and WiFi
- **ESC** (Electronic Speed Controller): Motor driver that converts PWM signals to motor power
- **LiDAR** (Light Detection and Ranging): Laser-based distance sensor for environment scanning
- **IMU** (Inertial Measurement Unit): Sensor measuring orientation and acceleration
- **I2C** (Inter-Integrated Circuit): Serial communication protocol for sensors
- **UART** (Universal Asynchronous Receiver-Transmitter): Serial communication between ESPs
- **TCP** (Transmission Control Protocol): Network protocol used for ground station communication
- **GS** (Ground Station): External computer for visualization (monitoring only)

### Key Objectives

- **Autonomous Mapping**: Create detailed environment maps using LiDAR sensor data
- **Robust Localization**: Maintain accurate position estimation through sensor fusion
- **Intelligent Navigation**: Plan and execute collision-free paths to destinations
- **Real-time Control**: Execute precise motor control for path following
- **Remote Monitoring**: Provide GS visualization

> **Important**: The car should be able to operate fully autonomously → with **no** GS.
> No computations nor storage will be offloaded to the GS.

### Core Technologies

- **Hardware**: Dual ESP32-S3 microcontrollers, LiDAR sensor, wheel encoders, IMU
- **Operating System**: [FreeRTOS](https://www.freertos.org/) for real-time multitasking and priority-based scheduling
- **Communication**: UART (inter-processor), TCP (ground station)
- **Algorithms**: 
  - [A* pathfinding](https://en.wikipedia.org/wiki/A*_search_algorithm) - Optimal graph search algorithm
  - [Pure Pursuit](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) controller - Path tracking for car-like robots
  - [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm) - Efficient grid ray tracing
  - [BFS](https://en.wikipedia.org/wiki/Breadth-first_search) (Breadth-First Search) - Frontier clustering
  - [Bayesian occupancy grid mapping](https://ieeexplore.ieee.org/document/30720) - Probabilistic environment representation

### System at a Glance

```
┌─────────────────────────────────────────────────────────┐
│  ESP-1 CONTROLLER: Mapping & Planning (1-10 Hz)         │
│  • LiDAR processing & Bayesian mapping                  │
│  • Frontier detection & clustering                      │
│  • A* global path planning                              │
│  • Ground station telemetry (WiFi/TCP)                  │
└─────────────────────────────────────────────────────────┘
                         ↕ UART
┌─────────────────────────────────────────────────────────┐
│  ESP-2 OPERATOR: Localization & Driving (50-100 Hz)     │
│  • Sensor fusion (encoder + IMU)                        │
│  • Pure Pursuit path following                          │
│  • Motor & steering control                             │
│  • Emergency obstacle handling                          │
└─────────────────────────────────────────────────────────┘
```

---

## Quick Start

### Prerequisites Checklist

**Hardware:**
- Assembled robot (see [Hardware section](#hardware))
- 9.0V NiMH battery (charged)
- USB-C cables for flashing ESP32s
- Computer with WiFi capability (for visualization)

**Software:**
- [Python 3.8+](https://www.python.org/downloads/)
- [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html) or VS Code extension ([install guide](https://docs.platformio.org/en/latest/core/index.html))

### Flash & Run

**Step 0: Clone the Repository**
```bash
git clone https://github.com/epfl-cs358/2025fa-SLAMaleykoum.git
cd 2025fa-SLAMaleykoum
```

**Step 1: Install Ground Station Dependencies**
```bash
# Create virtual environment
python3 -m venv slamaleykoum_venv

# Activate environment
source slamaleykoum_venv/bin/activate  # On Windows: slamaleykoum_venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

**Step 2: Flash ESP32 Firmware**

This project uses two ESP32s. You must flash them individually.

```bash
# In VS Code with PlatformIO extension:
# 1. Connect ESP1 via USB (use RIGHT port - left port may not work)
# 2. Open PlatformIO extension (alien icon)
# 3. Click esp1/General/Upload
# 4. Wait for upload to complete
# 5. Repeat for ESP2: Connect ESP2 → esp2/General/Upload
```

**Step 3: Power Up & Connect**

1. **Power sequence:**
   ```
   Battery → ESC → Turn ESC switch ON
   ```
   ⚠️ **WARNING**: When disconnecting, first turn the ESC off, then disconnect the battery.

2. **Connect to robot WiFi:**
   - **SSID:** `LIDAR_AP`
   - **Password:** `l1darpass`

3. **Launch ground station:**
_Note: Further documentation can be found [here](assets/docs/GROUND_STATION.md)_
   ```bash
   # Ensure your venv is active
   source slamaleykoum_venv/bin/activate
   
   # Run the interface
   python ground_station.py
   ```
   A *Pygame* window will appear. Press **START** to begin logging data and visualizing the real-time LIDAR feed.

**What to expect:**
- Ground station shows live LiDAR scan
- Robot begins mapping and moving toward frontiers
- Map builds progressively as robot explores

**First-time issues?** → Check out [Troubleshooting guide](assets/docs/troubleshooting.md)

> *Dev Note: To change WiFi credentials, modify [`src/esp1/main_esp1.cpp`](src/esp1/main_esp1.cpp) before flashing.*

---

## Hardware

### Component List

| Component                              | Reference Links                                                                                                                                                                                                 | Price (CHF) |
|----------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|
| Tamiya Blitzer Beetle                  | [Manual](https://www.tamiyausa.com/media/files/58502ml-829-5367.pdf)                                                                                                                                             | 129.00      |
| RPLIDAR C1                             | [Datasheet](https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf), [SDK](https://github.com/Slamtec/rplidar_sdk), [Wiki](https://www.waveshare.com/wiki/RPLIDAR_C1)              | 79.90       |
| ESP32-S3-WROOM-1 Microcontroller (x2)  | [Datasheet](https://cdn-shop.adafruit.com/product-files/5477/esp32-s3_datasheet_en.pdf)                                                                                                                         | 50.90       |
| DMS15 Servo                            | [Wiki](https://wiki.dfrobot.com/DSS-M15S_270%C2%B0_15KG_DF_Metal_Servo_with_Analog_Feedback_SKU__SER0044)                                                                                                         | 5.00        |
| BNO086 IMU                             | [Datasheet](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080_085-Datasheet_v1.16.pdf)                                                                      | 19.90       |
| AS5600 Encoder                         | [Datasheet](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf)                                          | 12.90       |
| HC-SR04 Ultrasonic Sensor              | [Datasheet](https://handsontec.com/dataspecs/sensor/SR-04-Ultrasonic.pdf)                                                                                                                                       | 2.00        |
| 540J Motor                             | [Datasheet](https://asset.conrad.com/media10/add/160267/c1/-/en/001385115DS01/adatlap-1385115-540-es-motor-reely-532114c.pdf)                                                                                   | —           |
| THW-1060-RTR ESC                       | [Datasheet](https://www.hobbywing.com/en/uploads/file/20221015/f60b7ebe160a7b283927ae8916d36763.pdf)                                                                                                            | —           |
| LM2596 Buck Converter                  | [Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)                                                                                                                                                       | 5.90           |
| 7.2V Battery                           | [Product Page](https://www.galaxus.ch/fr/s5/product/gens-ace-modelisme-dune-batterie-720-v-5000-mah-batterie-rc-9459930)                                                                                        | 32.90       |
| **Total Cost**                         |                                                                                                                                                                                                                 | **338.4**  |

**Additional materials needed:**
- $3 \times 1 K\Omega$ resistors (for the ultrasonic sensor voltage divider)
- Assorted jumper cables and connectors (male/female)
- Heat-shrink tubing or insulation sleeves
- Soldering kit (soldering iron, solder wire)
- Hot air/heat gun (for shrinking tubing)
- Screw set (M3 and M6 as used in mounts)

### Assembly Overview
Clicky link: [Full Step-by-step Assembly guide here](/assets/docs/hardware/ASSEMBLE_GUIDE.md)

The robot is built on a Tamiya Blitzer Beetle chassis with custom 3D-printed mounts for the LiDAR, ESP32 boards, encoder, ultrasonic sensor, and battery. Assembly is straightforward: build the base chassis, install the encoder and magnet on the motor shaft, mount the electronics platform, attach the sensors, and finish with wiring.

**Assembly phases:**
1. **Chassis** - Build base chassis & install motor/encoder
2. **3D-printed mounts** - Install LiDAR platform, ESP32 holders, sensor brackets
3. **Electronics mounting** - Secure ESPs, buck converter, IMU
4. **Wiring** - Follow electrical diagram, install voltage divider
5. **Final assembly** - Install battery, route cables, verify connections

**📘 Full step-by-step assembly guide (with photos, CAD files, wiring diagrams):**  
→ [**Complete Assembly Instructions**](/assets/docs/hardware/ASSEMBLE_GUIDE.md)

### Wiring & Electrical

Clicky link: [Full wiring explanation HERE](/assets/docs/hardware/circuit/README.md)

**Electrical Diagram:**

<p align="center">
  <img src="/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png" alt="Electrical Circuit" width="1000"/>
</p>

**⚠️ Important Wiring Notes (Do Not Skip)**

- **Common Ground**: All components (ESP32s, ESC, sensors, servo) must share a common ground from the buck converter.

- **5V vs 3.3V Power**:
  - ESP32 boards, ultrasonic sensor, lidar, and servo are powered from 5V (buck converter)
  - IMU and encoder are powered from ESP2's 3.3V pin

- **Ultrasonic Sensor (HC-SR04) → Voltage Divider Required**
  - The HC-SR04 ECHO pin outputs 5V
  - ESP32 GPIOs are 3.3V-only
  - A resistor voltage divider is mandatory on the ECHO line
  - Connecting ECHO directly to the ESP32 will damage the GPIO

- **ESP-to-ESP Communication**: UART cross-connection is required (TX ↔ RX between ESP1 and ESP2).

**📘 Detailed Wiring & Soldering Guide:**  
→ [**Full wiring explanation**](/assets/docs/hardware/circuit/README.md)

### CAD Files

You can explore all STL files directly in the [**CAD folder**](/assets/CAD).

Here is a quick video of the car's body and roof: [**CAD Video**](/assets/docs/hardware/CAD/fast_cad_video.mp4)

<p align="center"><img src="assets/Demo/CAD_NEW_BODY.gif" width=700></p>

**Note:** The CAD files for the **front bumper**, **ultrasonic sensor case**, and **encoder mount** were originally designed by the group from whom we inherited the car, [**TurboSLAM**](https://github.com/epfl-cs358/2025sp-turboslam).

**Before implementing the hardware setup**, check the [Problems and Recommendations](assets/docs/troubleshooting.md) section (Hardware parts) to get a full scope of what may need to be modified.

---

## System Architecture

The computational load is divided between two ESP32-S3 microcontrollers with clearly defined roles:

- **ESP-1 (Controller)**: High-level perception and planning
- **ESP-2 (Operator)**: Low-level control and execution

We use **FreeRTOS** to enable parallel task execution with priority-based scheduling, ensuring time-critical control loops run reliably while computationally expensive planning tasks run in the background. Inter-processor communication happens via **UART** serial links.

**Rationale:**

1. **Performance**: Each processor focuses on its task without competing for resources
2. **Modularity**: Components are well-isolated, making testing and debugging easier
3. **Reliability**: Control loop continues operating even if global planning encounters issues
4. **Efficiency**: Only essential data is sent from one ESP to the other, limiting information flow
5. **Parallelism**: FreeRTOS allows multiple tasks with different priorities to run concurrently on each dual-core ESP32

*Note: For each ESP we've got a dedicated README file delving into more details. They can be found in [`include/esp1/README.md`](include/esp1/README.md) and [`include/esp2/README.md`](include/esp2/README.md)*

### Architecture Diagrams

**ESP-1 System Diagram:**
![ESP-1 Architecture](assets/docs/esp1/esp1-diagram.png)

**ESP-2 System Diagram:**
![ESP-2 Architecture](assets/docs/esp2/esp2-diagram.png)

### ESP-1 Controller: Mapping & Planning

🦄 **[Full ESP-1 Documentation →](include/esp1/README.md)**

**Primary Mission**: Create and maintain a global understanding of the environment and plan high-level navigation strategies.

**Core Responsibilities**:
- Generate and maintain occupancy grid maps using Bayesian updates
- Find the frontier cells to explore and cluster them using BFS algorithms
- Define a temporary goal for the car to reach
- Plan global paths using A* algorithm
- Share telemetry with ground station via WiFi (visualization purposes)
- Send the computed path to ESP-2

**Key Components:**
- **LiDAR Processing**: Extract and convert RPLIDARC1 data into readable scans
- **Bayesian Grid Mapping**: Build persistent 2D occupancy grid from LiDAR data
- **Mission Planner**: Select exploration goals based on frontier detection
- **Global Planner (A*)**: Compute collision-free paths on the grid

**Operating Frequency**: 1-10 Hz

### ESP-2 Operator: Localization & Motion Control

🦄 **[Full ESP-2 Documentation →](include/esp2/README.md)**

**Primary Mission**: Execute precise vehicle control and maintain high-frequency local position tracking.

**Core Responsibilities**:
- Execute Pure Pursuit path following algorithm
- Handle emergency stop commands and maneuvers to escape when stuck
- Aggregate sensor data (odometry, IMU) to track the position of the car and forward to ESP-1

**Key Components:**
- **Sensor Fusion**: Combine encoder + IMU for pose estimation
- **Pure Pursuit Controller**: Local path-tracking with bicycle model
- **Motor Control**: ESC PWM control with emergency stop
- **Safety System**: Ultrasonic-based obstacle detection
- **Recovery Maneuvers**: Automated escape sequences when blocked

**Operating Frequency**: 50-100 Hz

### Communication Architecture

#### Inter-ESP Communication (UART)

**Protocol**: Custom packet-based with checksums

**Data Flow:**

| Direction | Message Type | Content |
|-----------|--------------|---------|
| ESP-2 → ESP-1 | `Pose2D` | Current car's position (x, y, θ) |
| ESP-1 → ESP-2 | `PathMessage` | Global path waypoints |

👉 **[Full ESP Link Protocol Documentation →](include/common/esp_link_readme.md)**

#### Ground Station Communication (WiFi/TCP)
🦄 **[Full GS Documentation →](include/esp2/README.md)**

**WiFi Mode**: Access Point (AP)
```
SSID:     LIDAR_AP
Password: l1darpass
```

**ESP-1 → Ground Station**:
- Real-time map visualization data
- Mission status: goal and state
- Robot pose and trajectory
- Telemetry logs

The ESP1 creates a Wi-Fi access point (AP) that we connect to for **monitoring purposes only**. We used MQTT at first to get feedback during tests, but quickly switched to TCP, as it supports higher data throughput.
We additionally log the CPU profiling for the esp1 for debugging pourpuses. Detailed information: [CPU Profiling Documentaiton](assets/docs/debugging/CPU_PROFILING.md)

If you want to establish the same MQTT connection to debug, here is the guide to follow: [**WiFi and MQTT connection guide**](/assets/docs/WiFi_and_MQTT_Connection_Guide___SLAMaleykoum.pdf).

### Shared Mechanisms (`common/`)

- `data_types.h`: Definitions of shared data structures
- `esp_link.h`: The communication system from one ESP to the other. [→ ESP Link README](include/common/esp_link_readme.md)
- `transforms.h`: Functions that transform the position of the robot to the grid and the other way around. [→ Transforms README](include/common/transforms_readme.md)
- `utils.h`: Regroups the functions used across different files to avoid redefining it.
- `wifi_connection.h`: Protocol to connect the WiFi for MQTT

---

## Software Setup

### Development Environment

**Prerequisites:**
* [Python 3.8+](https://www.python.org/downloads/)
* [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html) or VS Code extension

### Project Structure
```
SLAMaleykoum/
├── include/                    # Header files
│   ├── common/                 # Shared utilities & communication
│   ├── esp1/                   # Mapping & Planning headers
│   └── esp2/                   # Localization & Control headers
│
├── src/                        # Implementation files
│   ├── common/                 # Shared utilities
│   ├── esp1/                   # ESP-1 source code
│   └── esp2/                   # ESP-2 source code
│
├── assets/                     # Documentation & media
│   ├── Demo/                   # Demo videos
│   ├── Images/                 # Photos & diagrams
│   └── docs/                   # Technical documentation
│       ├── hardware/           # Assembly guide, CAD files, wiring
│       ├── esp1/               # Architecture diagrams
│       └── esp2/               # Architecture diagrams
│
├── archives/                   # Historical deprecated implementations
│
├── README.md
├── requirements.txt            # Set up python requirements
├── ground_station.py           # Visualization & telemetry
├── platformio.ini              # Build configuration
└── boards/                     # Custom board definitions
```

**Key Directories:**
- `include/` + `src/` - Dual-ESP codebase organized by subsystem
- `assets/` - All documentation, images, and demo videos
- `archives/` - Previous implementations and test code

### Ground Station (PC)

The ground station script runs on your computer to visualize telemetry.  
*Note: We recommend using a virtual environment.*

Install the dependencies from our `requirements.txt` file.

```bash
# Create virtual environment
python3 -m venv slamaleykoum_venv

# Activate environment
source slamaleykoum_venv/bin/activate  # On Windows: slamaleykoum_venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Firmware (ESP32s)

This project uses two ESP32s. You must flash them individually.

**Upload Code:**

Open in PlatformIO: Open the project folder in VS Code.

1. Connect to the ESP1 via USB.
2. Open the *platformio* extension in VS Code (the alien icon).
3. Click on `esp1/General/Upload`.
4. Repeat for the ESP2: Connect ESP2 → `esp2/General/Upload`.

**⚠️ Important:** Use the **RIGHT USB port** on each ESP32. Left ports don't seem to work for programming.
*Note: In our design, the esp1 is upside down, so the right port is the one on the left looking from the outside of the car.*

### WiFi Configuration

**Default credentials** (edit before flashing if needed):
```cpp
#define WIFI_SSID "LIDAR_AP"
#define WIFI_PASSWORD "l1darpass"
```

---

## Configuration & Tuning

### Map Parameters

Configure the mapping grid size and resolution:

```cpp
// src/esp1/main_esp1.cpp
#define GRID_SIZE_X 70      // cells
#define GRID_SIZE_Y 70      // cells
#define RESOLUTION 0.2      // meters/cell <- This is the most important parameter.
                            // It directly affects the size of the grid.
```

**Physical map size:** 14m × 14m (with 0.2m resolution)
**Memory usage:** 70 × 70 × 1 byte = 4.9 KB

**⚠️ Important:** Grid dimensions **must match** in [`ground_station.py`](ground_station.py):
```python
GRID_SIZE_X = 70
GRID_SIZE_Y = 70
RESOLUTION = 0.2
```

**Trade-offs:**
- Larger `RESOLUTION` → bigger mapping area → less detail
- Smaller `RESOLUTION` → smaller mapping area → more detail
- Larger grid → more detail, more memory, slower updates
- Smaller grid → less detail, faster, but limited range

### Control Parameters

```cpp
// include/esp2/control/pure_pursuit.h
#define LOOKAHEAD_DISTANCE 0.3   // meters
#define TARGET_SPEED 0.2         // m/s
#define WAYPOINT_THRESHOLD 0.1   // meters
#define MAX_STEERING_ANGLE 30    // degrees
```

**Tuning guide:**
- `LOOKAHEAD_DISTANCE`:
  - Too small → oscillation, sharp turns
  - Too large → cuts corners, misses waypoints
- `TARGET_SPEED`:
  - Higher → faster exploration, worse odometry, worse mapping
  - Lower → accurate mapping, slow mission

### Sensor Parameters

```cpp
// include/esp2/hardware/encoder.h
#define ENCODER_TICKS_PER_REV 4096
#define GEAR_RATIO 10.0
#define WHEEL_RADIUS 0.032       // meters

// include/esp2/hardware/imu.h
#define IMU_SAMPLE_RATE 100      // Hz
#define YAW_STABILITY_THRESHOLD 15  // degrees

// include/esp2/hardware/ultrasonic.h
#define EMERGENCY_THRESHOLD 0.2  // meters
#define ULTRASONIC_TIMEOUT 30    // ms
```

### FreeRTOS Task Priorities

```cpp
// ESP-1
Lidar_Read_Task:       Priority 4, Core 1, Stack 3072
Lidar_Sync_Map_Task:   Priority 4, Core 1, Stack 8192
Bayesian_Grid_Task:    Priority 3, Core 1, Stack 20480
IPC_Receive_Task:      Priority 3, Core 0, Stack 2048
TCP_Transmit_Task:     Priority 1, Core 0, Stack 8192
Mission_Planner_Task:  Priority 2, Core 0, Stack 4096
Global_Planner_Task:   Priority 2, Core 0, Stack 8192

// ESP-2
TaskReceivePath:       Priority 3, Core 0, Stack 6144
TaskOdometryUnified:   Priority 3, Core 0, Stack 10240
TaskMotor:             Priority 2, Core 1, Stack 6144
TaskUltrasonic:        Priority 2, Core 1, Stack 6144
TaskPurePursuit:       Priority 1, Core 1, Stack 6144
```

---

## Documentation Index

### Core System Documentation

**Main System:**
- [Main README](README.md) - This document
- [Project Proposal](https://www.overleaf.com/9942875199zgzbkrmgkkkj#3fbbb2) - Original project vision

**ESP Subsystems:**
- [ESP-1: Mapping & Planning](include/esp1/README.md)
- [ESP-2: Localization & Control](include/esp2/README.md)

### Software Documentation
- [ESP-1: Mapping & Planning](include/esp1/README.md)
- [ESP-2: Localization & Control](include/esp2/README.md)
- [Ground station](assets/docs/GROUND_STATION.md)
- [Pure Pursuit Controller](include/esp2/control/pure_pursuit_readme.md)

### Hardware Documentation

**Assembly & Build:**
- [Complete Assembly Guide](/assets/docs/hardware/ASSEMBLE_GUIDE.md)
- [Wiring & Circuit Details](/assets/docs/hardware/circuit/README.md)

**Shared Utilities:**
- [ESP Link Protocol](include/common/esp_link_readme.md)
- [Coordinate Transforms](include/common/transforms_readme.md)

### Configuration & Setup

- [WiFi & MQTT Setup Guide](/assets/docs/WiFi_and_MQTT_Connection_Guide___SLAMaleykoum.pdf)

### Debugging & Development Tools

- [CPU Usage Profiling Guide](/assets/docs/debugging/CPU_PROFILING.md)
- [Troubleshooting Guide](assets/docs/troubleshooting.md)
---

## Archives

The purpose of this directory is to save our previous work, and keep track of the tests we made.

### Previous Work

We tried many things that were eventually not used. These can be found in the subdirectories of `esp1` and `esp2`.

#### `esp1`
- `rpLidar` (and `rpLidarTypes`) contains the full KKest library as found on GitHub. While we kept the same underlying implementation, we simplified it in our codebase to reduce memory usage. The original version is kept here for reference.

#### `esp2`
- `motor_pid` was ultimately not functional due to the poor quality of the ESC we used.
- `ekf_localizer` was not used because the IMU was not accurate enough for acceleration_y and acceleration_x so the computation of the position using IMU was unreliable.
- `pure_pursuit` is used in the project, but the archived files contain earlier implementations based on EKF and PID control. We kept the full version here for reference, while a cleaned and correct implementation is used in the main code.
- `recovery_maneuver` turned the car toward the first waypoint of a path. Not used due to delays with new paths; current version does an automatic 180° turn for reliability. We kept this improved version in the archive.

### The Tests

Some tests are no longer directly runnable without modifications, as data types evolved over time and the include paths differ from the current project structure.

There is one test directory per ESP, containing all the tests developed throughout the project. Some tests target specific components, while others exercise the full system.

To avoid redefining the same constants and parameters in every test, two shared files are used:
`test_common_esp<i>` contains constants and the prototypes of the setup and loop functions, and `test_globals_esp<i>` defines and initializes global variables.

Each test entry point is implemented in `test_main_esp<i>`, which simply includes these shared files. In the setup and loop functions of the test main, the test to run is selected via a test ID.

The Python files are used for TCP monitoring and for displaying the received data in a structured format.

### Development & Debugging Tools

During development, we created custom CPU profiling tools to analyze FreeRTOS task performance:
- Task execution timeline tracking
- CPU usage per core
- Mutex contention analysis

See [CPU Profiling README](assets/docs/debugging/CPU_PROFILING.md) for implementation details.

---

## Ongoing Works & Next Steps

This project continues to evolve beyond the course timeline (Automn Semester 2025). Below are the improvements and features currently in development or planned for future implementation.

### Improvements

**Hardware Upgrades:**
- Replace THW-1060-RTR ESC with IBT-4 (BTS7960) motor driver for smooth PWM control and enable PID velocity control
- Replace current wireing with a proper printed circuit board (PCB)

**Software Enhancements:**
- Switch from TCP to UDP since we don't care about dropping packets
- Test the PID velocity controller (currently blocked by ESC limitations)
- Improvements of the Path Planning algorithm to take into account non-holonomic constraints
- Improve the Goal Definition to better handle complicated goals to reach
- Add drift correction using LiDAR scan matching (Something similar to [this](https://static.googleusercontent.com/media/research.google.com/en//pubs/archive/45466.pdf))
- Optimize Bayesian grid update performance with [Single Instruction, Multiple Data (SIMD)](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data) operations
- Test adaptive lookahead distance in Pure Pursuit based on current speed

**System Robustness:**
- Implement formal state machine for mission control
- Resume development of the CPU analysis/testing
- Develop comprehensive unit test suite with hardware-in-the-loop testing
- Battery voltage monitoring with low-power warnings

### Known Issues to Address 🐛

- Odometry drift due to encoder noise and IMU drift
- Occasional A* failures in highly cluttered environments


**Current Focus**: Replacing the ESC and implementing PID velocity control (expected completion: February 2026)

---

## Credits

#### Project Team

SLAMaleykoum was develeped as part of the [**Making Intelligent Things** (CS-358)](https://edu.epfl.ch/coursebook/en/making-intelligent-things-a-CS-358-A) course at **EPFL** (École Polytechnique Fédérale de Lausanne). Building on the foundation laid by the [**TurboSLAM**](https://github.com/epfl-cs358/2025sp-turboslam) team who designed the original electrical circuit, encoder mount, and ultrasonic sensor case.

#### Use of AI Tools in Development

This project was developed with the assistance of AI-powered development tools, in accordance with EPFL's academic integrity guidelines.

**AI Tools Used:**
- **Claude (Anthropic)** & **Gemini Pro (Google)**: Code debugging, algorithm optimization, and technical documentation

**Scope of AI Assistance:**

**AI Assisted With:**
- Optimizing Bayesian grid updates and A* search
- Code review and identifying edge cases
- Structuring documentation and improving technical writing
- Troubleshooting hardware integration issues (encoder noise, IMU drift, ESC limitations)

**AI Did NOT:**
- Define project goals or system architecture
- Make engineering decisions or trade-off evaluations
- Perform hardware assembly, wiring, or physical testing
- Collect experimental data or validate performance in real-world conditions

**Validation:** All AI-generated code was manually reviewed, tested on actual hardware, and adapted to meet project-specific constraints.

**Team Contribution:** Core system design, algorithmic implementation, hardware integration, and experimental validation were performed by the project team. AI tools accelerated development but **did not** replace the hands-on engineering work, domain expertise, and hundreds of hours of testing and debugging required to build a working autonomous robot.

#### Key Technologies & Libraries

- **FreeRTOS** - Real-time operating system for multitasking
- **Rob Tillaart's AS5600 library** - Magnetic encoder reading
- **RPLIDAR C1 SDK** - Slamtec's LiDAR interface
- **PlatformIO** - Development platform for ESP32
- **Python + Pygame** - Ground station visualization

#### Hardware Credits

- **Tamiya Blitzer Beetle** - RC car chassis platform
- **TurboSLAM Team** - Original design of the electrical circuit, and original CAD designs for bumper, ultrasonic case, and encoder mount

---

## Conclusion

SLAMaleykoum demonstrates the transformation of a standard RC car into a fully autonomous robot capable of real-time mapping, localization, and navigation. By distributing computation across two ESP32-S3 microcontrollers and leveraging FreeRTOS, the system achieves a robust, modular, and fully self-contained architecture.

The project involved addressing multiple hardware and software challenges, leading to practical design choices and algorithmic optimizations suited to embedded constraints. Overall, SLAMaleykoum provides a solid foundation for future improvements and stands as a complete and extensible SLAM platform for autonomous robotics on resource-limited hardware.

---

**Questions? Issues? Feedback?**

For technical support or to report issues, please refer to the [Troubleshooting](assets/docs/troubleshooting.md) section first. For further assistance, consult the detailed component documentation in the [Documentation Index](#documentation-index).

_For any further questions or feedback, contact santiago.silva-carrillo@epfl.ch_