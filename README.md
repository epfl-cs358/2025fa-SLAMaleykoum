
# SLAMaleykoum

## Table of Contents
1. [Project Overview](#project-overview)
2. [Hardware Overview](#hardware-overview)
3. [System Architecture](#system-architecture)
4. [Hardware Platform](#hardware-platform)
5. [Software Components](#software-components)
6. [Data Flow and Communication](#data-flow-and-communication)
7. [Implementation Details](#implementation-details)
8. [Mission and Task Management](#mission-and-task-management)
9. [Set it up](#set-it-up)


---

## Project Overview

### Vision
SLAMaleykoum is an autonomous robotics platform that transforms a standard RC car chassis into an intelligent mobile robot capable of mapping and navigating unknown environments. The system performs full Simultaneous Localization and Mapping (SLAM), enabling the robot to autonomously explore, build a map, localize itself with high accuracy in real time, navigate to target coordinates, dynamically avoid obstacles, and maintain a reliable estimate of its position throughout the mission.

Here is our original [Project proposal](https://www.overleaf.com/9942875199zgzbkrmgkkkj#3fbbb2) for our SLAM Car. Please note that it represents the initial version of the project. Since then, many aspects have evolved and changed throughout development, so the proposal should be considered an early-stage reference rather than the final design.


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

### Component List

| Component                     | Reference Links                                                                                                                                                                                                 | Buy Link                                                                                                                                                                                                 | Price (CHF) |
|-------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|
| Tamiya Blitzer Beetle         | [Manual](https://www.tamiyausa.com/media/files/58502ml-829-5367.pdf)                                                                                                                                             | [Galaxus](https://www.galaxus.ch/fr/s5/product/tamiya-blitzer-beetle-kit-rc-vehicle-12900000)                                                                                                           | 129.00      |
| RPLIDAR C1                    | [Datasheet](https://d229kd5ey79jzj.cloudfront.net/3157/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf), [SDK](https://github.com/Slamtec/rplidar_sdk), [Wiki](https://www.waveshare.com/wiki/RPLIDAR_C1)              | [Bastelgarage](https://www.bastelgarage.ch/rplidar-c1-12m-scanner-lidar-laser-360-degres)                                                                                                               | 79.90       |
| ESP32-S3-WROOM-1 Microcontroller (x2) | [Datasheet](https://cdn-shop.adafruit.com/product-files/5477/esp32-s3_datasheet_en.pdf)                                                                                                                        | [Galaxus](https://www.galaxus.ch/en/s1/product/espressif-esp32-s3-devkitm-1-n8-development-board-development-boards-kits-24505914)                                                                     | 25.45 ×2    |
| DMS15 Servo                   | [Wiki](https://wiki.dfrobot.com/DSS-M15S_270%C2%B0_15KG_DF_Metal_Servo_with_Analog_Feedback_SKU__SER0044)                                                                                                      | —                                                                                                                                                                                                       | 5.00           |
| BNO086 IMU                    | [Datasheet](https://docs.sparkfun.com/SparkFun_VR_IMU_Breakout_BNO086_QWIIC/assets/component_documentation/BNO080_085-Datasheet_v1.16.pdf)                                                                     | [Bastelgarage](https://www.bastelgarage.ch/bno055-capteur-intelligent-a-9-axes?search=9dof)                                                                                                             | 19.90       |
| AS5600 Encoder                | [Datasheet](https://files.seeedstudio.com/wiki/Grove-12-bit-Magnetic-Rotary-Position-Sensor-AS5600/res/Magnetic%20Rotary%20Position%20Sensor%20AS5600%20Datasheet.pdf)                                         | —                                                                                                                                                                                                       | 2.00       |
| HC-SR04 Ultrasonic sensor     | [Datasheet](https://handsontec.com/dataspecs/sensor/SR-04-Ultrasonic.pdf)                                                                                                                                    | —                                                                                                                                                                                                       | 2.00           |
| 540J Motor                    | [Datasheet](https://asset.conrad.com/media10/add/160267/c1/-/en/001385115DS01/adatlap-1385115-540-es-motor-reely-532114c.pdf)                                                                                  | —                                                                                                                                                                                                       | —           |
| THW-1060-RTR ESC              | [Datasheet](https://www.hobbywing.com/en/uploads/file/20221015/f60b7ebe160a7b283927ae8916d36763.pdf)                                                                                                           | —                                                                                                                                                                                                       | —           |
| LM2596 Buck converter         | [Datasheet](https://www.ti.com/lit/ds/symlink/lm2596.pdf)                                                                                                                                                      | —                                                                                                                                                                                                       | —           |
| 7.2V Battery                  | [Product Page](https://www.galaxus.ch/fr/s5/product/gens-ace-modelisme-dune-batterie-720-v-5000-mah-batterie-rc-9459930)                                                                                       | [Galaxus](https://www.galaxus.ch/fr/s5/product/gens-ace-modelisme-dune-batterie-720-v-5000-mah-batterie-rc-9459930)                                                                                      | 32.90       |
| **Total Cost**                |                                                                                                                                                                                                                |                                                                                                                                                                                                         | **321.6**   |


Alongside the listed components, you will also need:
- 3 × 1 kΩ resistors (for the ultrasonic sensor voltage divider)
- Assorted jumper cables and connectors (male/female)
- Heat‑shrink tubing or insulation sleeves
- Soldering kit (soldering iron, solder wire)
- Hot air/heat gun (for shrinking tubing)
- Screw set (M3 and M6 as used in mounts)
### How to Assemble


To begin, assemble the mechanical base of the car following the official [Tamiya Blitzer Beetle manual](https://www.tamiyausa.com/media/files/58502ml-829-5367.pdf).
We only need the minimal mechanical build so the car can drive, skip the decorative carcass.

**Step 1: Build the Car**  
- Assemble the chassis according to the manual.  
- Mount the steering servo inside the chassis.  
- Ensure the servo cables are routed through the rectangular opening so they can later connect to the ESP.  

**Step 2: Encoder Mount + Magnet**  
- 3D‑print the encoder mount (CAD file available in /assets/CAD/).  
- Attach the mount in front of the motor.  
- Fix the encoder magnet directly onto the motor shaft as shown in the provided image.  
- This ensures accurate rotation measurement.  
- Then screw down the encoder on its mount.

**Step 3: Layer Platform (ESP + Lidar Holder)**  
- Place the 3D‑printed platform on the chassis.  
- This platform integrates holders for both ESP boards and the lidar.  
- Screw the lidar onto its holder first (space is tight, so it’s easier to mount before other components).  
- Then screw down the buck converter and IMU onto their designated spots.  

**Step 4: Mount the ESP Boards**  
- Place ESP1 and ESP2 onto the platform.  
- ⚠️ Important: mount both facing **upwards** so their LEDs are visible; this makes debugging much easier.  
- Our current ESP1 faces down, which hides the LEDs and complicates troubleshooting.  

**Step 5: Ultrasonic Sensor Mount**  
- Attach the front 3D‑printed piece designed for the ultrasonic sensor at the front of the car.  
- Screw the sensor securely into the mount.  

**Step 6: Final Assembly**  
- Place the hood/top of the car back on.  
- Insert the battery into its dedicated slot the hood design holds it firmly and stabilizes it during motion.  
- Double‑check that all mounts are secure and cables are routed cleanly for wiring.  

**Step 7: Wiring**  
- Once the mechanical build is complete, proceed to the [Soldering & Wiring](#soldering--wiring) section.  
- Follow the circuit [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png) for all connections.  

*TODO: For each step, insert assembly photos so readers can clearly see component placement and orientation.*
---

### Soldering & Wiring

To build the robot from scratch, each component must be soldered and wired according to the electrical diagram.  
Use heat shrink tubing to insulate exposed connections and ensure long-term reliability.

**Electrical Diagram**

<p align="center">
  <img src="/assets/circuit/slamaleykoum_electrical_circuit.drawio.png" alt="Electrical Circuit" width="1100"/>
</p>


**Power Connections**

- **Battery Power Split**:  
  Solder a 2‑PIN JST male connector to the 7.2V battery.  
  This battery powers the ESC and also supplies input to the buck converter.  
  Connect the black wire to the converter’s VIN (–) and the red wire to VIN (+).

<p align="center">
  <img src="\assets\Images\circuit_images\battery_buck_esc_soldering.jpeg" alt="Battery power split" width="200"/>
</p>

- **Motor Driver / ESC**:  
  Connect the battery to the ESC only after the hardware and software setup is complete.  
  Attach the red wire (+) from the battery to **BAT+** on the ESC, and the black wire (–) to **BAT–**.

- **Motor Wiring**:  
  - Connect the **yellow wire** from the ESC (M3) to the **+ terminal** of the motor (yellow).  
  - Connect the **blue wire** from the ESC (M1) to the **– terminal** of the motor (green).  

- **Buck Converter (5V Power Split):**  
  Connect the buck converter to the battery (as described above).  
  Create a single 5V harness by soldering all red 5V leads together from **OUT +**, and a ground harness by soldering all black leads together from **OUT –**.  

  From this split:
  - **5V bus (red):** one male connector (main feed), plus branches to both ESP32 boards (two female connectors), the ultrasonic sensor (male), the lidar (male), and the servo (male).  
  - **Ground bus (black):** common ground returning to **OUT –** for all the same devices. The ground from the buck converter is taken from **OUT –** via a male connector, then soldered to multiple cables that connect to the GND pins of different components:  
    - Servo (male)  
    - ESP32 board 1 (female)  
    - ESP32 board 2 (female)  
    - ESC (GND)  
    - Lidar (male)  
    - IMU (male)  
    - Encoder (male)  

- **ESP32 board (ESP1)**: Connect V5 and GND to the Buck converter as describes above.
- **ESP32 board (ESP2)**: Connect V5 and GND to the Buck converter as describes above.
- **Ultrasonic sensor**:  Connect V5 to the Buck converter as describes above. GND  connected to ground of ESP2 as digram, more soldering and modifications related to Data connection to be viewed later on. 
- **Lidar**: Connect V5 and GND to the Buck converter as describes above.

Both the encoder and the IMU are powered by the **3V3 pin of ESP2**, using two cables soldered together on the same 3V3 pin to provide a shared supply.

- **Encoder**:  
  - Use 3‑pin and 4‑pin screw terminal blocks soldered to the encoder.  
  - Power the encoder through its **VCC** pin using a cable screwed into the terminal block and connected to the **3V3 pin of ESP2** via a female connector (see [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
  - Ground (GND) is connected to the buck converter’s ground harness as described above.

<p align="center">
  <img src="/assets/Images/circuit_images/encoder_connector_mount.jpg" alt="Encoder screw terminal blocks" width="200"/>
</p>

- **IMU**:  
  - Powered from the **3V3 pin of ESP2**, using a male connector on the IMU side and a female connector to ESP2 (see [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
  - Ground (GND) is connected via a male connector on the IMU and a female connector on ESP2, both tied into the buck converter’s ground harness as described above.

#### Data Connections

- **Both ESPs**  
  - Communication via UART: connect pin 12 of ESP1 to pin 13 of ESP2, and pin 13 of ESP1 to pin 12 of ESP2 (light pink and blue cables on the [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).

- **ESP1**  
  - **Lidar**  
    - **TX**: male‑to‑female cable connected to pin 4 on ESP2 (green cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
    - **RX**: male‑to‑female cable connected to pin 5 on ESP2 (yellow cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).

- **ESP2**  
  - **Motor Driver**  
    - **PWM**: male‑to‑female cable connected to pin 15 on ESP2 (grey cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **Servo**  
    - **Pulse**: male‑to‑female cable connected to pin 6 on ESP2 (orange cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **Encoder**  
    - **DIR**: male‑to‑female cable connected to GND on ESP2 (yellow cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **IMU**  
    - **INT**: male‑to‑female cable connected to pin 4 on ESP2 (black cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **I²C Bus (shared between Encoder & IMU)**  
    - **SDA**: two male cables (encoder + IMU) soldered together into a female connector, attached to pin 8 on ESP2 (purple cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
    - **SCL**: two male cables (encoder + IMU) soldered together into a female connector, attached to pin 9 on ESP2 (green cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).
  - **Ultrasonic Sensor (HC‑SR04) — Voltage Divider on Echo Pin**  
    - The HC‑SR04 must be powered at **5V** (from the buck converter).  
    - Its **ECHO pin outputs 5V**, which is too high for the ESP32’s 3.3V GPIO.  
    - To reduce this safely, we use a **resistor voltage divider**.

    - ***Circuit Description:***  
      - **Resistors used**: three resistors, each 1 kΩ.  
      - **Ground reference**: a black cable from the sensor’s GND is soldered to two resistors in series.  
      - **Divider branch**: these two resistors connect to a blue cable, which then passes through a third 1 kΩ resistor.  
      - **Echo connection**: the third resistor is connected to the sensor’s **ECHO pin** (via male connector).  
      - **ESP32 input**: the other end of the blue cable is connected to **ESP2 pin 19** (via female connector).  
      - **Result**: the divider steps the 5V ECHO signal down to ~3.3V, safe for the ESP32.

    - ***Summary:***  
      - **Trig**: male‑to‑female cable connected directly to **ESP2 pin 5** (green cable on [diagram](/assets/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
      - **Echo**: routed through the resistor voltage divider before reaching **ESP2 pin 19** (female connector).

#### Example Layout

<p align="center">
  <img src="/assets/circuit/soldering_step1_placeholder.png" alt="Soldering Step 1" width="300"/><br>
  <b>Soldering Step 1</b>
</p>

<p align="center">
  <img src="/assets/circuit/soldering_step2_placeholder.png" alt="Soldering Step 2" width="300"/><br>
  <b>Soldering Step 2</b>
</p>


### CAD Files

You can explore all STL files directly in the [CAD folder](/assets/CAD).  

This folder contains STL files for all custom 3D‑printed parts used in the car build.  
Each part has its STL file and a preview image of the CAD design.

- **Front Bumper & Ultrasonic Sensor Case**  
  <p align="center"><img src="/assets/Images/CAD/bumber_cad_image.jpg" alt="Bumper CAD design" width="300"/></p>
  <p align="center"><img src="/assets/Images/CAD/ultrasonic_holder_cad_image.jpg" alt="Ultrasonic Sensor Holder CAD design" width="300"/></p>

- **Encoder Mount**  
  <p align="center"><img src="/assets/Images/CAD/encoder_holder_cad_image.jpg" alt="Encoder Mount CAD design" width="300"/></p>

- **Car Body Platform**  
  <p align="center"><img src="/assets/Images/CAD/body_cad_image.jpg" alt="Car Body CAD design" width="300"/></p>

- **Roof Cover**  
  <p align="center"><img src="/assets/Images/CAD/roof_cad_image.jpg" alt="Roof CAD design" width="300"/></p>

- **Body & Roof Cover (combined view)**  
  <p align="center"><img src="/assets/Images/CAD/body_roof_cad_image.jpg" alt="Body & Roof CAD design" width="300"/></p>

Note: The CAD files for the **front bumper**, **ultrasonic sensor case**, and **encoder mount** were originally designed by the group from whom we inherited the car, **TurboSLAM**.


### Challenges & Recommendations

Before implementing the hardware setup, check the [Problems and Recommendations](#problems-and-recommendations) section to get a full scope of what may need to be modified.


## System Architecture

The computational load is devided over the two ESP32-S3 microcontrollers. The first one takes care of the mapping & global planning, while the second one takes care of localization & control of the vehicle. We run **FreeRTOS** to take care of our parallelism and we are comunicating between our two ESPs via **UART** (via the serial ports).

### ESP-1: Mapping & Planning

  👉 [Open ESP-1 README](include/esp1/README.md)

**Primary Mission**: Create and maintain a global understanding of the environment and plan high-level navigation strategies.

**Operating Frequency**: 1-10 Hz (depends on task)

**Core Responsibilities**:
- Generate and maintain occupancy grid maps
- Find the frontiers to explore and cluster them using BFS algorithms
- Define a goal to reach
- Plan global paths using A* algorithm
- Share maps with ground station via wifi
- Send path to follow to esp2

### ESP-2: Localization & Control

  👉 [Open ESP-2 README](include/esp2/README.md)

**Primary Mission**: Execute precise vehicle control and maintain high-frequency local pose tracking.

**Operating Frequency**: 50-100 Hz for control loops

**Core Responsibilities**:
- Execute Pure Pursuit path following algorithm
- Handle emergency stop commands and maneuvers to escape when stuck
- Aggregate sensor data (odometry, IMU) and forward to ESP-1

### Shared mecanisms (`common/`)

- `data_types.h`: Definitions of shared data structures
- `esp_link.h`: The communication system from one esp to the other. [Open esp_link README](include/common/esp_link_readme.md)
- `transforms.h`: Functions that transforms the position of the robot to the grid and the other way around. [Open transforms README](include/common/transforms_readme.md)
- `utils.h`: regroups the functions used accross different files to avoid redifining it.
- `wifi_connection.h`: Protocol to connect the wifi for MQTT

### Why This Architecture?

1. **Performance**: Each processor can focus on its task without competing for resources
2. **Modularity**: Components are well-isolated, making testing and debugging easier
3. **Reliability**: Control loop continues operating even if global planning encounters issues
4. **Efficiency**: Only essential datas are sent from one esp to the other, limiting the flow of information.


## Data Flow and Communication

### Inter-ESP Communication (UART)

**ESP-2 → ESP-1**: Current local pose estimate

**ESP-1 → ESP-2**: Global path updates (when new path computed)

### Ground Station Communication (wifi / MQTT)

The ESP creates a Wi-Fi access point (AP) that we connect to for monitoring purposes only. We used MQTT at first to get feedback during tests, but quickly switched to TCP, as it supports higher data throughput.

**ESP-1 → Ground Station**:
- Real-time map visualization data
- Mission status: goal and state
- Robot pose and trajectory
- Telemetry

--- 

#### Diagram
![alt text](/docs/global-architecture-2.png)


## Set it up
### Software
#### Prerequisites
* [Python 3.8+](https://www.python.org/downloads/)
* [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html) or VS Code extension.

#### Ground Station (PC)
The ground station script runs on your computer to visualize telemetry.
*Note: We recommend using a virtual environment.*
Install the dependencies from our `requirements.txt` file.

```bash
# Create virtual environment
python3 -m venv slamaleykoum_venv

# Activate environment
source slamaleykoum_venv/bin/activate # On Windows: slamaleykoum_venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Firmware (ESP32s)
This project uses two ESP32s. You must flash them individually.

Open in PlatformIO: Open the project folder in VS Code.

**Upload Code:**
Connect to the ESP1 via USB. Open the *platformio* extension in vscode (the alien logo). Click on `/esp2/General/Upload`. Repete for the ESP2.

---

## Usage / Operation

### 1. Power Up Sequence
1.  Connect the NiMH battery to the ESC.
2.  Turn on the ESC switch.
>WARNING: When disconnecting, first turn the ESC off, then disconnect the battery.

### 2. WiFi Connection
The car acts as an Access Point. Connect your computer to the following network:
* **SSID:** `LIDAR_AP`
* **Password:** `l1darpass`

> *Dev Note: To change these credentials, modify `include/wifi_config.h` (or your specific path) before flashing.*

### 3. Launch Ground Station
Once connected to the WiFi, launch the Python interface:

```bash
# Ensure your venv is active
source slamaleykoum_venv/bin/activate

# Run the interface
python ground_station.py
```
A *Pygame* window will appear. Press START to begin logging data and visualizing the real-time LIDAR feed.

## Troubleshooting
If you encounter issues, check the list below before reaching out.

| Problem | Possible Cause | Solution |
| :--- | :--- | :--- |
| **Map has "fuzzy" walls** | The environment might contain windows, or reflective surfaces, maybe even the obstacles are too thin or the walls have wholes in them (mesh / fence) | Since the lidar can't correctly detect those kinds of objects, try mapping something else |
| **Wifi doesn't appear** | You can not flasht the ESP32 from the left port, hence the code was never uploaded OR you might have uploaded the code of the ESP2 on the ESP1, it happens way too often | Flash the ESPs again, with their respective code on their RIGHT port (In our setup, it's the center ports, since the esp1 is upside-down) |
| **Wheels turn, car doesn't moove** | The motor doesn't get any power | The ESC must be switched on imidiatly after connecting the battery |
| **ESP doesn't initialize or crashes often** | Cables might have come loose | Double check the pin connections |
| **ESP keeps crashing** | Battery might be to low | Check the battery level, charge it |
| **Map is not displaying but python code is up** | Incoherent map sizes | Double check the values in the `ground_station.py` file and the rest of the code, for the size of the map (grid size) and their max bounds are the same |
| **ESP1 crashes imidiately** | The map size is too big | Reduce the map size. Note: The max nb of cells we managed to run with is 70x70 but if the real world size is not enough, you can increase the `RESOLUTION` value which will increase what each cell represents in the real world |

## Problems and Recommendations

### Common ESP1 & ESP2
- **Hardware wear and wiring issues**: reused components had weak solder joints and loose connectors, causing intermittent failures (especially the encoder).  
  **Fix**: replaced fragile connectors with screw terminal blocks → stable signals.  
  **Recommendation**: inspect wiring early, re‑solder weak joints, standardize connectors, and consider a small PCB for reliability.

### ESP1
- No major unique issues documented beyond general wiring/debugging challenges.

### ESP2
- **Encoder jitter**: noisy tick timing from electrical/mechanical issues degraded velocity estimation.  
  **Fix**: used AS5600 library for error handling.  
  **Recommendation**: sample encoder at high, consistent rates; fuse with IMU data; consider wheel‑mounted encoder for higher resolution.

- **Motor PID control**: stock THW‑1060 ESC only allowed discrete throttle steps, preventing smooth PID control.  
  **Recommendation**: replace with IBT‑4 (BTS7960) for smooth PWM and closed‑loop speed control.

- **IMU acceleration**: drift and bias made acceleration unreliable for odometry/EKF fusion.  
  **Recommendation**: 



## Archives
//TODO: Talk about the problems

#### File structure:
For now the file structure is still in development.
You can the things that WILL NOT CHANGE are mainly the hardware files, since they will "just" get/send data from/to the sensors/actuators and do so in a "nice" way to make it easily accessible from other modules. (aka integrate them with our custom api)

Parts like the `pid_controller` or `ekf_filter` in `odometry/` will not change either since regardless of how we link everyhting those are things we MUST do.

## Conclusion

---
