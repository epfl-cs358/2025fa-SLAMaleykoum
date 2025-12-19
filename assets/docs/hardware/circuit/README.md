# SLAM Car Electrical Diagram

<p align="center">
  <img src="/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png" alt="Electrical Circuit" width="1000"/>
</p>



## Quick Recap

This diagram represents the electrical circuit of our project.  
The colors mostly match the real car wiring, with a few adjustments made for clarity and visibility.


### Adjustements: 
- **ESP-to-ESP communication cables** are shown in pink/blue (real life: black) to avoid confusion with ground.  
- **ESC PWM to ESP2 pin 15** is shown in grey (real life: white) to make it stand out clearly.  

## Detailed explaination of the wiring

#### Power Connections

- **Battery Power Split**:  
  Solder a 2‑PIN JST male connector to the 7.2V battery.  
  This battery powers the ESC and also supplies input to the buck converter.  
  Connect the black wire to the converter’s VIN (–) and the red wire to VIN (+).

<p align="center">
  <img src="/assets/Images/circuit_images/battery_buck_esc_soldering.jpeg" alt="Battery power split" width="200"/>
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
  - **5V power (red):** one male connector (main feed), plus branches to both ESP32 boards (two female connectors), the ultrasonic sensor (male), the lidar (male), and the servo (male).  
  - **Ground (black):** common ground returning to **OUT –** for all the same devices. The ground from the buck converter is taken from **OUT –** via a male connector, then soldered to multiple cables that connect to the GND pins of different components:  
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
  - Power the encoder through its **VCC** pin using a cable screwed into the terminal block and connected to the **3V3 pin of ESP2** via a female connector (see [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
  - Ground (GND) is connected to the buck converter’s ground harness as described above.

<p align="center">
  <img src="/assets/Images/circuit_images/encoder_connector_mount.jpg" alt="Encoder screw terminal blocks" width="200"/>
</p>

- **IMU**:  
  - Powered from the **3V3 pin of ESP2**, using a male connector on the IMU side and a female connector to ESP2 (see [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
  - Ground (GND) is connected via a male connector on the IMU and a female connector on ESP2, both tied into the buck converter’s ground harness as described above.

#### Data Connections

- **Both ESPs**  
  - Communication via UART: connect pin 12 of ESP1 to pin 13 of ESP2, and pin 13 of ESP1 to pin 12 of ESP2 (light pink and blue cables on the [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).

<p align="center">
  <img src="/assets/Images/circuit_images/IMG_2675.jpg" alt="Both ESPs with their respective cables" width="200"/>
</p>

- **ESP1**  
  - **Lidar**  
    - **TX**: male‑to‑female cable connected to pin 4 on ESP2 (green cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
    - **RX**: male‑to‑female cable connected to pin 5 on ESP2 (yellow cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).

- **ESP2**  
  - **Motor Driver**  
    - **PWM**: male‑to‑female cable connected to pin 15 on ESP2 (grey cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **Servo**  
    - **Pulse**: male‑to‑female cable connected to pin 6 on ESP2 (orange cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **Encoder**  
    - **DIR**: male‑to‑female cable connected to GND on ESP2 (yellow cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **IMU**  
    - **INT**: male‑to‑female cable connected to pin 4 on ESP2 (black cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  

  - **I²C Bus (shared between Encoder & IMU)**  
    - **SDA**: two male cables (encoder + IMU) soldered together into a female connector, attached to pin 8 on ESP2 (purple cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
    - **SCL**: two male cables (encoder + IMU) soldered together into a female connector, attached to pin 9 on ESP2 (green cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).
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
      - **Trig**: male‑to‑female cable connected directly to **ESP2 pin 5** (green cable on [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png)).  
      - **Echo**: routed through the resistor voltage divider before reaching **ESP2 pin 19** (female connector).
<p align="center">
  <img src="/assets/Images/circuit_images/IMG_2672.jpg" alt="Ultra sonic cables" width="200"/>
</p>

If you want to view more pictures of the car check : [Here](/assets/Images/Photoshoot/)

