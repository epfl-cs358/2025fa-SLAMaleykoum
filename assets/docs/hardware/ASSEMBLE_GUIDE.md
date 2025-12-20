
🏠 [Backlink to the main README](/README.md)

# How to Assemble

To begin, assemble the mechanical base of the car following the official [Tamiya Blitzer Beetle manual](https://www.tamiyausa.com/media/files/58502ml-829-5367.pdf).
We only need the minimal mechanical build so the car can drive, skip the decorative carcass.

**Step 1: Build the Car**  
- Assemble the chassis according to the manual.  
- Mount the steering servo inside the chassis.  
- Ensure the servo cables are routed through the rectangular opening so they can later connect to the ESP.  

<p align="center"><img src="/assets/Images/tamiya blitzer chassis.jpg" alt="Chassis" width="200"/></p>

**Step 2: Encoder Mount + Magnet**  
- 3D‑print the encoder mount [CAD file](/assets/docs/hardware/CAD/).  
- Attach the mount in front of the motor.  
- Fix the encoder magnet directly onto the motor shaft as shown in the provided image.  
- This ensures accurate rotation measurement.  
- Then screw down the encoder on its mount.

<p align="center"><img src="/assets/Images/Photoshoot/IMG_2678.jpg" alt="Encoder + Magnet" width="200"/></p>

**Step 3: Layer Platform (ESP + Lidar Holder)**  
- Place the 3D‑printed platform on the chassis.  
- This platform integrates holders for both ESP boards and the lidar.  
- Screw the lidar onto its holder first (space is tight, so it’s easier to mount before other components).  
- Then screw down the buck converter and IMU onto their designated spots.  


<p align="center"><img src="/assets/Images/Photoshoot/IMG_2671.jpg" alt="Lidar" width="200"/></p>

**Step 4: Mount the ESP Boards**  
- Place ESP1 and ESP2 onto the platform.  
*Improovement note: mount both microcontrollers facing **upwards** so their LEDs and reset buttons are visible.*
- Our current ESP1 faces down, which hides the LEDs and complicates troubleshooting.  


<p align="center"><img src="/assets/Images/circuit_images/IMG_2675.jpg" alt="ESPs" width="200"/></p>

**Step 5: Ultrasonic Sensor Mount**  
- Attach the front 3D‑printed piece designed for the ultrasonic sensor at the front of the car.  
- Screw the sensor securely into the mount.  

<p align="center"><img src="/assets/Images/Photoshoot/IMG_2674.jpg" alt="Ultrasonic mount" width="200"/></p>
<p align="center"><img src="/assets/Images/Photoshoot/IMG_2673.jpg" alt="Ultrasonic mount" width="200"/></p>

**Step 6: Final Assembly**  
- Place the hood/top of the car back on.  
- Insert the battery into its dedicated slot the hood design holds it firmly and stabilizes it during motion.  
- Double‑check that all mounts are secure and cables are routed cleanly for wiring.  

<p align="center"><img src="/assets/Images/Photoshoot/IMG_2669.jpg" alt="Front open view" width="200"/></p>
<p align="center"><img src="/assets/Images/Photoshoot/IMG_2666.jpg" alt="Front closed" width="200"/></p>
<p align="center"><img src="/assets/Images/Photoshoot/IMG_2667.jpg" alt="Back with battery" width="200"/></p>


**Step 7: Wiring**  
- Once the mechanical build is complete, proceed to the [Soldering & Wiring](#soldering--wiring) section.  
- Follow the circuit [diagram](/assets/docs/hardware/circuit/slamaleykoum_electrical_circuit.drawio.png) for all connections.  


<p align="center"><img src="/assets/Images/Photoshoot/IMG_2668.jpg" alt="Side view open" width="200"/></p>
