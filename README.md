 WRO Future Engineers 2025

This repository contains engineering materials of a **self-driven vehicle model** built for the **WRO Future Engineers competition** in the 2025 season.  
The project demonstrates **sensor fusion, vision-based navigation, and autonomous control logic** on a small-scale robotic vehicle.  

---

## 📂 Repository Content

- **t-photos/** → 2 team photos (official + funny).  
- **v-photos/** → 6 photos of the vehicle (all sides, top, and bottom).  
- **video/** → `video.md` with link to the driving demo.  
- **schemes/** → Electrical/mechanical schematics showing sensors, controllers, and wiring.  
- **src/** → Control software:  
  - `tracker.py` – Python vision system (Raspberry Pi).  
  - `main.ino` – Arduino/ESP32 firmware for sensors and actuators.  
- **models/** → CAD/3D print/CNC files for the vehicle.  
- **other/** → Extra documentation (protocols, specs, setup notes).  

---

## 🧑‍💻 Code and Logic Overview

### **Python Vision Module (`tracker.py`)**
- Captures video via PiCamera/USB camera.  
- Detects **red and yellow pillars** with HSV color masks.  
- Computes:
  - **Color code** (1=red, 0=yellow, 2=none)  
  - **Error** (horizontal offset from center)  
  - **Area** (size = confidence).  
- Sends results over serial (`color error area`).  
- Provides live MJPEG stream + JSON API via Flask.  

👉 **Why?**  
Vision is computationally heavy, so it runs on the Raspberry Pi. This keeps the Arduino free for **real-time control**.

---

### **Arduino/ESP32 Firmware (`main.ino`)**
- Reads **two LIDARs VL53L1X  TOF 400C** (left/right) → wall-following.  
- Reads **ultrasonic sensor HC SRO4** (front) → obstacle detection.  
- Reads **TCS34725 color sensor** (via I²C mux) → backup/extra detection.  
- Receives **vision data - 160 degree FOV CAMERA** from Pi via serial.  
- Controls **servo steering -HOBBY SERVO ** and **motor PWM - (25 ga)**.  

👉 **Why?**  
The microcontroller fuses all sensor inputs and ensures **low-latency actuation**.

---

## Control Logic

The control system is **layered**:

1. **Wall-following (baseline)**  
   - Error = Right LIDAR – Left LIDAR.  
   - PD control keeps car centered.  
   - Works even if vision is unavailable.  

2. **Vision-based correction**  
   - If a colored pillar is detected:  
     - **Red → steer left**  
     - **Green → steer right**  
   - Influence grows with object size (area = closeness).  
   - Prevents reacting to false detections far away.  

3. **Obstacle avoidance**  
   - If sonar < 30 cm → stop, turn sharply.  
   - If sonar < 50 cm → apply extra correction.  
   - Protects against sudden obstacles that LIDAR/vision may miss.  

4. **Blending strategy**  
   - Final steering = mix of wall PD + vision correction.  
   - Weight depends on pillar area (confidence).  
   -  Smooth handoff between modes, avoids conflicts.  

---

## ⚙️ Why This Works

- **Redundancy:** If one sensor fails, others cover.  
- **Scalability:** Modular code, sensors can be swapped.  
- **Competition-ready:** Combines stable navigation + color-based decision making.  
- **Safety:** Sonar ensures last-resort collision prevention.  

---

## 🚀 Running the System

### Vision Module (Python on Raspberry Pi 4B)
```bash
pip3 install opencv-python numpy flask pyserial picamera2
python3 tracker.py
