# BioGaming Minecraft Controller with PID Smoothing

This repository contains an Arduino-based **biogaming interface** built for the  
EN.580.471/771 Principles of BME Instrumentation (Fall 2024) course project.  
It allows a player to control a **Minecraft avatar hands-free** using a combination of:

- **MPU6050 accelerometer/gyroscope** → detects head tilts for movement (WASD)
- **HX711 + Load Cell** → detects foot pressure for jump and fly commands
- **Arduino Leonardo HID** → emulates keyboard and mouse input to Minecraft

The design is targeted for **patients with bilateral upper-limb amputations**,  
enabling full gameplay without traditional hand-based controls.

---

## 🔧 Features
- **Head tilt control (MPU6050)**  
  - Tilt forward/back → `W` / `S` movement  
  - Tilt left/right → `A` / `D` strafing  

- **Foot pressure (Load Cell)**  
  - Tap/step → Jump (`Space`)  
  - Sustained press → Fly toggle  

- **Mouse control**  
  - Click left/right to attack/place blocks  

- **Custom PID Smoothing**  
  - Filters noisy accelerometer signals  
  - Makes movement smoother, reducing jitter and improving accessibility  

- **Live PID Tuning (via Serial Monitor)**  
  - Adjust gains in real time without recompiling  
  - Controls:  
    - `p` = increase Kp, `o` = decrease Kp  
    - `i` = increase Ki, `u` = decrease Ki  
    - `d` = increase Kd, `c` = decrease Kd  
  - Prints updated values for easy tuning  

---

## 📂 Files
- **`Minecraft_project_two_PID_with_SerialTuning.ino`**  
  Main Arduino sketch with PID smoothing and live tuning.  

- **`LICENSE`**  
  Apache 2.0 License (open-source distribution).  

---

## 🚀 Usage
1. Connect the MPU6050 and HX711 + load cell to the Arduino Leonardo.  
2. Upload the `.ino` sketch using Arduino IDE.  
3. Open Minecraft on your computer.  
4. Open Arduino Serial Monitor at **9600 baud** (optional for tuning).  
5. Control your avatar via head tilts + foot pressure!  

---

## 📖 Reference
This project is inspired by the EN.580.471/771 course project prompt and designed  
to improve **accessibility of gaming** for individuals with disabilities. 
