# Arduino Projects Portfolio

This repository contains a collection of three distinct Arduino-based projects, each demonstrating the application of various sensors, control systems, and communication methods.

***

### 1. BioGaming Minecraft Controller with PID Smoothing

This project contains an Arduino-based **biogaming interface** built for the EN.580.471/771 Principles of BME Instrumentation (Fall 2024) course project. It allows a player to control a **Minecraft avatar hands-free** using a combination of:

-   **MPU6050 accelerometer/gyroscope** → detects head tilts for movement (WASD)
-   **Q2HX711 + Load Cell** → detects breath/pressure for jump and fly commands
-   **Arduino Leonardo HID** → emulates keyboard and mouse input to Minecraft

The design is targeted for **patients with bilateral upper-limb amputations**, enabling full gameplay without traditional hand-based controls.

---

### Features

-   **Head tilt control (MPU6050)**
    -   Tilt forward/back → `W` / `S` movement
    -   Tilt left/right → `A` / `D` strafing
-   **Breath pressure (Q2HX711)**
    -   Moderate breath → Jump (`Space`)
    -   Sustained breath → Fly toggle
-   **Mouse control**
    -   Click left/right to attack/place blocks
-   **Custom PID Smoothing**
    -   Filters noisy accelerometer signals
    -   Makes movement smoother, reducing jitter and improving accessibility
-   **Live PID Tuning (via Serial Monitor)**
    -   Adjust gains in real time without recompiling
    -   Controls:
        -   `p` = increase Kp, `o` = decrease Kp
        -   `i` = increase Ki, `u` = decrease Ki
        -   `d` = increase Kd, `c` = decrease Kd
    -   Prints updated values for easy tuning

---

### Files

-   **`Minecraft_project_two_PID_with_SerialTuning.ino`**
    Main Arduino sketch with PID smoothing and live tuning.

-   **`LICENSE`**
    Apache 2.0 License (open-source distribution).

---

### Usage

1.  Connect the MPU6050 and HX711 + load cell to the Arduino Leonardo.
2.  Upload the `.ino` sketch using Arduino IDE.
3.  Open Minecraft on your computer.
4.  Open Arduino Serial Monitor at **9600 baud** (optional for tuning).
5.  Control your avatar via head tilts + foot pressure!

---

### Reference

This project is inspired by the EN.580.471/771 course project prompt and designed to improve **accessibility of gaming** for individuals with disabilities.

***

### 2. Autonomous GPS-Guided Car (`sketch_automous.ino`)

This project details the code for an autonomous vehicle designed to navigate to a target GPS coordinate. The system uses a state machine to manage different operational modes.

* **Key Components:** The car is controlled by two motors and a steering servo. It uses a **TinyGPS++** library with a GPS module to determine its current location, while an **Adafruit BNO055 IMU** provides precise heading data. An RFM69 wireless module is also included for potential remote control or data transmission.
* **Functionality:** A **PID control loop** is implemented to calculate the necessary steering angle to correct for errors between the current heading and the desired heading. The `calculateDistance()` function uses the **Haversine formula** to determine the distance between the current and target GPS coordinates. The code includes a state machine with states for `IDLE`, `DRIVING`, `TURNING`, and `EMERGENCY_STOP`.

***

### 3. Wearable Posture and Eye Health Monitor (`sketch_project_wearable.ino`)

This project is a wearable device that monitors a user's posture and screen-related eye strain. It provides a series of warnings using an LED based on distance and light levels.

* **Key Components:** The device utilizes an **Adafruit TSL2591** light sensor to detect blue light exposure, while a **NewPing ultrasonic sensor** measures the distance to an object, such as a computer monitor.
* **Functionality:** The code checks for a combination of conditions to determine if a warning is needed. It warns the user to "FIX YOUR POSTURE" if they are too close to the screen, "WATCH THE BRIGHTNESS" if a high amount of blue light is detected, and "TAKE A BREAK" if excessive exposure is detected over a period of time. The sketch also includes a basic implementation of a **Kalman filter** to potentially improve sensor data accuracy by fusing measurements.