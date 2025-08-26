#include <MPU6050.h>
#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <KeyboardLayout.h>
#include <Keyboard_da_DK.h>
#include <Keyboard_de_DE.h>
#include <Keyboard_es_ES.h>
#include <Keyboard_fr_FR.h>
#include <Keyboard_hu_HU.h>
#include <Keyboard_it_IT.h>
#include <Keyboard_pt_PT.h>
#include <Keyboard_sv_SE.h>
#include <Q2HX711.h>

// This project implements a custom controller for Minecraft using an Arduino,
// an MPU6050 accelerometer, and a pressure sensor.
// The code translates physical head movements and breath pressure into
// in-game controls.

// --- Sensor Pin Definitions ---
// MPS_OUT_pin and MPS_SCK_pin connect to the HX711 module for the pressure sensor.
const byte MPS_OUT_pin = A2;
const byte MPS_SCK_pin = A3; // clock data pin
int avg_size = 10;           // Number of data points to average over (not used in current code).
float pressCal = 0.0;        // Variable for pressure calibration offset.

// Q2HX711 object to handle communication with the HX710B chip for the pressure sensor.
Q2HX711 hx711(MPS_OUT_pin, MPS_SCK_pin);

// --- Accelerometer Variables ---
// Variables for gyroscope and accelerometer calibration.
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

// Raw accelerometer values and their calibrated offsets.
float AccX, AccY, AccZ;
float aX = 0.0;
float aY = 0.0;
float aZ = 0.0;

// Variables to store pressure sensor readings.
float flow;
float air;
int factor;

// --- PID Control Variables ---
// PID gains for fine-tuning the mouse movement.
double Kp = 2.5;
double Ki = 0.1;
double Kd = 0.1;

// Variables to store past errors for integral and derivative calculations.
double previous_errorX = 0;
double integralX = 0;
double previous_errorY = 0;
double integralY = 0;

// Define the step size for tuning the PID gains.
const float tuningStep = 0.1;


// This function reads accelerometer data from the MPU6050.
void gyro_signals(void) {
  // Begin I2C transmission to the MPU6050 (address 0x68) to configure it.
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Request accelerometer data (6 bytes starting from register 0x3B).
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // Read the 16-bit raw data for each axis.
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Convert the raw 16-bit integer values to a float, with a scale factor of 4096.
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Subtract the calibration values to normalize the readings.
  AccX -= aX;
  AccY -= aY;
  AccZ -= aZ;
}

// The setup() function runs once when the sketch starts.
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Mouse.begin();
  Keyboard.begin(); // Initializes the keyboard library for sending key presses.
  Wire.setClock(400000); // Set I2C clock speed for MPU6050.
  Wire.begin();
  delay(250);

  // Wake up the MPU6050 sensor from sleep mode.
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // --- MPU6050 Calibration ---
  // This loop calibrates the accelerometer to find the average offset
  // for a neutral, non-tilted position.
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    aX += AccX;
    aY += AccY;
    aZ += AccZ;
    delay(1);
  }

  // Calculate the average offset values.
  aX /= 2000;
  aY /= 2000;
  aZ /= 2000;
  
  // Inform the user that the device is ready for use.
  Serial.println("Calibration complete. Device is ready.");
  Serial.println("Use 'p'/'o', 'i'/'u', 'd'/'c' to tune Kp, Ki, and Kd.");
  Serial.println("Current PID values:");
  Serial.print("Kp: "); Serial.println(Kp);
  Serial.print("Ki: "); Serial.println(Ki);
  Serial.print("Kd: "); Serial.println(Kd);
}

// The loop() function runs continuously.
void loop() {
  // Check for incoming serial data for PID tuning
  checkSerialForPIDTuning();
  
  // Read from the sensors in every loop cycle.
  gyro_signals();
  air = hx711.read() / 100.0;
  flow = 90000.0; // This is a hardcoded baseline value for the pressure sensor.

  // --- In-Game Action Control via Pressure Sensor ---
  if ((air - flow) > 25 && (air - flow) < 600) {
    factor = 1;
    Mouse.click(MOUSE_LEFT);
    Serial.println("fight");
  } else if ((air - flow) < -2800) {
    factor = -1;
    Mouse.click(MOUSE_RIGHT);
    Serial.println("block");
  } else {
    factor = 0;
  }

  // --- WASD Movement via Head Tilt ---
  if (AccZ < -0.4) {
    Keyboard.press('w');
  } else {
    Keyboard.release('w');
  }
  if (AccZ > 0.1) {
    Keyboard.press('s');
  } else {
    Keyboard.release('s');
  }
  if (AccY < -0.2) {
    Keyboard.press('d');
  } else {
    Keyboard.release('d');
  }
  if (AccY > 0.2) {
    Keyboard.press('a');
  } else {
    Keyboard.release('a');
  }
  
  // --- Jump and Fly Control via Pressure Sensor ---
  if ((air - flow) > 600) {
    Keyboard.press(' ');
    Serial.println("Jump");
  } else if ((air - flow) < 600) {
    Keyboard.release(' ');
  }
  if ((air - flow) > 2000) {
    Keyboard.press(' ');
    Keyboard.release(' ');
    Keyboard.press(' ');
    Keyboard.release(' ');
    Keyboard.press(' ');
    Keyboard.release(' ');
    Serial.println("fly");
  }
  
  // --- PID Control for Mouse Look ---
  double errorX = AccX;
  integralX += errorX;
  double derivativeX = errorX - previous_errorX;
  int mouse_move_X = (int)(Kp * errorX + Ki * integralX + Kd * derivativeX) * 10;
  
  double errorY = AccY;
  integralY += errorY;
  double derivativeY = errorY - previous_errorY;
  int mouse_move_Y = (int)(Kp * errorY + Ki * integralY + Kd * derivativeY) * 10;

  // The mouse movement is commented out to allow for testing without interfering with the user's mouse.
  // Uncomment this line to enable mouse control.
  // Mouse.move(mouse_move_Y, mouse_move_X);
  
  previous_errorX = errorX;
  previous_errorY = errorY;

  delay(10);
}

// --- Live PID Tuning (via Serial Monitor) ---
// This function checks for and processes single-character commands sent over the serial monitor
// to adjust the PID gains on the fly. This is a powerful debugging and tuning tool.
void checkSerialForPIDTuning() {
  if (Serial.available()) {
    char command = Serial.read();
    
    // --- Kp Tuning ---
    if (command == 'p') {
      Kp += tuningStep;
      Serial.print("Kp increased to: ");
      Serial.println(Kp);
    } else if (command == 'o') {
      Kp -= tuningStep;
      Serial.print("Kp decreased to: ");
      Serial.println(Kp);
    }
    
    // --- Ki Tuning ---
    else if (command == 'i') {
      Ki += tuningStep;
      Serial.print("Ki increased to: ");
      Serial.println(Ki);
    } else if (command == 'u') {
      Ki -= tuningStep;
      Serial.print("Ki decreased to: ");
      Serial.println(Ki);
    }
    
    // --- Kd Tuning ---
    else if (command == 'd') {
      Kd += tuningStep;
      Serial.print("Kd increased to: ");
      Serial.println(Kd);
    } else if (command == 'c') {
      Kd -= tuningStep;
      Serial.print("Kd decreased to: ");
      Serial.println(Kd);
    }
    
    // An optional command to print all current values
    else if (command == 'v') {
      Serial.print("Current Kp: "); Serial.println(Kp);
      Serial.print("Current Ki: "); Serial.println(Ki);
      Serial.print("Current Kd: "); Serial.println(Kd);
    }
  }
}