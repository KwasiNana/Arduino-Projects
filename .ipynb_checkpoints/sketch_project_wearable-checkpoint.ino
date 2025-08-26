#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <NewPing.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;
#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

Matrix<3, 3> A;        // State transition matrix
Matrix<3, 3> Q;        // Process noise covariance
Matrix<2, 2> R;        // Measurement noise covariance
Matrix<3, 3> P;        // Process covariance
Matrix<3, 1> x;        // State vector
Matrix<2, 1> z;        // Measurement vector
Matrix<3, 3> I;        // Identity matrix
Matrix<2, 3> H;        // Observation model

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
// Define LED pin (replace with your actual LED pin)
const int ledPin = 13;

// Define blue light threshold and distance threshold (adjust as needed)
const float threshold = 4; // Lux
const float distanceThreshold = 35.0; // cm

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// Variables for calibration
unsigned long lastReadTime = 0;
int readings[10]; // Array to store past 10 readings
int readingIndex = 0;
float calibratedLux = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (tsl.begin()) {
    Serial.println("Found a TSL2591 sensor");
  } else {
    Serial.println("No sensor found ... check your wiring?");
  }
  configureSensor(); 

  A.Fill(0);
  A(0, 0) = 1; A(0, 1) = 0.001; A(0, 2) = 0.0005; // Example values based on Δt
  A(1, 1) = 1; A(1, 2) = 0.001;
  A(2, 2) = 1;

  H.Fill(0);
  H(0, 0) = 1; // Maps position
  H(1, 2) = 1; // Maps acceleration
  Q.Fill(0.01); // Assume small process noise
  R.Fill(0.1); 
  I.Fill(0); // Assume small measurement noise
  I(0,0) = 1;
  I(1,1) = 1;
  I(2,2) = 1;
  P = I;
  x.Fill(0); // Start with zero state
}

void loop() {
  // Read the sensor and calculate calibrated lux
  delay(100);
  float dist = sonar.ping_cm();
  delay(1000);
  

  if(dist < 20.0){
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    z(0) = lux;
  
    if(z(0) < threshold){
      for (int i = 0; i < 2; i++){
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
       }
      }
      Serial.println("FIX YOUR POSTURE");
     }

  // Check if blue light exposure is above the threshold and within the distance range
  if (dist <= 40.00 && dist>= 20) {
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    z(0) = lux;
    z(1) = dist; 
    if (z(0) >= threshold && millis() > 50){
       for (int i = 0; i < 2; i++){
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
       }
       Serial.println("WATCH THE BRIGHTNESS");
    }
    else if (z(0) >= threshold && millis() > 1200000){
      for (int i = 0; i < 5; i++){
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
       }
      Serial.println("TAKE A BREAK");
    }
  }
  else {
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    z(0) = lux;
    z(1) = dist; 
    
    if (z(0) >= threshold && millis() > 50){
      for (int i = 0; i < 2; i++){
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
       }
      Serial.println("FIX YOUR POSTURE AND WATCH YOUR EYES");
    }
  }

  if (dist <= 40.00 && dist>= 20){
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);
    z(0) = lux;
    z(1) = dist; 
  
    if (z(0) < threshold){
       Serial.println("GOOD");
    }
  }
 
}

void configureSensor() {
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);

  Serial.println(F("------------------------------------"));
  Serial.print(F("Gain: "));
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain) {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print(F("Timing: "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC);
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}