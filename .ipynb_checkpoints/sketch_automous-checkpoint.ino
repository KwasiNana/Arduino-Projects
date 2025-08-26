#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <SPI.h>
#include <RFM69.h>
#include <RFM69registers.h>
#include <Servo.h>
#include <SoftwareSerial.h> 
#include <TinyGPS++.h> 
#include <Wire.h> 
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h>

using namespace BLA;

// Define pins
const int motor1Pin = 9; 
const int motor2Pin = 10; 
const int steerPin = 11; 
const int gpsRxPin = 8; 
const int gpsTxPin = 7; 
const int ssPin = 10; // Slave Select pin for RF45


// Create objects
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin); 
TinyGPSPlus gps; 
Servo motor1; 
Servo motor2; 
Servo steer; 
Adafruit_BNO055 bno = Adafruit_BNO055(55); 

// Define states
enum CarState {
  IDLE, 
  DRIVING, 
  TURNING,  
  EMERGENCY_STOP 
};
CarState currentState = IDLE; 

// Variables
float targetLatitude, targetLongitude; 
float currentLatitude, currentLongitude; 
float desiredHeading; 
float currentHeading; 
float kp = 0.5; // Proportional gain
float ki = 0.01; // Integral gain
float kd = 0.1; // Derivative gain
float errorSum = 0;
float lastError = 0;
float distanceToTarget; // Distance to target position
float desiredRadius = 5.0; // Desired radius for circular path
float turningAngle = 45.0; // Angle for turning maneuver
unsigned long previousMillis = 0; 
const long interval = 1000; // Time interval for turning (milliseconds)
bool turningComplete = false; // Flag for turning completion
bool buttonState = false; 
bool lastButtonState = false;

// The following definitions set the various pins and constants that will be needed for this project
#define JOYSTICK_PINX A1 // Joystick x-axis 
#define JOYSTICK_PINY A2 // Joystick y-axis 
#define BUTTON_PIN 10 //  button 
#define FREQUENCY  880 // frequency for car b
#define NODEID 1 // part of library found online
#define NETWORKID  100 // part of library found online

// Data structure that contains joystick data
struct Data {
 int joystickX; // joystick x-axis position 
 int joystickY; // joystick y-axis position 
 bool modeButtonPressed; // Mode button state
};

Data sendData; // data that will be sent to the RC car
Data receivedData; // data that received from the car
RFM69 radio;

Servo leftMotor;
Servo rightMotor;
Servo steeringServo;


void handleJoystickControl(Data data); // start data transfer

void setup(){
leftMotor.attach(A2); // defining which analog pin the left motor is 
rightMotor.attach(A2);
steeringServo.attach(A1);

// Reset for wireless module
 pinMode(9, OUTPUT);
 digitalWrite(9, HIGH);
 delay(100);
 digitalWrite(9, LOW);
 delay(100);

 Serial.begin(9600);
// Radio Initialize
 radio.initialize(FREQUENCY, NODEID, NETWORKID);
//Button Setup
 pinMode(BUTTON_PIN, INPUT_PULLUP); 

  // Initialize GPS
  gpsSerial.begin(9600); 

  // Initialize IMU
if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring OR platform?");
    while (1); 
  }
bno.setExtCrystalUse(true); 
}

void loop() {
  // Read button state
  buttonState = !digitalRead(BUTTON_PIN);

  // Read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read()); 
  }

  if (gps.location.isUpdated()) {
    currentLatitude = gps.location.lat();
    currentLongitude = gps.location.lng();
  }

  // Read IMU data
   // Read IMU data
  sensors_event_t event; 
  bno.getEvent(&event); 
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
  currentHeading = euler.z();

  // Calculate distance to target
  distanceToTarget = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude); 

  switch (currentState) {
    case IDLE:
      // Stop motors
      motor1.writeMicroseconds(1500); 
      motor2.writeMicroseconds(1500); 
      steer.write(90); // Center steering

      // Transition to DRIVING state when button is pressed
      if (buttonState && !lastButtonState) { 
        currentState = DRIVING; 
      }
      break;

    case DRIVING:
      // Calculate desired heading (example: circular path)
      desiredHeading = calculateDesiredHeading(currentLatitude, currentLongitude, targetLatitude, targetLongitude); 

      // Calculate steering error
      float error = desiredHeading - currentHeading; 

      // PID Control
      errorSum += error * 0.01; // 0.01 is the loop time (adjust as needed)
      float errorRate = (error - lastError) / 0.01; 
      float steeringAngle = kp * error + ki * errorSum + kd * errorRate;

      // Limit steering angle (optional)
      steeringAngle = constrain(steeringAngle, -90, 90); 

      // Set steering servo position
      steer.write(steeringAngle);

      // Control motor speed (example: constant speed)
      motor1.writeMicroseconds(1500); // Adjust for desired speed
      motor2.writeMicroseconds(1500); 

      // Transition to TURNING state if distance exceeds desired radius
      if (distanceToTarget > desiredRadius) { 
        currentState = TURNING;
        turningComplete = false; // Reset turning completion flag
      }
      break;

    case TURNING:
      // Calculate steering angle for turning
      steer.write(90 + turningAngle); // Adjust based on desired turn direction

      // Transition back to DRIVING state after a delay
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        turningComplete = true; 
      }

      if (turningComplete) {
        currentState = DRIVING; 
      }
      break;

    case EMERGENCY_STOP:
      // Stop motors immediately
      motor1.writeMicroseconds(1500); 
      motor2.writeMicroseconds(1500); 
      steer.write(90); // Center steering
      break;
    }
    lastButtonState = buttonState;
  }

// Helper functions
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula for calculating distance between two GPS coordinates
  const double R = 6371000; // Radius of the Earth in meters
  float dLat = deg2rad(lat2 - lat1);
  float dLon = deg2rad(lon2 - lon1);
  float a = 
      sin(dLat / 2) * sin(dLat / 2) +
      cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * 
      sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = R * c; 
  return distance;
}

float calculateDesiredHeading(float currentLat, float currentLon, float targetLat, float targetLon) {
  // Calculate bearing (initial heading)
  float dLon = deg2rad(targetLon - currentLon);
  float y = sin(dLon) * cos(deg2rad(targetLat));
  float x = cos(deg2rad(currentLat)) * sin(deg2rad(targetLat)) - 
           sin(deg2rad(currentLat)) * cos(deg2rad(targetLat)) * cos(dLon);
  float brng = atan2(y, x);
  brng = fmod((brng * 180 / M_PI) + 360, 360); // Convert to degrees and normalize to 0-360

  // Adjust for circular path (basic implementation)
  // This example assumes clockwise circular motion
  float offsetAngle = 90; // Adjust this value to control the curvature of the path
  float desiredHeading = brng + offsetAngle;
  if (desiredHeading > 360) {
    desiredHeading -= 360;
  }

  return desiredHeading;
}

float deg2rad(float deg) {
  return deg * M_PI / 180;
}

