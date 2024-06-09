# Arduino Autonomous Robot

This project is an Arduino-based autonomous robot designed to navigate and avoid obstacles using an ultrasonic sensor. The robot moves forward, backward, and turns left or right based on the detected distance from obstacles. It uses DC motors for movement and a servo motor for directional sensing.

## Introduction
The Arduino Autonomous Robot is a simple yet effective way to understand the basics of robotics, sensors, and motor control. This project can be used for educational purposes or as a foundation for more advanced robotic projects.

## Components
- Arduino Uno
- Ultrasonic Sensor (HC-SR04)
- DC Motors x4
- Motor Driver Shield (AFMotor)
- Servo Motor
- Battery Pack
- Jumper Wires
- Chassis

## Installation
1. **Clone the repository:**
   ```sh
   git clone https://github.com/yourusername/arduino-autonomous-robot.git
   ```
2. **Install the necessary libraries:**
   - AFMotor
   - NewPing
   - Servo

3. **Upload the code to your Arduino:**
   Open `robot.ino` in the Arduino IDE and upload it to your Arduino Uno.

## Usage
1. **Assemble the robot:**
   - Connect the ultrasonic sensor to the Arduino (Trig to A0, Echo to A1).
   - Connect the motors to the motor driver shield.
   - Attach the servo motor to pin 10.
   - Ensure the battery pack is connected to power the motors and Arduino.

2. **Power on the robot:**
   - Place the robot on the ground in an open area.
   - Turn on the power switch.

3. **Observe the robot:**
   - The robot should move forward and avoid obstacles by turning left or right based on the sensor readings.

## Code Explanation
The code consists of several key functions:
- `readPing()`: Reads the distance from the ultrasonic sensor.
- `moveForward()`, `moveBackward()`, `turnRight()`, `turnLeft()`, `moveStop()`: Functions to control the movement of the robot.
- `executeObstacleAvoidance()`: Handles obstacle detection and avoidance logic.
- `lookRight()`, `lookLeft()`: Functions to check distances on either side using the servo.

```cpp
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// Define pins and constants
#define TRIG_PIN A0
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 190
#define MAX_SPEED_OFFSET 20

// Initialize components
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;

void setup() {
    Serial.begin(9600);
    myservo.attach(10);
    myservo.write(115);
    delay(2000);
    distance = readPing();
    delay(100);
}

void loop() {
    distance = readPing();
    Serial.println(distance);
    if (distance <= 15) {
        executeObstacleAvoidance();
    } else {
        moveForward();
    }
}
```