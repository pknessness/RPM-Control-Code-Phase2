#include <AccelStepper.h>

// Define pin connections & motor's steps per revolution
const float pi = 3.1415;
const int dirPin = 5;
const int stepPin = 2;
const int enPin = 8;
const int micro = 32;
const int stepsPerRevolution = 200*micro;
const int speed = 7*stepsPerRevolution/60;
const int accel = 0.2*stepsPerRevolution/(2*pi);

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);   // Set en step low to enable step circuit


  // Set up the stepper
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(accel);
  stepper.setSpeed(speed);
  stepper.moveTo(stepsPerRevolution*2);
}

void loop() {

  // Move the stepper
  if (stepper.distanceToGo() == 0) {
    delay(1000);
    stepper.moveTo(-stepper.currentPosition());
  }
  stepper.run();

}

