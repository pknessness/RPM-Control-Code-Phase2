#include <AccelStepper.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 2;
const int enPin = 8;
const int stepsPerRevolution = 200*16;

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  // Set up the stepper
  stepper.setMaxSpeed(3000.0);
  stepper.setAcceleration(500.0);
  stepper.setSpeed(500.0);
  stepper.moveTo(stepsPerRevolution/4);
}

void loop() {
  // Set en step low to enable step circuit
  digitalWrite(enPin, LOW);

  // Move the stepper
  if (stepper.distanceToGo() == 0) {
    delay(1000);
    stepper.moveTo(-stepper.currentPosition());
  }
  stepper.run();

}

