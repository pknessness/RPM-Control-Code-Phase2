#include <AccelStepper.h>

// Define stepper motor connections and characteristics
const int DIR_PIN_A = 4;
const int STEP_PIN_A = 5;
const int DIR_PIN_B = 2;
const int STEP_PIN_B = 3;
// #define motorInterfaceType 1
const int ledPin = 13;
const int enPin = 8; // Enable pin for the stepper driver
const int micro = 64; // Microstepping factor

// Calculate steps per revolution
const float stepsPerRevolution = 2 * 200 * micro;

// Create instances of the stepper motors
AccelStepper stepperA(AccelStepper::DRIVER,STEP_PIN_A, DIR_PIN_A);
AccelStepper stepperB(AccelStepper::DRIVER,STEP_PIN_B, DIR_PIN_B);

void setup() {
  // Serial.begin(9600); // Start serial communication at 9600 baud
  pinMode(STEP_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(STEP_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(ledPin, OUTPUT);

  //digitalWrite(enPin, LOW); // Enable the stepper driver

  // Setup stepper motors
  stepperA.setMaxSpeed(7 * stepsPerRevolution / 60);
  stepperA.setSpeed(7 * stepsPerRevolution / 60);
  stepperA.setAcceleration(0.2 * stepsPerRevolution / (2 * 3.14159265));
  stepperB.setMaxSpeed(7 * stepsPerRevolution / 60);
  stepperB.setSpeed(7 * stepsPerRevolution / 60);
  stepperB.setAcceleration(0.2 * stepsPerRevolution / (2 * 3.14159265));

  // Initially move steppers to two revolutions
  stepperA.moveTo(stepsPerRevolution * 2);
  stepperB.moveTo(stepsPerRevolution * 2);
}

void loop() {
  // Continuously run the steppers towards their set target
  stepperA.run();
  stepperB.run();
}
