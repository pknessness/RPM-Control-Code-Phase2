#include <AccelStepper.h>

// Define stepper motor connections and characteristics
#define DIR_PIN_A 5;
#define STEP_PIN_A 2;
#define DIR_PIN_B 6;
#define STEP_PIN_B 3;
#define motorInterfaceType 1
#define enPin 8; // Enable pin for the stepper driver
const int micro = 32; // Microstepping factor

// Calculate steps per revolution
const float stepsPerRevolution = 200 * micro;

// Create instances of the stepper motors
AccelStepper stepperA(motorInterfaceType, STEP_PIN_A, DIR_PIN_A);
AccelStepper stepperB(motorInterfaceType, STEP_PIN_B, DIR_PIN_B);

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); // Enable the stepper driver

  // Setup stepper motors
  stepperA.setMaxSpeed(7 * stepsPerRevolution / 60);
  stepperA.setAcceleration(0.2 * stepsPerRevolution / (2 * 3.14159265));
  stepperB.setMaxSpeed(7 * stepsPerRevolution / 60);
  stepperB.setAcceleration(0.2 * stepsPerRevolution / (2 * 3.14159265));

  // Initially move steppers to two revolutions
  stepperA.moveTo(stepsPerRevolution * 2);
  stepperB.moveTo(stepsPerRevolution * 2);
}

void loop() {
//  if (Serial.available() > 0) {
//   // Read the incoming data from Python
//   String data = Serial.readStringUntil('\n');
//   int commaIndex = data.indexOf(',');
//   long stepsA = data.substring(0, commaIndex).toInt();
//   long stepsB = data.substring(commaIndex + 1).toInt();

// Set the new target positions for the stepper motors
  stepperA.moveTo(stepsA);
  stepperB.moveTo(stepsB);


  // Continuously run the steppers towards their set target
  stepperA.run();
  stepperB.run();
}
