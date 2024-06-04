// Motor 1 pin definitions
#define MOTOR1_STEP_PIN 5       // Motor 1 Step pin connected to digital pin 2
#define MOTOR1_DIR_PIN 4        // Motor 1 Direction pin connected to digital pin 3

// Motor 2 pin definitions
#define MOTOR2_STEP_PIN 3       // Motor 2 Step pin connected to digital pin 4
#define MOTOR2_DIR_PIN 2        // Motor 2 Direction pin connected to digital pin 5

// Number of base steps per motor revolution (change this depending on your motor)
#define BASE_STEPS_PER_REVOLUTION 200

// Microstepping setting (full-step = 1, half-step = 2, etc.)
#define MICROSTEPPING 8  // This value can be adjusted as needed

// Enable pin
#define enPin 8

// Desired RPM
const int desiredRPM = 10;
const int stepDelay = (((60/desiredRPM)*1000000)/(BASE_STEPS_PER_REVOLUTION * MICROSTEPPING));

void setup() {
  // Setup motor 1 pins
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  
  // Setup motor 2 pins
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);

  // Set motor directions (HIGH for one direction, LOW for the other)
  digitalWrite(MOTOR1_DIR_PIN, LOW);
  digitalWrite(MOTOR2_DIR_PIN, LOW);

  digitalWrite(enPin, LOW); //Low for enable, High for disable
}

void loop() {
  digitalWrite(MOTOR1_STEP_PIN, HIGH);
    digitalWrite(MOTOR2_STEP_PIN, LOW);
    delayMicroseconds(stepDelay);

    // Pulse the step pin for motor 2
    digitalWrite(MOTOR2_STEP_PIN, HIGH);
    digitalWrite(MOTOR1_STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
}
