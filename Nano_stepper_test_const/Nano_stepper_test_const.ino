// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 2;
const int enPin = 8;
const int micro = 1;
const int stepsPerRevolution = 200*micro; //800
const int stepSpeed = 3000/micro;
const int stepsToMove = 800/micro;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

    // Set en step low to enable step circuit
  digitalWrite(enPin, LOW);

  // Set motor direction (HIGH clockwise, LOW counterclockwise)
  digitalWrite(dirPin, LOW);
}

void loop() {


  digitalWrite(stepPin,HIGH);
  delayMicroseconds(stepSpeed);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(stepSpeed);

}

