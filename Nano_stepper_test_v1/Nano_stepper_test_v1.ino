// Define pin connections & motor's steps per revolution
const int dirPin = 5;
const int stepPin = 2;
const int enPin = 8;
const int stepsPerRevolution = 200; //800
const int stepSpeed = 2812.5*16;
const int stepsToMove = 800/16;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
}

void loop() {
  // Set en step low to enable step circuit
  digitalWrite(enPin, LOW);

 
  for (int i = 0; i<5; i++) {

    // Set motor direction (HIGH clockwise, LOW counterclockwise)
    digitalWrite(dirPin, LOW);

    // Spin motor slowly
    for(int x = 0; x < stepsToMove; x++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(stepSpeed);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(stepSpeed);
    }
    delay(1000); // Wait a tenth of a second

    // Set motor direction (HIGH clockwise, LOW counterclockwise)
    digitalWrite(dirPin, HIGH);

    // Spin motor quickly
    for(int x = 0; x < stepsToMove; x++) {
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(stepSpeed);
      digitalWrite(stepPin,LOW);
      delayMicroseconds(stepSpeed);
    }
    delay(100); // Wait a tenth of a second
  }
  while(1);
}

