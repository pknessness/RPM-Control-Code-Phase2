#include <SD.h>
#include <Math.h>
#include <Servo.h>

File myFile;
Servo servoYaw;
Servo servoPitch;

const int chipSelect = 4; // Chip select pin for the SD card

void setup() {
  Serial.begin(9600);
  //servoYaw.attach(9);  // Connect the Yaw servo to pin 9
  //servoPitch.attach(10); // Connect the Pitch servo to pin 10


void loop() {
  if (myFile.available()) {
    String line = myFile.readStringUntil('\n');
    calculateAndMove(line);
  } else {
    Serial.println("End of file");
    myFile.close(); // close the file:
    while(true); // Stop the loop
  }
}

void calculateAndMove(String dataLine) {
  int firstCommaIndex = dataLine.indexOf(',');
  int lastCommaIndex = dataLine.lastIndexOf(',');
  
  float x = dataLine.substring(0, firstCommaIndex).toFloat();
  float y = dataLine.substring(firstCommaIndex + 1, lastCommaIndex).toFloat();
  float z = dataLine.substring(lastCommaIndex + 1).toFloat();

  // Calculate yaw (theta) and pitch (phi)
  float r = sqrt(x * x + y * y + z * z);
  float theta = atan2(y, x); // Yaw - rotation around z-axis
  float phi = acos(z / r); // Pitch - rotation around y-axis

  // Convert radians to degrees
  float m1Degrees = theta * (180.0 / PI);
  float m2Degrees = phi * (180.0 / PI);

  // Convert from degrees to steps for stepper motor
  float m1Steps = m1Degrees*200*micro/360
  float m2Steps = m2Degrees*200*micro/360

  // Move servos
  servoM1.write((int)m1Degrees);
  servoM2.write((int)m2Degrees);

  Serial.print("m1: ");
  Serial.print(m1Degrees);
  Serial.print(" degrees, m2: ");
  Serial.print(m2Degrees);
  Serial.println(" degrees");
}


