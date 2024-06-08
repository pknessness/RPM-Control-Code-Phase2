#include <AccelStepper.h>
#include <Adafruit_SensorLab.h>
#include <Adafruit_LSM6DS3.h>
#define LSM_CS 10
#define LSM_SCK 13
#define LSM_MISO 12
#define BUFFER_LENGTH 50
#define steptime 360
#define accelX accel.acceleration.x + 0.05
#define accelY accel.acceleration.y + 0.18
#define accelZ accel.acceleration.z + 0.33
//A = X; outer frame
//B = Y; inner frame

#define RUN_PRINT 0
#define DEBUG_PRINT 1

Adafruit_LSM6DS3TRC lsm6ds3trc;

int wp = 0;
int ledPin = 13;  // Most Arduinos have an on-board LED on pin 13.

// Define connections for Motor A
const int DIR_PIN_A = 4;
const int STEP_PIN_A = 5;
const int enPin = 8;

// Define connections for Motor B
const int DIR_PIN_B = 2;
const int STEP_PIN_B = 3;

//Microstepping
const int micro = 64;

// Steps per revolution for your stepper motors
const int stepsPerRevolution = 2 * 200 * micro;

long prevPointA = 0;
long prevPointB = 0;
//float distToGoA = 1000000;
//float distToGoB = 1000000;

int ABcounter = 0;            // Enable alternating A and B readlines
unsigned long steptimer = 0;  // time between steps
unsigned long starttime = 0;  // starting time offset
unsigned long timedelay = 0;

float speedB = 0.0;
float speedA = 0.0;

int iter = 0;
int prevIter = 0;

unsigned long debugDt;
int counter;

// Create instances of the stepper motors
AccelStepper
  stepperA(AccelStepper::DRIVER,STEP_PIN_A, DIR_PIN_A);
AccelStepper
  stepperB(AccelStepper::DRIVER,STEP_PIN_B, DIR_PIN_B);

void setup() {
  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
  }

  // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // Declare pins as Outputs
  pinMode(STEP_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(STEP_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(A0, OUTPUT);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Set the maximum speed and acceleration for the motors
  //stepperA.setMaxSpeed(7 * stepsPerRevolution / 60);
  //stepperA.setSpeed(2*7*stepsPerRevolution/60);
  stepperA.setMaxSpeed(10000);

  // stepperB.setMaxSpeed(7 * stepsPerRevolution / 60);
  //stepperB.setSpeed(2*7*stepsPerRevolution/60);
  stepperB.setMaxSpeed(10000);
  // Start the serial communication
  Serial.begin(115200);
}

String a;
String b;

long bufferA[BUFFER_LENGTH] = { 0 };
long bufferB[BUFFER_LENGTH] = { 0 };

void loop() {

  if (Serial.available()) {
    char inByte = Serial.read();
    if (inByte == ',') {
      ABcounter = 1;
    } else if (inByte == ';') {
      ABcounter = 0;
      bufferA[wp] = 2 * a.toInt(); //Divide micro by whatever microstepping is in csv file
      bufferB[wp] = 2 * b.toInt();
      // Serial.print("b_A: ");
      // Serial.print(bufferA[wp]);
      // Serial.print("b_B: ");
      // Serial.println(bufferB[wp]);
      wp++;
      a = "";
      b = "";
      digitalWrite(A0, HIGH);
      digitalWrite(ledPin, HIGH);
    }

    if ((wp != 0) && (starttime == 0)) {
      starttime = millis();
      timedelay = starttime;
    }

    if ((57 >= inByte && 48 <= inByte)) {
      if (ABcounter == 0) {
        a += inByte;
      } else {
        b += inByte;
      }
    }
  }

  // stepperA.setSpeed(1000 *(bufferA[0] - stepperA.currentPosition()) / dt);

  // stepperB.setSpeed(1000 *(bufferB[0] - stepperB.currentPosition()) / dt);
  speedA = 1000.0 * (bufferA[0]-prevPointA)/steptime;
  speedB = 1000.0 * (bufferB[0]-prevPointB)/steptime;
  stepperA.setSpeed(speedA);

  if ((bufferB[0] - prevPointB) < -170 * micro) {
    // Adjust the velocity to prevent rollover
    stepperB.setSpeed(1000.0 * ((stepsPerRevolution + bufferB[0]) - prevPointB) / steptime);
    // Serial.print("OF FW, new speed: ");
    // Serial.println(stepperB.speed());
    // Serial.print("b_B: ");
    // Serial.print(bufferB[0]);
    // Serial.print(" Calc: ");
    // Serial.println((stepsPerRevolution + bufferB[0]) - prevPointB);
  } else if ((bufferB[0] - prevPointB) > 170 * micro) {
      stepperB.setSpeed(1000.0 * (bufferB[0] - (stepsPerRevolution + prevPointB)) / steptime);
      // Serial.print("OF BW, new speed: ");
      // Serial.println(stepperB.speed());
  } else {
      stepperB.setSpeed(speedB);
  }

  stepperA.runSpeed();
  stepperB.runSpeed();
  
  unsigned long currentMillis = millis();
  iter = (currentMillis - starttime) / steptime;

  if ((iter > prevIter) && (wp != 0)) {
    prevPointA = bufferA[0];
    prevPointB = bufferB[0];

    // Initialize IMU
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    // lsm6ds3trc.getEvent(&accel);
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    // lsm6ds3trc._read();
    // lsm6ds3trc.fillAccelEvent(&accel, millis());
  

    #if RUN_PRINT
      Serial.print(iter);
      Serial.print("\t");

      // Serial.print(bufferA[0]);
      // Serial.print("\t");
      // Serial.print(bufferB[0]);
      // Serial.print("\t");

      Serial.print(accelX); // Print all acceleration info when it reaches next step
      Serial.print("\t");
      Serial.print(accelY);
      Serial.print("\t");
      Serial.print(accelZ);
      Serial.print("\t");

      Serial.print(stepperA.currentPosition());
      Serial.print("\t");
      Serial.println(stepperB.currentPosition());
    #endif

    for (int i = 1; i < BUFFER_LENGTH;
         i++) {
      bufferA[i - 1] = bufferA[i];
      bufferB[i - 1] = bufferB[i];
    }
    //stepperA.moveTo(bufferA[0]);
    //stepperB.moveTo(bufferB[0]);
    wp = wp - 1;
  }

  #if DEBUG_PRINT
    if(counter > 100){
      Serial.print(millis());
      Serial.print("ms ");
      Serial.print(millis() - debugDt);
      Serial.print("dt A:");
      Serial.print(stepperA.speed());
      Serial.print("sps B:");
      Serial.print(stepperB.speed());
      Serial.println("sps ");
      counter = 0;
    }
    counter ++;
  #endif
  debugDt = millis();

  if ((wp < (BUFFER_LENGTH - 5) && millis() - timedelay > 100)) {

    // Serial.println("---");
    
    // // Serial.print(" b_A:");
    // // Serial.println(bufferA[0]);
    // Serial.print(" b_B[");
    // Serial.print(bufferB[0]);
    // Serial.print("] - p_B[");
    // Serial.print(prevPointB);
    // // Serial.print(" s_A:");
    // // Serial.println(stepperA.speed());

    // Serial.print(" speed_B:");
    // Serial.println(speedB);

    // Serial.print(" s_A:");
    // Serial.print(stepperA.speed());
    // Serial.print(" s_B:");
    // Serial.print(stepperB.speed());
    Serial.println("--");
    timedelay = millis();
  }

  if ((wp == 0) && (millis() - timedelay > 100)) {
    Serial.println('Trajectory Complete Thank the Lord');
  }

  prevIter = iter;
}