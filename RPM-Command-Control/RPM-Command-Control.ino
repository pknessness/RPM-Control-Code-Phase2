/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/

#include "AA_MCP2515.h"

struct Motor {
  uint16_t angle;
  uint16_t velocity;
  uint16_t current;
  uint8_t temp;
};

void printMotor(Motor m, char c = '?');

// TODO: modify CAN_BITRATE, CAN_PIN_CS(Chip Select) pin, and CAN_PIN_INT(Interrupt) pin as required.
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_PIN_CS = 53;
const int8_t CAN_PIN_INT = 2;

CANConfig config(CAN_BITRATE, CAN_PIN_CS, CAN_PIN_INT);
CANController CAN(config);

// uint8_t data[] = { 0xA4, 0x00, 0x3C, 0x00, 0xA0, 0x8C, 0x00, 0x00 };

uint8_t controlCode = 0xA6;
uint16_t maxSpeed = 100;

uint16_t angleA = 0;
uint8_t reverseA = 0;

uint16_t angleB = 0;
uint8_t reverseB = 0;

Motor motorA;
Motor motorB;

double heading = 0;

String inst;

int innerLoopCount = 0;

const char endCharA = 'A';
const char endCharB = 'B';
const char purge = '_';

void setup() {
  Serial.begin(115200);

  while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(1000);
  }
  Serial.println("CAN begin OK");
}

void loop() {

  if(Serial.available()){
    char inChar = Serial.read();
    Serial.println("--");
    Serial.println(inChar);
    Serial.println((int)inChar);
    Serial.println("--");
    if(inChar >= 48 && inChar <= 57){
      inst += inChar;
    }else if(inChar == 'q'){
      reverseA = !reverseA;
    }else if(inChar == 'e'){
      reverseB = !reverseB;
    }else if(inChar == endCharA){
      angleA = atoi(inst.c_str());
      Serial.print("new angleA: ");
      Serial.println(angleA);
      inst = "";
    }else if(inChar == endCharB){
      angleB = atoi(inst.c_str());
      Serial.print("new angleB: ");
      Serial.println(angleB);
      inst = "";
    }else if(inChar == purge){
      inst = "";
    }
  }

  if(innerLoopCount > 200){

    uint8_t dataA[8] = { 
      controlCode, 
      reverseA, (uint8_t)maxSpeed, 
      (uint8_t)(maxSpeed>>8), 
      (uint8_t)(angleA), 
      (uint8_t)(angleA>>8), 
      0x00, 
      0x00};

    uint8_t dataB[8] = { 
      controlCode, 
      reverseB, (uint8_t)maxSpeed, 
      (uint8_t)(maxSpeed>>8), 
      (uint8_t)(angleB), 
      (uint8_t)(angleB>>8), 
      0x00, 
      0x00};

    // transmit A
    CANFrame frameA(0x141, dataA, sizeof(dataA));
    CAN.write(frameA);
    frameA.print("CAN _A_ TX");
    if (CAN.read(frameA) == CANController::IOResult::OK) {
      frameA.print("_A_ RX");
      frameA.getData(dataA,8);
      motorA.angle = dataA[6] | (dataA[7]<<8);
      motorA.velocity = dataA[4] | (dataA[5]<<8);
      motorA.current = dataA[2] | (dataA[3]<<8);
      motorA.temp = dataA[1];
      printMotor(motorA);
    }

    // transmit B
    CANFrame frameB(0x142, dataB, sizeof(dataB));
    CAN.write(frameB);
    frameB.print("_B_ CAN TX");
    if (CAN.read(frameB) == CANController::IOResult::OK) {
      frameB.print("_B_ RX");
    }

    // modify data to simulate updated data from sensor, etc
    // angleA += 1000;

    innerLoopCount = 0;
  }
  innerLoopCount ++;
  

  delay(1);
}

// void returnFrame(CANFrame frame){
//   print
// }

void printMotor(Motor m, char c = '?'){
  Serial.print("Motor ");
  Serial.print(c);
  Serial.print("| Angle: ");
  Serial.print(m.angle);
  Serial.print("| Velocity");
  Serial.print(m.velocity);
  Serial.print("| Temp");
  Serial.println(m.temp);
}
