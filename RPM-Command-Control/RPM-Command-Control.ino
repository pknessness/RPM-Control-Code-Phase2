/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/
#define MAX_VELO_RPM 8
#define ACCEL_RAD_S_S 0.2
#define DT_MS 1500
#define ANGLE_OF_ATTACK 10
#define SEED 2132138

#include "AA_MCP2515.h"

struct Motor {
  int16_t angle;
  int16_t velocity;
  int16_t current;
  uint8_t temp;
};

void printMotor(Motor m, char c = '?');
void setVelocity(uint16_t canID, int32_t velocity_dps_hundreth);
void setAngle(uint16_t canID, int16_t velocity_dps, int16_t angle_deg_hundreth);

// TODO: modify CAN_BITRATE, CAN_PIN_CS(Chip Select) pin, and CAN_PIN_INT(Interrupt) pin as required.
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_PIN_CS = 53;
const int8_t CAN_PIN_INT = 2;

CANConfig config(CAN_BITRATE, CAN_PIN_CS, CAN_PIN_INT);
CANController CAN(config);

// uint8_t data[] = { 0xA4, 0x00, 0x3C, 0x00, 0xA0, 0x8C, 0x00, 0x00 };

uint8_t controlCode = 0xA2;
uint16_t maxSpeed = 100;

uint16_t angleA = 0;
uint8_t reverseA = 0;

uint16_t angleB = 0;
uint8_t reverseB = 0;

Motor motorA;
Motor motorB;

Motor motors[2] = {motorA, motorB};

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

  setCurrent(0x141,0);
  setCurrent(0x142,0);


  randomSeed(SEED);
}

void loop() {

  // if(Serial.available()){
  //   char inChar = Serial.read();
  //   // Serial.println("--");
  //   // Serial.println(inChar);
  //   // Serial.println((int)inChar);
  //   // Serial.println("--");
  //   if(inChar >= 48 && inChar <= 57){
  //     inst += inChar;
  //   }else if(inChar == 'q'){
  //     reverseA = !reverseA;
  //   }else if(inChar == 'e'){
  //     reverseB = !reverseB;
  //   }else if(inChar == endCharA){
  //     angleA = atoi(inst.c_str());

  //     Serial.print("new angle: ");
  //     Serial.println(angleA);
  //     setVelocity(0x141, angleA);

  //     inst = "";
  //   }else if(inChar == endCharB){
  //     angleB = atoi(inst.c_str());
  //     // Serial.print("new angleB: ");
  //     // Serial.println(angleB);

  //     inst = "";
  //   }else if(inChar == purge){
  //     inst = "";
  //   }
    
  // }


  if(innerLoopCount > DT_MS){
    heading += (random(65536)/65536.0) * ANGLE_OF_ATTACK + (ANGLE_OF_ATTACK/2);
    double heading_rad = heading * 3.14159 / 180;
    setVelocity(0x141,(int32_t)(sin(heading_rad)*MAX_VELO_RPM * 6 * 100));
    setVelocity(0x142,(int32_t)(cos(heading_rad)*MAX_VELO_RPM * 6 * 100));
    // printMotor(motorA,'a');
    // printMotor(motorB,'b');
    innerLoopCount = 0;
  }else{
    queryMotor(0x141);
    queryMotor(0x142);
  }
  innerLoopCount ++;

  getFeedback();

  Serial.print("Heading:");
  Serial.print(heading);
  Serial.print(" A Des:[");
  Serial.print(sin(heading)*MAX_VELO_RPM);
  Serial.print("] Act:[");
  Serial.print(motors[0].velocity/100.0);
  Serial.print("] B Des:");
  Serial.print(cos(heading)*MAX_VELO_RPM);
  Serial.print("] Act:[");
  Serial.println(motors[1].velocity/100.0);

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

void setVelocity(uint16_t canID, int32_t velocity_dps_hundreth){
    uint8_t data[8] = { 
        0xA2, 
        0x00, 
        0x00, 
        0x00, 
        (uint8_t)(velocity_dps_hundreth), 
        (uint8_t)(velocity_dps_hundreth>>8), 
        (uint8_t)(velocity_dps_hundreth>>16), 
        (uint8_t)(velocity_dps_hundreth>>24)};

    // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    CANFrame frame(canID, data, sizeof(data));
    // frame.print("TX");
    CAN.write(frame);
}

void setAngleSingle(uint16_t canID, int16_t velocity_dps, int16_t angle_deg_hundreth){
    uint8_t data[8] = { 
        0xA6, 
        0x00, 
        (uint8_t)(velocity_dps), 
        (uint8_t)(velocity_dps>>8), 
        (uint8_t)(angle_deg_hundreth), 
        (uint8_t)(angle_deg_hundreth>>8), 
        0x00, 
        0x00};

    // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    CANFrame frame(canID, data, sizeof(data));
    frame.print("TX");
    CAN.write(frame);
}

void setCurrent(uint16_t canID, int16_t current_hundreth){
    uint8_t data[8] = { 
        0xA1, 
        0x00, 
        0x00, 
        0x00, 
        (uint8_t)(current_hundreth), 
        (uint8_t)(current_hundreth>>8), 
        0x00, 
        0x00};

    // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
    CANFrame frame(canID, data, sizeof(data));
    frame.print("TX");
    CAN.write(frame);
}

void queryMotor(uint8_t canID){
  uint8_t data[8] = { 
      0x9C, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00};

  // uint8_t data[8] = {0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
  CANFrame frame(canID, data, sizeof(data));
  // frame.print("TX");
  CAN.write(frame);
}

void getFeedback(){
  uint8_t data[8] = {0,0,0,0,0,0,0,0};
  CANFrame frame(0x140,data,8);
  while (CAN.read(frame) == CANController::IOResult::OK) {
      frame.getData(data, 8);
      uint16_t id = frame.getId()-0x241;
      if(data[0] == 0xA1 || data[0] == 0xA2 || data[0] == 0xA6 || data[0] == 0x9C){
          motors[id].angle = (int16_t)(data[6] | (data[7]<<8)); 
          motors[id].velocity = (int16_t)(data[4] | (data[5]<<8));
          motors[id].current = (int16_t)(data[2] | (data[3]<<8));
          motors[id].temp = data[1];
      }else{
          frame.print("WUT");
      }
  }
}