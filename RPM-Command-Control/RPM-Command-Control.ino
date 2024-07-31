/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/
#define MAX_VELO_RPM 7
#define ACCEL_RAD_S_S 0.2
#define DT_MS 300
#define ANGLE_OF_ATTACK 15
#define SEED 2132138

#define EN_CAN 1
#define EN_IMU 1

#include "AA_MCP2515.h"
#include <Adafruit_LSM6DS3TRC.h>

Adafruit_LSM6DS3TRC lsm6ds3trc;

struct Motor {
  int16_t angle;
  int16_t velocity;
  int16_t current;
  uint8_t temp;

  uint16_t encoder;
  uint16_t encoderRaw;
  int16_t encoderOffset;
};

void printMotor(Motor m, char c = '?');
void setVelocity(uint16_t canID, int32_t velocity_dps_hundreth);
void setAngle(uint16_t canID, int16_t velocity_dps, int16_t angle_deg_hundreth);

// TODO: modify CAN_BITRATE, CAN_PIN_CS(Chip Select) pin, and CAN_PIN_INT(Interrupt) pin as required.
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_500kbps;
const uint8_t CAN_PIN_CS = 10;
const int8_t CAN_PIN_INT = 2;

const float xOffset = 0.6; //-9.89 9.77
const float yOffset = 0.295; //-10.09 9.50
const float zOffset = -0.325; //-9.50 10.15

float integralX = 0;
float integralY = 0;
float integralZ = 0;

CANConfig config(CAN_BITRATE, CAN_PIN_CS, CAN_PIN_INT);
CANController CAN(config);

// uint8_t data[] = { 0xA4, 0x00, 0x3C, 0x00, 0xA0, 0x8C, 0x00, 0x00 };

Motor motorA;
Motor motorB;

Motor motors[2] = {motorA, motorB};

double heading = 0;

String inst;

int innerLoopCount = 0;
int innerLoopCount2 = 0;

const char endCharA = 'A';
const char endCharB = 'B';
const char purge = '_';

float valueA = 0;
float valueB = 0;

int pointCount = 0;

char mode = 'm';

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

bool querySent = 0;
bool commandSent = 0;
bool printSent = 0;

void setup() {
  Serial.begin(115200);

  while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(1000);
  }
  Serial.println("CAN begin OK");

  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  resetMotor(0x141);
  resetMotor(0x142);

  pinMode(LED_BUILTIN, OUTPUT);

  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
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

  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
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

  // lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  // lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2


  randomSeed(SEED);
}

void loop() {
  
  #if EN_IMU
    // Serial.print(",");
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    // lsm6ds3trc.readAcceleration(gotX,gotY,gotZ);
    // Serial.print(";");
  #endif

  if(Serial.available()){
    char inChar = Serial.read();
    // Serial.println("--");
    // Serial.println(inChar);
    // Serial.println((int)inChar);
    // Serial.println("--");
    if(inChar >= 48 && inChar <= 57){
      inst += inChar;
    }else if(inChar == 'p'){
      mode = 'p';
      resetMotor(0x141);
      resetMotor(0x142);
      // setCurrent(0x141,0);
      // setCurrent(0x142,0);
    }else if(inChar == 'a'){
      mode = 'a';
      resetMotor(0x141);
      resetMotor(0x142);
      // setCurrent(0x141,0);
      // setCurrent(0x142,0);

      integralX = 0;
      integralY = 0;
      integralZ = 0;
    }else if(inChar == 'v'){
      mode = 'v';
      resetMotor(0x141);
      resetMotor(0x142);
      // setCurrent(0x141,0);
      // setCurrent(0x142,0);
    }else if(inChar == 'r'){
      mode = ' ';
      resetMotor(0x141);
      resetMotor(0x142);
      // setCurrent(0x141,0);
      // setCurrent(0x142,0);
    }else if(mode == 'p' && inChar == endCharA){
      valueA = atof(inst.c_str());
      setAngleSingle(0x141,MAX_VELO_RPM,(int16_t)(valueA*100));
      inst = "";
    }else if(mode == 'p' && inChar == endCharB){
      valueB = atof(inst.c_str());
      setAngleSingle(0x142,MAX_VELO_RPM,(int16_t)(valueB*100));
      inst = "";
    }else if(mode == 'v' && inChar == endCharA){
      valueA = atof(inst.c_str());
      setVelocity(0x141,(int32_t)(valueA*100));
      inst = "";
    }else if(mode == 'v' && inChar == endCharB){
      valueB = atof(inst.c_str());
      setVelocity(0x142,(int32_t)(valueB*100));
      inst = "";
    }else if(inChar == purge){
      inst = "";
    }
    
  }

  #if EN_CAN
    if(mode == 'a'){
      if(innerLoopCount > DT_MS){
        heading += (random(65536)/65536.0) * ANGLE_OF_ATTACK + (ANGLE_OF_ATTACK/2);
        double heading_rad = heading * 3.14159 / 180;
        setVelocity(0x141,(int32_t)(sin(heading_rad)*MAX_VELO_RPM * 6 * 100));
        setVelocity(0x142,(int32_t)(cos(heading_rad)*MAX_VELO_RPM * 6 * 100));
        // printMotor(motorA,'a');
        // printMotor(motorB,'b');
        innerLoopCount = 0;
        pointCount ++;
        // digitalWrite(LED_BUILTIN, HIGH);
      }else{
        queryMotor(0x141);
        queryMotor(0x142);
        // digitalWrite(LED_BUILTIN, LOW);
      }
      innerLoopCount ++;

    }else{
      queryMotor(0x141);
      queryMotor(0x142);
    }
  #endif

  getFeedback();
  delay(1);

  #if EN_IMU
    if(mode == 'a'){
      integralX += accel.acceleration.x + xOffset;
      integralY += accel.acceleration.y + yOffset;
      integralZ += accel.acceleration.z + zOffset;
    }
  #endif

  if(innerLoopCount2 > 100){
    // Serial.print("Heading:");
    // Serial.print(heading);
    // Serial.print(" A Des:[");
    // Serial.print(sin(heading)*MAX_VELO_RPM);
    // Serial.print("] Act:[");
    // Serial.print(motors[0].velocity/100.0);
    // Serial.print("] B Des:");
    // Serial.print(cos(heading)*MAX_VELO_RPM);
    // Serial.print("] Act:[");
    // Serial.println(motors[1].velocity/100.0);

    Serial.print("[Immediate] [");
    Serial.print(accel.acceleration.x + xOffset);
    Serial.print("] [");
    Serial.print(accel.acceleration.y + yOffset);
    Serial.print("] [");
    Serial.print(accel.acceleration.z + zOffset);

    Serial.print("] [Integral G] X:");
    Serial.print(integralX);
    Serial.print(" Y:");
    Serial.print(integralY);
    Serial.print(" Z:");
    Serial.print(integralZ);

    Serial.print(" | PC:");
    Serial.println(pointCount);

    innerLoopCount2 = 0;
  }
  innerLoopCount2 ++;

  

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
    // frame.print("TX");
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
    // frame.print("TX");
    CAN.write(frame);
}

void queryMotor(uint8_t canID){
  uint8_t data[8] = { 
      0x90, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00};
  CANFrame frame(canID, data, sizeof(data));
  // frame.print("TX");
  CAN.write(frame);
}

void resetMotor(uint8_t canID){
  uint8_t data[8] = { 
      0x76, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00};
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
          if(data[0] == 0x9C){
            Serial.println("9C");
          }
      }else if(data[0] == 0x90){
          motors[id].encoder = (int16_t)(data[2] | (data[3]<<8)); 
          motors[id].encoderRaw = (int16_t)(data[4] | (data[5]<<8));
          motors[id].encoderOffset = (int16_t)(data[6] | (data[7]<<8));
      }else{
          frame.print("WUT");
      }
  }
}