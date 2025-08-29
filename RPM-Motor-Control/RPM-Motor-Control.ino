/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/
#define MAX_VELO_RPM 15
#define ACCEL_RAD_S_S 0.2
#define DT_MS 300
#define ANGLE_OF_ATTACK 15
#define SEED 2132138

#define EN_CAN 1

#include "AA_MCP2515.h"

#define TIMING_CYCLE 100
#define TIMING_TOLERANCE 10

#define SERIAL_TIMING 0
#define COMMAND_TIMING 20
#define QUERY_TIMING 40
#define PRINT_TIMING 80

#define DT_CYCLES DT_MS/TIMING_CYCLE

enum MotorProfile{
  MOTOR_BOTH_OFF, //0
  MOTOR_BOTH_RANDOM, //1
  MOTOR_1_CONSTANT, //2 // for the pig on a stick
  MOTOR_BOTH_CONSTANT //3
}; 

MotorProfile currentProfile = MOTOR_BOTH_RANDOM;

void setMotorProfile(MotorProfile profile);


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

CANConfig config(CAN_BITRATE, CAN_PIN_CS, CAN_PIN_INT);
CANController CAN(config);

Motor motorA;
Motor motorB;

Motor motors[2] = {motorA, motorB};

double heading = 0;

// String inst;

int commandLoop = 0;
int printLoopCount = 0;

const char endCharA = 'A';
const char endCharB = 'B';
const char purge = '_';

float valueA = 0;
float valueB = 0;

int pointCount = 0;

char mode = 'm';

bool serialRead = 0;
bool commandSent = 0;
bool querySent = 0;
bool imuRecv = 0;
bool printSent = 0;

void setup() {
  Serial.begin(115200);

  while(CAN.begin(CANController::Mode::Normal) != CANController::OK) {
    Serial.println("CAN begin FAIL - delaying for 1 second");
    delay(1000);
  }
  Serial.println("CAN begin OK");

  Serial.println("LSM6DS3TR-C Found!");

  resetMotor(0x141);
  resetMotor(0x142);

  pinMode(7, OUTPUT);
  // lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  // lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2


  randomSeed(SEED);
}

void loop() {
  
  long time = millis();
  int cyclePosition = time%TIMING_CYCLE;

  if(cyclePosition > SERIAL_TIMING && cyclePosition <= SERIAL_TIMING + TIMING_TOLERANCE){
    if(Serial.available() > 0){
      char inChar = Serial.read();
      switch (inChar) {
        case '0':
          setMotorProfile(MOTOR_BOTH_OFF);
          break;
        case '1':
          setMotorProfile(MOTOR_BOTH_RANDOM);
          break;
        case '2':
          setMotorProfile(MOTOR_1_CONSTANT);
          break;
        case '3':
          setMotorProfile(MOTOR_BOTH_CONSTANT);
          break;
        default :
          setMotorProfile(currentProfile);
          break;
      }
      // Serial.println("--");
      // Serial.println(inChar);
      // Serial.println((int)inChar);
      // Serial.println("--");
      if(inChar >= 48 && inChar <= 57){
        // inst += inChar;
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
        // valueA = atof(inst.c_str());
        // setAngleSingle(0x141,MAX_VELO_RPM,(int16_t)(valueA*100));
        // inst = "";
      }else if(mode == 'p' && inChar == endCharB){
        // valueB = atof(inst.c_str());
        // setAngleSingle(0x142,MAX_VELO_RPM,(int16_t)(valueB*100));
        // inst = "";
      }else if(mode == 'v' && inChar == endCharA){
        // valueA = atof(inst.c_str());
        // setVelocity(0x141,(int32_t)(valueA*100));
        // inst = "";
      }else if(mode == 'v' && inChar == endCharB){
        // valueB = atof(inst.c_str());
        // setVelocity(0x142,(int32_t)(valueB*100));
        // inst = "";
      }else if(inChar == purge){
        // inst = "";
      }else if(inChar == 'd'){
        Serial.print("#");
        Serial.print(motors[0].encoder);
        Serial.print("=");
        Serial.print(motors[1].encoder);
        Serial.print("=");
      }
    }
  }else if(cyclePosition > COMMAND_TIMING && cyclePosition <= COMMAND_TIMING + TIMING_TOLERANCE){
    if(!commandSent){
      if(mode == 'a'){
        if(commandLoop > DT_CYCLES){
          // off by default
          if (currentProfile == MOTOR_BOTH_OFF) {
            setVelocity(0x141,0);
            setVelocity(0x142,0);
          }
          else if (currentProfile == MOTOR_BOTH_RANDOM) {
            heading += (random(65536)/65536.0) * ANGLE_OF_ATTACK - (ANGLE_OF_ATTACK/2);
            double heading_rad = heading * 3.14159 / 180;
            setVelocity(0x141,(int32_t)(sin(heading_rad)*MAX_VELO_RPM * 6 * 100));
            setVelocity(0x142,(int32_t)(cos(heading_rad)*MAX_VELO_RPM * 6 * 100));
          }

          else if (currentProfile == MOTOR_1_CONSTANT) {
            setVelocity(0x141,MAX_VELO_RPM * 6 * 100);
            setVelocity(0x142,0);  
          }

          else if (currentProfile == MOTOR_BOTH_CONSTANT) {
            setVelocity(0x141,MAX_VELO_RPM * 6 * 100);
            setVelocity(0x142,MAX_VELO_RPM * 6 * 100);  
          }
          // printMotor(motorA,'a');
          // printMotor(motorB,'b');
          commandLoop = 0;
          pointCount ++;
          digitalWrite(7, HIGH);
        }else{
          queryMotor(0x141);
          queryMotor(0x142);
          digitalWrite(7, LOW);
        }
        commandLoop ++;
      }else{
        queryMotor(0x141);
        queryMotor(0x142);
      }
      commandSent = 1;
    }
  }else if(cyclePosition > QUERY_TIMING && cyclePosition <= QUERY_TIMING + TIMING_TOLERANCE){
    if(!querySent){
      queryMotorEncoder(0x141);
      queryMotorEncoder(0x142);
      querySent = 1;
    }
  }else if(cyclePosition > PRINT_TIMING && cyclePosition <= PRINT_TIMING + TIMING_TOLERANCE){
    if(!printSent){
      if(printLoopCount > 1){

        double heading_rad = heading * 3.14159 / 180;
        // Serial.print("Heading:");
        // Serial.print(heading_rad);
        // Serial.print(" A Des:[");
        // Serial.print(sin(heading_rad)*MAX_VELO_RPM);
        // Serial.print("] Act:[");
        // Serial.print(motors[0].velocity/10.0);
        // Serial.print("] B Des:");
        // Serial.print(cos(heading_rad)*MAX_VELO_RPM);
        // Serial.print("] Act:[");
        // Serial.print(motors[1].velocity/10.0);

        // Serial.print("[Immediate] [");
        // Serial.print(accel.acceleration.x + xOffset);
        // Serial.print("] [");
        // Serial.print(accel.acceleration.y + yOffset);
        // Serial.print("] [");
        // Serial.print(accel.acceleration.z + zOffset);

        // Serial.print("] [Integral G] X:");
        // Serial.print(accelX);
        // Serial.print(" Y:");
        // Serial.print(accelY);
        // Serial.print(" Z:");
        // Serial.print(accelZ);

        // Serial.print(" | PC:");
        // Serial.println(pointCount);

        printLoopCount = 0;
      }
      printLoopCount ++;
      printSent = 1;
    }
  }else{
    serialRead = 0;
    commandSent = 0;
    querySent = 0;
    printSent = 0;
  }

  getFeedback();
}


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

void queryMotor(uint16_t canID){
  sendCommand(canID, 0x9C);
}

void queryMotorEncoder(uint16_t canID){
  sendCommand(canID, 0x90);
}

void resetMotor(uint16_t canID){
  sendCommand(canID, 0x76);
}

void sendCommand(uint16_t canID, uint8_t command){
  uint8_t data[8] = { 
      command, 
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
      if(id == 0 || id == 1){
        if(data[0] == 0xA1 || data[0] == 0xA2 || data[0] == 0xA6 || data[0] == 0x9C){
            motors[id].angle = (int16_t)(data[6] | (data[7]<<8)); 
            motors[id].velocity = (int16_t)(data[4] | (data[5]<<8));
            motors[id].current = (int16_t)(data[2] | (data[3]<<8));
            motors[id].temp = data[1];
            // if(data[0] == 0x9C){
            //   Serial.println("9C");
            // }
        }else if(data[0] == 0x90){
            motors[id].encoder = (int16_t)(data[2] | (data[3]<<8)); 
            motors[id].encoderRaw = (int16_t)(data[4] | (data[5]<<8));
            motors[id].encoderOffset = (int16_t)(data[6] | (data[7]<<8));
            // frame.print("90");
        }else{
            // frame.print("WUT");
        }
      }
      digitalWrite(7,HIGH);
  }
}

void setMotorProfile(MotorProfile profile) {
  currentProfile = profile;
}
