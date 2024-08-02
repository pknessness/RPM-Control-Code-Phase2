/*
  CAN Send Example

  This will setup the CAN controller(MCP2515) to send CAN frames.
  Transmitted frames will be printed to the Serial port.
  Transmits a CAN standard frame every 2 seconds.

  MIT License
  https://github.com/codeljo/AA_MCP2515
*/

#define EN_IMU 1

#include <Adafruit_LSM6DS3TRC.h>

#define TIMING_CYCLE 100
#define TIMING_TOLERANCE 10

#define SERIAL_TIMING 10
#define IMU_TIMING 60
#define PRINT_TIMING 80

#define DT_CYCLES DT_MS/TIMING_CYCLE

Adafruit_LSM6DS3TRC lsm6ds3trc;

const float xOffset = 0.6; //-9.89 9.77
const float yOffset = 0.295; //-10.09 9.50
const float zOffset = -0.325; //-9.50 10.15

float accelX = 0;
float accelY = 0;
float accelZ = 0;

int printLoopCount = 0;

char mode = 'm';

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

bool serialRead = 0;
bool imuRecv = 0;
bool printSent = 0;

void setup() {
  Serial.begin(115200);

  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  pinMode(7, OUTPUT);

  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);

  // lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  // lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop() {
  
  long time = millis();
  int cyclePosition = time%TIMING_CYCLE;

  if(cyclePosition > SERIAL_TIMING && cyclePosition <= SERIAL_TIMING + TIMING_TOLERANCE){
    if(Serial.available()){
      char inChar = Serial.read();
      // Serial.println("--");
      // Serial.println(inChar);
      // Serial.println((int)inChar);
      // Serial.println("--");
      if(inChar == 'd'){
        Serial.print("+");
        Serial.print(accelX);
        Serial.print("=");
        Serial.print(accelY);
        Serial.print("=");
        Serial.print(accelZ);
        Serial.print("=");
      }
    }
  }else if(cyclePosition > IMU_TIMING && cyclePosition <= IMU_TIMING + TIMING_TOLERANCE){
    #if EN_IMU
    if(!imuRecv){
      lsm6ds3trc.getEvent(&accel, &gyro, &temp);
      accelX = accel.acceleration.x + xOffset;
      accelY = accel.acceleration.y + yOffset;
      accelZ = accel.acceleration.z + zOffset;
      imuRecv = 1;
    }
    #endif
  }else if(cyclePosition > PRINT_TIMING && cyclePosition <= PRINT_TIMING + TIMING_TOLERANCE){
    if(!printSent){
      if(printLoopCount > 1){
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
    imuRecv = 0;
    printSent = 0;
  }
}