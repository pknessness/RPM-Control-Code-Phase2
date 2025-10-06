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
// #include <MemoryFree.h>
#include <avr/wdt.h>

#define TIMING_CYCLE 60
#define TIMING_TOLERANCE 8

#define SERIAL_TIMING 10
#define IMU_TIMING 40
#define PRINT_TIMING 50

#define DT_CYCLES DT_MS/TIMING_CYCLE

Adafruit_LSM6DS3TRC lsm6ds3trc;

const float xOffset = 0.06+0.253; //-9.89 9.77 
const float yOffset = 0.295+0.125; //-10.09 9.50
const float zOffset = -0.325-0.083; //-9.50 10.15

//after alignment it is 
//-9.85 0.13 -0.02 and 9.84 0.16 -0.02
//-0.48 9.77 -0.10 and 0.26 -9.80 0.05
//0.02 0.01 9.81 and -0.05 -0.01 -9.85

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
  Serial1.begin(115200);

  while (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    delay(1000);
  }

  Serial.println("LSM6DS3TR-C Found!");

  pinMode(7, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  // lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_52_HZ);

  wdt_enable(WDTO_1S);

  // lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  // lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop() {

  wdt_reset();
  
  long time = millis();
  int cyclePosition = time%TIMING_CYCLE;

  if(cyclePosition > SERIAL_TIMING && cyclePosition <= SERIAL_TIMING + TIMING_TOLERANCE){
    if(Serial1.available()){
      Serial.write(Serial1.read());
    }
    if(Serial.available()){
      char inChar = Serial.read();
      Serial1.write(inChar);
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
      // else if(inChar == 'm'){
      //   Serial.print("freeMemory()=");
      //   Serial.println(freeMemory());
      // }
    }
  }else if(cyclePosition > IMU_TIMING && cyclePosition <= IMU_TIMING + TIMING_TOLERANCE){
    #if EN_IMU
    if(!imuRecv){
      // Serial.print("1");
      if(!lsm6ds3trc.getEvent(&accel, &gyro, &temp)){
        Serial.println("DEATH");
        accelX = 0;
        accelY = 0;
        accelZ = 0;
      }else{
        // Serial.print("2");
        accelX = accel.acceleration.x + xOffset;
        accelY = accel.acceleration.y + yOffset;
        accelZ = accel.acceleration.z + zOffset;
        imuRecv = 1;
      }
      // Serial.println("3");
    }
    #endif
  }else if(cyclePosition > PRINT_TIMING && cyclePosition <= PRINT_TIMING + TIMING_TOLERANCE){
    if(!printSent){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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