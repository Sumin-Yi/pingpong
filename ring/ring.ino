//#include "TinyIRSender.hpp"
//#include <LSM6DS3.h>
//#include <Wire.h>
//#include <ArduinoBLE.h>  // Bluetooth library
//
//
//// Create an instance of class LSM6DS3
//LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A
//
////Madgwick filter;
//
//BLEService customService("1101"); // Custom BLE service
//BLEStringCharacteristic rxCharacteristic("2101",  BLERead | BLENotify, 20);
//// BLEStringCharacteristic txCharacteristic("2A38", BLEWrite | BLENotify, 20);
//
//#define THRESHOLD_ACCEL_Y 3.0   // Y축 가속도 기준 (단위: g)
//#define THRESHOLD_GYRO_X 150    // X축 자이로 기준 (단위: dps)
//
//bool is_paired = false;
//
//// void onWriteCallback(BLEDevice central, BLECharacteristic characteristic) {
////   String receivedData = rxCharacteristic.value();
////   Serial.print("서버로부터 받은 데이터: ");
////   Serial.println(receivedData);
//
////   if(receivedData.equals("PAIRED"))
////     is_paired = true;
//  
//// }
//
//void setup(void)
//{
//  Serial.begin(115200);
//
//  pinMode(7, OUTPUT);
//  pinMode(LED_BUILTIN, OUTPUT);
//
//  if (myIMU.begin() != 0) {
//    Serial.println("Device error");
//  } else {
////    Serial.println("aX,aY,aZ,gX,gY,gZ");
//  }
//
//  Wire1.setClock(400000UL);  //SCL 400kHz
//    
//  // change defalt settings, refer to data sheet 9.13, 9.14, 9.19, 9.20
//  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x1C);    // 12.5Hz 2000dps
//  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x1A);   // 12.5Hz 4G 
//  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);    // HPF 16mHz
//  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);   // ODR/4
//
////  filter.begin(12.5);
//
//  // Initialize BLE
//  if (!BLE.begin()) {
//    Serial.println("Failed to initialize BLE!");
//    while (1);
//  }
//
//  BLE.setLocalName("ring");
//  BLE.setAdvertisedService(customService);
//
//  // Add BLE characteristics
//  
//  customService.addCharacteristic(rxCharacteristic);
//  // customService.addCharacteristic(txCharacteristic);
//
//  BLE.addService(customService);
//
//
//  //rxCharacteristic.setEventHandler(BLEWritten, onWriteCallback);
//
//  // Start advertising
//  BLE.advertise();
//  Serial.println("Bluetooth device is now advertising...");
//
//  Serial.println("Wating...");
//
//  // while(!is_paired)
//  // {
//  //   Serial.println("YAHO!");
//  //   if(BLE.connected())
//  //   {
//  //     Serial.println("YAp!");
//  //     BLE.poll();
//  //   }
//    
//  // }
//
//}
//
//#define THRESHOLD 40.
//#define THRESHOLDACCEL 0.8
//#define STABILITY_T 300
//#define SIGNAL_DELAY 100
////#define THRESHOLD_ACCEL_X 0.9   // Y축 가속도 기준 (단위: g)
//#define THRESHOLD_GYRO_Y_LOWER 300    // X축 자이로 기준 (단위: dps)
////#define THRESHOLD_GYRO_Y_UPPER 50    // X축 자이로 기준 (단위: dps)
//
//uint8_t sAddress = 0x02;
//uint8_t sCommand = 0x10;
//uint8_t sRepeats = 1;
//
//// bool is_rising = false;  // 상승 상태 플래그
//// bool detection_enabled = false;
//// unsigned long last_rising_time = 0;  // 마지막 상승 시간
//// #define TIMEOUT 500  // 상승 후 하강 대기 시간 (ms)
//
//bool detect_finger_tap() {
//            
////  float accelX = myIMU.readFloatAccelX();  // Y축 가속도 (단위: g)
//  float gyroY = myIMU.readFloatGyroY();    // X축 자이로 (단위: dps)
//
////  float accelY = myIMU.readFloatAccelY();  // Y축 가속도 (단위: g)
////  float gyroX = myIMU.readFloatGyroX();    // X축 자이로 (단위: dps)
////  float accelZ = myIMU.readFloatAccelZ();  // Z축 가속도 (단위: g)
////  float gyroY = myIMU.readFloatGyroY();    // Y축 자이로 (단위: dps)
////  float accelX = myIMU.readFloatAccelX();  // X축 가속도 (단위: g)
////  float gyroZ = myIMU.readFloatGyroZ();    // Z축 자이로 (단위: dps)
//
//  // 디버그 출력 (필요할 경우 주석 처리 가능)
////  Serial.print("Accel Y: ");
////  Serial.print(accelY, 2);
////  Serial.print(" g, Gyro X: ");
////  Serial.print(gyroX, 2);
////  Serial.print(", Accel Z: ");
////  Serial.print(accelZ, 2);
////  Serial.print(" g, Gyro Y: ");
////  Serial.print(gyroY, 2);
////  Serial.print(", Accel X: ");
////  Serial.print(accelX, 2);
////  Serial.print(" g, Gyro Z: ");
////  Serial.println(gyroZ, 2);
//  
////  // debug
////  Serial.print("Accel Y: ");
////  Serial.print(accelY, 2);
////  Serial.print(" g, Gyro X: ");
////  Serial.print(gyroX, 2);
//
//  if (abs(gyroY) > THRESHOLD_GYRO_Y_LOWER) {
//    Serial.println("Upward Finger Tap Detected");
//    return true;  // Tap 감지
//  }
//
//  return false;  // Tap 미감지
//
//}
//
//long now = millis();
//long last_time = now;
//unsigned long finger_time;
//
//unsigned long lastTapTime = 0;  // 마지막 탭 발생 시간
//const unsigned long debounceDelay = 600;
//
//unsigned long roop_time = 0;
//void loop(void)
//{
//  roop_time = millis();
//  long last_time_since_signal = now;
//  digitalWrite(LED_BUILTIN, HIGH);
//
//  BLEDevice central = BLE.central();
//
//
//  if (central) {
//    if (detect_finger_tap()) {
//      unsigned long currentTime = millis();  // 현재 시각
//  
//      if (currentTime - lastTapTime > debounceDelay) {
//        lastTapTime = currentTime;  // 현재 시각으로 업데이트
//  
//        // 신호 전송
//        rxCharacteristic.writeValue("FINGER_TAP_DETECTED");
//        Serial.println("Sent signal: FINGER_TAP_DETECTED");
//      }
//    }
//
//    do {
//      float aX, aY, aZ;
//      now = millis();
//      aX = myIMU.readFloatGyroX();
//      aY = myIMU.readFloatGyroY();
//      aZ = myIMU.readFloatGyroZ();
//  
//      // sum up the absolutes
//      float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
//  
//      if (aSum > THRESHOLD) {
//        last_time = now;
//      }
//      if (detect_finger_tap()) {
//        unsigned long currentTime = millis();  // 현재 시각
//    
//        if (currentTime - lastTapTime > debounceDelay) {
//          lastTapTime = currentTime;  // 현재 시각으로 업데이트
//    
//          // 신호 전송
//          rxCharacteristic.writeValue("FINGER_TAP_DETECTED");
//          Serial.println("Sent signal: FINGER_TAP_DETECTED");
//        }
//      }
//
//    //Serial.println(aSum);
//    } while (now - last_time < STABILITY_T || now - last_time_since_signal <= SIGNAL_DELAY);
//
//  digitalWrite(LED_BUILTIN, LOW);
//  sendFAST(D7, sCommand, sRepeats);
//
//  // ax = myIMU.readFloatGyroX();
//  // ay = myIMU.readFloatGyroY();
//  // az = myIMU.readFloatGyroZ();
//  // gx = myIMU.readFloatGyroX();  // Gyro data
//  // gy = myIMU.readFloatGyroY();
//  // gz = myIMU.readFloatGyroZ();
//
//  // // calculate the attitude with Madgwick filter
//  // filter.updateIMU(gx, gy, gz, ax, ay, az);
//
//  // lastRoll = filter.getRoll();
//  // lastPitch = filter.getPitch();  // -180 ~ 180deg
//  // lastYaw = filter.getYaw();
//
//  }
//
//  
// 
//}

#include "TinyIRSender.hpp"
#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>  // Bluetooth library

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

BLEService customService("1101"); // Custom BLE service
BLEStringCharacteristic rxCharacteristic("2101",  BLERead | BLENotify, 20);

void setup(void)
{
  Serial.begin(115200);

  pinMode(7, OUTPUT);

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("ring");
  BLE.setAdvertisedService(customService);

  // Add BLE characteristics
  
  customService.addCharacteristic(rxCharacteristic);
  // customService.addCharacteristic(txCharacteristic);

  BLE.addService(customService);


  //rxCharacteristic.setEventHandler(BLEWritten, onWriteCallback);

  // Start advertising
  BLE.advertise();
  Serial.println("Bluetooth device is now advertising...");

  Serial.println("Wating...");
  
}


#define THRESHOLD 40.
#define STABILITY_T 300
#define SIGNAL_DELAY 100
#define THRESHOLD_GYRO_Y_LOWER 300    // X축 자이로 기준 (단위: dps)

uint8_t sAddress = 0x02;
uint8_t sCommand = 0x10;
uint8_t sRepeats = 1;


bool timer(long *last, long threshold) {
  long now = millis();
  if (now - *last >= threshold) {
    *last = now;
    return true;
  }

  return false;
}

bool detect_finger_tap() {
  float gyroY = myIMU.readFloatGyroY();

  if (abs(gyroY) > THRESHOLD_GYRO_Y_LOWER) {
    Serial.println("Upward Finger Tap Detected");
    return true;
  }
  return false;
}

long now = millis();
long last_time = now;

unsigned long lastTapTime = 0;
const unsigned long debounceDelay = 600;

void loop(void)
{
  BLEDevice central = BLE.central();
  
  long last_time_since_signal = now;
  digitalWrite(LED_BUILTIN, HIGH);

  if (detect_finger_tap()) {
  unsigned long currentTime = millis();  // 현재 시각

    if (currentTime - lastTapTime > debounceDelay) {
      lastTapTime = currentTime;  // 현재 시각으로 업데이트
  
      // 신호 전송
      rxCharacteristic.writeValue("FINGER_TAP_DETECTED");
      Serial.println("Sent signal: FINGER_TAP_DETECTED");
    }
  }

  do {

    float aX, aY, aZ;
    now = millis();
    aX = myIMU.readFloatGyroX();
    aY = myIMU.readFloatGyroY();
    aZ = myIMU.readFloatGyroZ();
    

    // sum up the absolutes
    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

    if (aSum > THRESHOLD) {
      last_time = now;
    }

    if (detect_finger_tap()) {
      unsigned long currentTime = millis();  // 현재 시각
    
      if (currentTime - lastTapTime > debounceDelay) {
        lastTapTime = currentTime;  // 현재 시각으로 업데이트
    
        // 신호 전송
        rxCharacteristic.writeValue("FINGER_TAP_DETECTED");
        Serial.println("Sent signal: FINGER_TAP_DETECTED");
      }
    }
  
  //    Serial.println(aSum);

  } while (now - last_time_since_signal <= SIGNAL_DELAY);

  digitalWrite(LED_BUILTIN, LOW);
  sendFAST(D7, sCommand, sRepeats); 
}
