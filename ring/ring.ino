#include <LSM6DS3.h>
#include <Wire.h>
#include <ArduinoBLE.h>  // Bluetooth library

// Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    // I2C device address 0x6A

BLEService customService("1101"); // Custom BLE service
BLEStringCharacteristic rxCharacteristic("2101",  BLERead | BLENotify, 20);
// BLEStringCharacteristic txCharacteristic("2A38", BLEWrite | BLENotify, 20);

#define IR_SEND_PIN 7

void setup(void)
{
  Serial.begin(115200);

  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
//    Serial.println("aX,aY,aZ,gX,gY,gZ");
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

  // while(!is_paired)
  // {
  //   Serial.println("YAHO!");
  //   if(BLE.connected())
  //   {
  //     Serial.println("YAp!");
  //     BLE.poll();
  //   }
    
  // }

}

#define THRESHOLD 40.
#define THRESHOLDACCEL 0.8
#define STABILITY_T 300
#define SIGNAL_DELAY 100
//#define THRESHOLD_ACCEL_X 0.9   // Y축 가속도 기준 (단위: g)
#define THRESHOLD_GYRO_Y_LOWER 300    // X축 자이로 기준 (단위: dps)
//#define THRESHOLD_GYRO_Y_UPPER 50    // X축 자이로 기준 (단위: dps)


bool detect_finger_tap() {
            
  float gyroY = myIMU.readFloatGyroY();    // X축 자이로 (단위: dps)

  if (abs(gyroY) > THRESHOLD_GYRO_Y_LOWER) {
    Serial.println("Upward Finger Tap Detected");
    return true;  // Tap 감지
  }

  return false;  // Tap 미감지

}

unsigned long lastTapTime = 0;  // 마지막 탭 발생 시간
const unsigned long debounceDelay = 600;

unsigned long roop_time = 0;
void loop(void)
{ 
  digitalWrite(LED_BUILTIN, HIGH);

  BLEDevice central = BLE.central();


  if (central) {
    if (detect_finger_tap()) {
      unsigned long currentTime = millis();  // 현재 시각
  
      if (currentTime - lastTapTime > debounceDelay) {
        lastTapTime = currentTime;  // 현재 시각으로 업데이트
  
        // 신호 전송
        rxCharacteristic.writeValue("FINGER_TAP_DETECTED");
        Serial.println("Sent signal: FINGER_TAP_DETECTED");
      }
    }
    tone(IR_SEND_PIN, 38000);
    digitalWrite(LED_BUILTIN, LOW);
  
  }
 
}
