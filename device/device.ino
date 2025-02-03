#include <IRremote.hpp>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, OUTPUT);

  // IR receiver
  IrReceiver.begin(2, DISABLE_LED_FEEDBACK);

  // LEDs
  pinMode(3, OUTPUT); // GREEN LED
  pinMode(4, OUTPUT); // RED LED
  pinMode(5, OUTPUT); // YELLOW LED

  // switch
  pinMode(6, INPUT_PULLUP);

  // ID input
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // serial timeout stuff (max waiting time)
#define WAITING_TIMEOUT 1000
  Serial.setTimeout(WAITING_TIMEOUT);

  matrix_off();

}

uint8_t sAddress = 0x02;
uint8_t sCommand = 0x10;
uint8_t sRepeats = 1;

enum Detection {
  DET_NONE,
  DET_UNKNOWN,
  DET_DETECTED,
};

Detection attemptDetect();
Detection attemptDetect() {

  /*
  * Check if received data is available and if yes, try to decode it.
  * Decoded result is in the IrReceiver.decodedIRData structure.
  *
  * E.g. command is in IrReceiver.decodedIRData.command
  * address is in command is in IrReceiver.decodedIRData.address
  * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
  */
  if (IrReceiver.decode()) {

      /*
      * Print a summary of received data
      
      */
      // Serial.print("Protocol: ");
      // Serial.println(IrReceiver.decodedIRData.protocol);
      // Serial.print("Command: ");
      // Serial.println(IrReceiver.decodedIRData.command, HEX);
      // Serial.print("Address: ");
      // Serial.println(IrReceiver.decodedIRData.address, HEX);

      if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
          // Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
          // We have an unknown protocol here, print extended info
          // IrReceiver.printIRResultRawFormatted(&Serial, true);
          IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
          return DET_UNKNOWN;
      } else {
          IrReceiver.resume(); // Early enable receiving of the next IR frame
          // IrReceiver.printIRResultShort(&Serial);
          // IrReceiver.printIRSendUsage(&Serial);
      }
      // Serial.println();

      /*
      * Finally, check the received data and perform actions according to the received command
      */
      if (IrReceiver.decodedIRData.command == 0x10) {
        // digitalWrite(6, HIGH);
        Serial.println("receive");
        return DET_DETECTED;
      } else {
        return DET_UNKNOWN;
      }

  }

  return DET_NONE;
}


#define RED 3
#define YELLOW 4
#define GREEN 5
#define LED_NONE -1

void cond_on(const uint8_t target, const uint8_t checked) {
  if (target == checked)
    digitalWrite(checked, LOW);
  else
    digitalWrite(checked, HIGH);
}

void select_led(const uint8_t pin) {
  cond_on(pin, RED);
  cond_on(pin, YELLOW);
  cond_on(pin, GREEN);
}

void based_led(const uint8_t pin, bool cond) {
  if (cond)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}


uint8_t read_id() {
  uint8_t cum = 0;
  #define BIT_READ(pin, pwr) (cum |= !digitalRead(pin) << pwr)
  BIT_READ(12, 0); 
  BIT_READ(11, 1); 
  BIT_READ(10, 2); 
  BIT_READ(9, 3); 
  BIT_READ(8, 4); 
  BIT_READ(7, 5); 

  return cum;

  // 7 8 9 10 11 12
}

bool is_on() {
  return digitalRead(6);
}

void cond_aon(const uint8_t target, const uint8_t checked) {
  if (target == checked)
    analogWrite(checked, 0);
  else
    analogWrite(checked, 255);
}

uint8_t step2pin(int step) {
  switch (step) {
    case 0: return A0;
    case 1: return A1;
    case 2: return A2;
    case 3: return A5;
    case 4: return A4;
    default: return A3;
  } 
}


#define NUM_LEDS 6
#define CYCLE_T 800
#define BLINK_N 0  // i turned this off for now
#define BLINK_PERIOD 500
#define STEP_DELAY (CYCLE_T/NUM_LEDS)

void matrix_spinner(bool state) {
  long now = millis() % CYCLE_T;
  int step = now / STEP_DELAY;
  int target = step2pin(step);

  // signal state between spins
  bool should_signal_state = ((millis() + 1234) % BLINK_PERIOD) < BLINK_N;
  if (should_signal_state) {
    if (state) matrix_off(); else matrix_on();
    return;
  }

  cond_aon(target, A0);
  cond_aon(target, A1);
  cond_aon(target, A2);
  cond_aon(target, A3);
  cond_aon(target, A4);
  cond_aon(target, A5);
}


void matrix_off() {
  analogWrite(A0, 255);
  analogWrite(A1, 255);
  analogWrite(A2, 255);
  analogWrite(A3, 255);
  analogWrite(A4, 255);
  analogWrite(A5, 255);
}

void matrix_on() {
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  analogWrite(A4, 0);
  analogWrite(A5, 0);
}

void matrix_blinker(bool state) {
  long which = millis() % BLINK_PERIOD;
  if (which < BLINK_N) {
    if (state) matrix_off(); else matrix_on();
  } else {
    if (state) matrix_on(); else matrix_off();
  }
}

void cycle_light() {
  #define LED_BLINK_PERIOD 333
  int which = (millis() % (3 * LED_BLINK_PERIOD)) / LED_BLINK_PERIOD;

  switch (which) {
    case 0: select_led(RED); break;
    case 1: select_led(YELLOW); break;
    case 2: select_led(GREEN); break;
  }
}


enum DeviceState {
  STATE_STABLE,
  STATE_DETECTED,
  STATE_CONFIRM,
  STATE_UNKNOWN
};

enum RingState {
  RING_STABLE,
  RING_CONFIRMED
};

#define TIME_WINDOW 7000
#define WAITING_TIME 300

DeviceState currentState = STATE_STABLE;
RingState currentRing = RING_STABLE;
unsigned long lastDetectionTime = 0;
unsigned long lastDetectedSentTime = 0;
unsigned long prev_time = 0;
unsigned long selc_time = 0;
unsigned long det_time = 0;

void loop() {
  // Main code loop
  if (is_on()) {
    Detection detection = attemptDetect(); // IR 신호 감지 시도
    unsigned long now = millis(); // 현재 시간 확인

    // Handle detection states
    if (detection == DET_DETECTED && (now - lastDetectedSentTime > WAITING_TIME)) {
      if (currentState == STATE_STABLE) {
        lastDetectionTime = now; // 마지막 감지 시간 기록
        Serial.write("DETECTED\n"); // 서버로 DETECTED 전송
        lastDetectedSentTime = now; // 마지막 전송 시간 기록
        Serial.print("Detect : ");
        Serial.println(millis());
      }
    }

    // 서버에서 오는 명령 처리
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n'); // 한 줄 읽기
      command.trim(); // 공백 제거

      if (command == "SELECTED") {
        Serial.println("Command received: SELECTED");
        currentState = STATE_DETECTED; // 상태를 DETECTED로 변경
        lastDetectionTime = now; // 상태 변경 시간 기록
      } else if (command == "CONFIRMED" && currentState == STATE_DETECTED) {
        Serial.println("Command received: CONFIRMED");
        currentState = STATE_CONFIRM; // 상태를 CONFIRM으로 변경
      }
    }


    // Check TIME_WINDOW expiration for STATE_DETECTED
    if (currentState == STATE_DETECTED && (now - lastDetectionTime >= TIME_WINDOW)) {
      currentState = STATE_STABLE; // TIME_WINDOW 경과 시 STABLE로 상태 복원
      currentRing = RING_STABLE;  // 링 상태도 초기화
    }

    // LED 상태 처리
    switch (currentState) {
      case STATE_DETECTED:
        select_led(YELLOW); // 감지 상태일 때 YELLOW LED
        break;

      case STATE_CONFIRM:
        select_led(GREEN); // 확인 상태일 때 GREEN LED
        break;

      case STATE_STABLE:
        select_led(RED); // 안정 상태일 때 RED LED
        break;

      case STATE_UNKNOWN:
        select_led(RED); // UNKNOWN 상태일 때도 RED LED
        break;
    }
  } else {
    // 디바이스가 꺼져 있을 때 모든 LED 끄기
    digitalWrite(3, HIGH); // RED LED 끄기
    digitalWrite(4, HIGH); // YELLOW LED 끄기
    digitalWrite(5, HIGH); // GREEN LED 끄기
  }
}
