const int IR_RECEIVE_PIN = 2;
unsigned long detectionStartTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A0, OUTPUT);

  // IR detector
  pinMode(IR_RECEIVE_PIN, INPUT);

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

}

uint8_t sAddress = 0x02;
uint8_t sCommand = 0x10;
uint8_t sRepeats = 1;

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

#define TIME_WINDOW 1000
#define BREAKOFF 200 // tolerance: max amount of time spent since last detected signal.

DeviceState currentState = STATE_STABLE;
RingState currentRing = RING_STABLE;
static long last_signal_time = 0;
bool was_on = false;
bool signalDetected = false;

DeviceState device_state = STATE_STABLE;

long last_detection = 0;

void loop() {
  // Main code loop
  if (is_on()) {
    if (currentState == STATE_STABLE) {
      int irState = digitalRead(IR_RECEIVE_PIN);

      if (irState == LOW) {
        if (!signalDetected) {
          detectionStartTime = millis();
          signalDetected = true;
        }

        if (millis() - detectionStartTime >= BREAKOFF) {
          signalDetected = false;
          Serial.println("DETECTED\n");
        }
      } else {
        signalDetected = false;
      }

      if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command = "SELECTED") {
          Serial.println("Command received: SELECTED");
          currentState = STATE_DETECTED;
          last_signal_time = millis();
        }
      }
    }
    else if (currentState == STATE_DETECTED) {
      if (millis() - last_signal_time >= TIME_WINDOW) {
        currentState = STATE_STABLE;
      }
      else {
        if (Serial.available() > 0) {
          String command = Serial.readStringUntil('\n');
          command.trim();
  
          if (command = "CONFIRMED") {
            Serial.println("Command received: CONFIRMED");
            currentState = STATE_STABLE;
            if (was_on) {
              matrix_off();
              was_on = !was_on;
            }
            else{
              matrix_on();
              was_on = !was_on;
            }
          }
        } 
      }
    }

    switch (currentState) {
      case STATE_STABLE:
        select_led(RED);
        break;

      case STATE_DETECTED :
        select_led(YELLOW);
        break;
    }

  } else {
     // 디바이스가 꺼져 있을 때 모든 LED 끄기
     digitalWrite(3, HIGH); // RED LED 끄기
     digitalWrite(4, HIGH); // YELLOW LED 끄기
     digitalWrite(5, HIGH); // GREEN LED 끄기
     matrix_off();
   }
}
