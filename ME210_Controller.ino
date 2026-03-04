// ============================================================
//  ME210_Controller.ino  –  Controller Arduino (I2C Master)
//  Handles: sensors, FSM, limit switches, ultrasonic, line
//  Sends motor commands over I2C to the Peripheral Arduino.
// ============================================================

#include <Wire.h>
#include <Metro.h>

// ── I2C ─────────────────────────────────────────────────────
#define PERIPHERAL_ADDR  8   // I2C address of the peripheral

// Motor command bytes sent over I2C
#define CMD_STOP          0
#define CMD_FORWARD       1
#define CMD_BACKWARD      2
#define CMD_STRAFE_RIGHT  3
#define CMD_ROTATE_RIGHT  4

// ── Ultrasonic pins ──────────────────────────────────────────
#define TRIG_PIN_1  9
#define ECHO_PIN_1  10
#define TRIG_PIN_2  11
#define ECHO_PIN_2  12

// Wall distance thresholds (cm)
#define LEFT_WALL  45
#define BACK_WALL  35

// ── Line / Hog sensors ───────────────────────────────────────
#define LINE_FRONT  A1
#define LINE_BACK   A3
#define HOG_LEFT    A2
#define HOG_RIGHT   A0

// ── Limit switches ───────────────────────────────────────────
#define LIMIT_BACK  2
#define LIMIT_LEFT  3

// ── Constants ────────────────────────────────────────────────
#define LINE_THRESHOLD      300
#define LED_TIME_INTERVAL  1000

// ── FSM ──────────────────────────────────────────────────────
typedef enum {
  STATE_STOP, STATE_ORIENTATION, STATE_CORNER,
  STATE_FIND_CENTRELINE, STATE_FORWARD,
  STATE_BACK, STATE_SHOOT1, STATE_SHOOT2,
  STATE_SHOOT3, STATE_RETURN_HOME
} States_t;

States_t state;
States_t previous;

static Metro metTimer0 = Metro(LED_TIME_INTERVAL);

uint8_t onHogLine       = 0;
uint8_t onCenterLine    = 0;
uint8_t orientationFound = 0;
uint8_t locationFound   = 0;

// ── Limit-switch ISR flags ───────────────────────────────────
volatile bool limitBackTriggered = false;
volatile bool limitLeftTriggered = false;

void limitBackISR() { limitBackTriggered = true; }
void limitLeftISR() { limitLeftTriggered = true; }

// ────────────────────────────────────────────────────────────
//  I2C helper – send a command byte and read back "OK"/"FAIL"
// ────────────────────────────────────────────────────────────
bool sendMotorCommand(uint8_t cmd) {
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(cmd);
  uint8_t err = Wire.endTransmission();
  // error from 
  if (err != 0) {
    Serial.print("[I2C] TX error: "); Serial.println(err);
    return false;
  }

  delay(5);  // give peripheral time to process

  // Request 4-byte response ("OK\r\n" or "FAIL")
  Wire.requestFrom((uint8_t)PERIPHERAL_ADDR, (uint8_t)4);
  String resp = "";
  while (Wire.available()) {
    char c = Wire.read();
    if (c != '\r' && c != '\n' && c != '\0') resp += c;
  }

  if (resp == "OK") {
    return true;
  } else {
    Serial.print("[I2C] Peripheral responded: "); Serial.println(resp);
    return false;
  }
}

// ── Motor command wrappers ───────────────────────────────────
void stopAll()       { sendMotorCommand(CMD_STOP);         }
void driveForward()  { sendMotorCommand(CMD_FORWARD);      }
void driveBackward() { sendMotorCommand(CMD_BACKWARD);     }
void strafeRight()   { sendMotorCommand(CMD_STRAFE_RIGHT); }
void rotateRight()   { sendMotorCommand(CMD_ROTATE_RIGHT); }

// ────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();  // join I2C bus as master

  // Limit switches
  pinMode(LIMIT_BACK, INPUT_PULLUP);
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_BACK), limitBackISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_LEFT), limitLeftISR, FALLING);

  // Ultrasonic
  pinMode(TRIG_PIN_1, OUTPUT);  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);  pinMode(ECHO_PIN_2, INPUT);

  // Line / Hog sensors
  pinMode(LINE_FRONT, INPUT);
  pinMode(LINE_BACK,  INPUT);
  pinMode(HOG_LEFT,   INPUT);
  pinMode(HOG_RIGHT,  INPUT);

  stopAll();
  Serial.println("Controller ready – starting FSM");
  state = STATE_ORIENTATION;
}

// ────────────────────────────────────────────────────────────
//  Main loop
// ────────────────────────────────────────────────────────────
void loop() {
  checkGlobalEvents();

  switch (state) {
    case STATE_STOP:             handle_stop();            break;
    case STATE_ORIENTATION:      handle_orientation();     break;
    case STATE_CORNER:           handle_corner();          break;
    case STATE_FIND_CENTRELINE:  handle_find_centreline(); break;
    case STATE_FORWARD:          handle_forward();         break;
    case STATE_BACK:             handle_back();            break;
    case STATE_SHOOT1:           handle_shoot1();          break;
    case STATE_SHOOT2:           handle_shoot2();          break;
    case STATE_SHOOT3:           handle_shoot3();          break;
    case STATE_RETURN_HOME:      handle_return_home();     break;
    default:
      Serial.println("Unknown state; halting");
      stopAll();
      break;
  }
}

// ────────────────────────────────────────────────────────────
//  State handlers
// ────────────────────────────────────────────────────────────
void handle_stop() {
  stopAll();
  if (previous == STATE_FORWARD) {
    Serial.println("Sensors done!");
  }
}

void handle_orientation()    { rotateRight(); } // spin for corner
void handle_corner()         { driveBackward(); } // backup into wall; TODO make diagonal movement if possible (back and left)
void handle_find_centreline(){ strafeRight(); }
void handle_forward()        { driveForward(); }
void handle_back()           { driveBackward(); }

// Shooters are driven by controller, handling flywheel DC and stepper
void handle_shoot1()         { stopAll(); while (1) {} } // hang for debugging
void handle_shoot2()         { stopAll(); }
void handle_shoot3()         { stopAll(); }
void handle_return_home()    { driveBackward(); } // again, might need backwards and left

// ────────────────────────────────────────────────────────────
//  Ultrasonic helper
// ────────────────────────────────────────────────────────────
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.0343 / 2;
}

// ────────────────────────────────────────────────────────────
//  Event tests & responses
// ────────────────────────────────────────────────────────────
uint8_t test_for_orient() {
  if (state == STATE_ORIENTATION) {
    Serial.println("Searching for corner");
    if (getDistance(TRIG_PIN_1, ECHO_PIN_1) < LEFT_WALL &&
        getDistance(TRIG_PIN_2, ECHO_PIN_2) < BACK_WALL) {
      return true;
    }
  }
  return false;
}

void resp_to_orient() {
  if (state == STATE_ORIENTATION) {
    previous = state;
    state = STATE_CORNER;
    Serial.println("Orientation found – locking position");
  }
}

uint8_t test_for_wall() {
  if (state == STATE_CORNER) {
    Serial.println("Checking for wall");
    if (limitLeftTriggered && limitBackTriggered) return true;
  }
  return false;
}

void resp_to_wall() {
  if (state == STATE_CORNER) {
    previous = state;
    state = STATE_FIND_CENTRELINE;
    Serial.println("In corner – moving to centre line");
  }
}

uint8_t test_for_center() {
  if (state == STATE_FIND_CENTRELINE) {
    int frontVal = digitalRead(LINE_FRONT);
    int backVal  = analogRead(LINE_BACK);
    Serial.print("Front Sensor: "); Serial.print(frontVal);
    Serial.print("\t Back Sensor: "); Serial.println(backVal);
    if (frontVal && backVal < LINE_THRESHOLD) return true;
  }
  return false;
}

void resp_to_center() {
  if (state == STATE_FIND_CENTRELINE) {
    previous = state;
    state = STATE_FORWARD;
    Serial.println("On centre line");
  }
}

uint8_t test_for_hog_line() {
  if (state == STATE_FORWARD) {
    int leftVal  = analogRead(HOG_LEFT);
    int rightVal = analogRead(HOG_RIGHT);
    Serial.print("Hog Left: "); Serial.print(leftVal);
    Serial.print("\t Hog Right: "); Serial.println(rightVal);
    if (leftVal < LINE_THRESHOLD && rightVal < LINE_THRESHOLD) return true;
  }
  return false;
}

void resp_to_hog_lines() {
  if (state == STATE_FORWARD) {
    Serial.println("Hog line detected – stopping");
    previous = state;
    state = STATE_STOP;
  }
}

void checkGlobalEvents() {
  if (test_for_orient())   resp_to_orient();
  if (test_for_wall())     resp_to_wall();
  if (test_for_center())   resp_to_center();
  if (test_for_hog_line()) resp_to_hog_lines();
}
