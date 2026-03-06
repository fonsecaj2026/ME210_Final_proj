// ============================================================
//  ME210_Controller.ino  –  Controller Arduino (I2C Master)
//  Handles: sensors, FSM, limit switches, ultrasonic, line
//  Sends motor commands over I2C to the Peripheral Arduino.
// ============================================================
#include <Wire.h>
#include <Metro.h>

// TODO
// add commands for strafe left and diag BL
// fine tune motors
// add stepper motor functionality and dc flywheel control
// DC motor drive: EN A & B share one wire; In1 & In4 share high pin; In2 & In3 share low pin, 3 pins total
// Turn off enable when done
// Stepper motor:

// ── I2C ─────────────────────────────────────────────────────
#define PERIPHERAL_ADDR  8   // I2C address of the peripheral
// Motor command bytes sent over I2C
#define CMD_STOP          0
#define CMD_FORWARD       1
#define CMD_BACKWARD      2
#define CMD_STRAFE_RIGHT  3
#define CMD_ROTATE_RIGHT  4
#define CMD_DIAG_BL       5
#define CMD_STRAFE_LEFT   6
#define CMD_VEER_RIGHT    7
#define CMD_SHOOT         8

// ── Ultrasonic pins ──────────────────────────────────────────
#define TRIG_PIN_1  9
#define ECHO_PIN_1  10
#define TRIG_PIN_2  11
#define ECHO_PIN_2  12

// Wall distance thresholds (cm)
#define LEFT_WALL  18
#define BACK_WALL  8

// ── Line / Hog sensors ───────────────────────────────────────
#define LINE_FRONT  A1
#define LINE_BACK   A3
#define HOG_LEFT    A0
#define HOG_RIGHT   A2

// ── Limit switches ───────────────────────────────────────────
#define LIMIT_BACK  2
#define LIMIT_LEFT  3

// ── Constants ────────────────────────────────────────────────
#define LINE_THRESHOLD     40 //400
#define LED_TIME_INTERVAL  1000

// -------- Flywheel motor driver pins --------
#define SHOOT_PWM   5    // PWM speed control shared by both flywheel motors
#define SHOOT_HI    6    // shared direction input
#define SHOOT_LO    7    // shared direction input

// -------- Stepper driver pins (DRV8825) -----
#define DRV_EN      4    // enable pin (LOW = enabled)
#define DRV_STEP    8    // step pulse
#define DRV_DIR     13   // stepper direction

// -------- Shooting tuning -------------------
int flySpeed = 180;        // flywheel speed (0-255)

#define FLY_SPINUP_MS   2000
#define FEED_DIR        HIGH
#define FEED_STEPS      350
#define STEP_PULSE_US   1000

// ── FSM ──────────────────────────────────────────────────────
typedef enum {
  STATE_STOP, STATE_SCAN, STATE_ORIENTATION, STATE_CORNER,
  STATE_FIND_CENTRELINE, STATE_FORWARD, STATE_CORRECT_LINE,
  STATE_BACK, STATE_SHOOT1, STATE_SHOOT2,
  STATE_SHOOT3, STATE_RETURN_HOME
} States_t;

States_t state;
States_t previous;
static Metro metTimer0 = Metro(LED_TIME_INTERVAL);
long hog_delay = 0;
long spin_delay = 0;
int min_dist = 2000;

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
    return false;
  }
}

// ── Motor command wrappers ───────────────────────────────────
void stopAll()       { sendMotorCommand(CMD_STOP);         }
void driveForward()  { sendMotorCommand(CMD_FORWARD);      }
void driveBackward() { sendMotorCommand(CMD_BACKWARD);     }
void strafeRight()   { sendMotorCommand(CMD_STRAFE_RIGHT); }
void strafeLeft()    { sendMotorCommand(CMD_STRAFE_LEFT);  }
void rotateRight()   { sendMotorCommand(CMD_ROTATE_RIGHT); }
void motorBackLeft() { sendMotorCommand(CMD_DIAG_BL);      }
void veerRight()     { sendMotorCommand(CMD_VEER_RIGHT);   }

// ────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // while (!Serial) {}
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
  // activate stepper
  stepperEnable(true);
  delay(10000); // 10 sec
  Serial.println("Controller ready – starting FSM");
  spin_delay = millis();
  state = STATE_SCAN;
}
// ────────────────────────────────────────────────────────────
//  Main loop
// ────────────────────────────────────────────────────────────
void loop() {
  // ir test:
  // while(1) {
  //   int leftVal  = analogRead(HOG_LEFT);
  //     // int rightVal = analogRead(HOG_RIGHT);
  //   Serial.print("Hog Left: "); Serial.println(leftVal);
  //   delay(10);
  // }


  checkGlobalEvents();
  switch (state) {
    case STATE_STOP:             handle_stop();            break;
    case STATE_SCAN:             handle_scan();            break;
    case STATE_ORIENTATION:      handle_orientation();     break;
    case STATE_CORNER:           handle_corner();          break;
    case STATE_FIND_CENTRELINE:  handle_find_centreline(); break;
    case STATE_FORWARD:          handle_forward();         break;
    case STATE_CORRECT_LINE:     handle_correct_line();    break;
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

void handle_scan()           { rotateRight(); } // complete a full spin (time this)
void handle_orientation()    { rotateRight(); } // spin for minimum
void handle_corner()         { strafeLeft(); }  // strafe into left wall
void handle_find_centreline(){ strafeRight(); }
void handle_forward()        { driveForward(); }
void handle_correct_line()   { veerRight(); }
void handle_back()           { driveBackward(); } // timer based, need to implement; use millis()

// Shooters are driven by controller, handling flywheel DC and stepper
void handle_shoot1()         { stopAll(); shootOnce(); while (1) {} } // hang for debugging
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
uint8_t test_for_scan() {
  if (state == STATE_SCAN) {
    int curr_L = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    int curr_B = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    int dist = curr_L + curr_B;

    if (dist < min_dist) {
      min_dist = dist;
    }
    if (spin_delay + 5200 < millis()) {
      return true;
    }
  }
  return false;
}

void resp_to_scan() {
  if (state == STATE_SCAN) {
    Serial.println("minima found");
    previous = state;
    state = STATE_ORIENTATION;
  }
}

uint8_t test_for_orient() {
  if (state == STATE_ORIENTATION) {
    Serial.println("Searching for corner");
    // long left = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    // long back = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    // Serial.print("left dist: ");
    // Serial.println(left);
    // Serial.print("back dist: ");
    // Serial.println(back);
    int curr_L = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    int curr_B = getDistance(TRIG_PIN_2, ECHO_PIN_2);
    int dist = curr_L + curr_B;

    // test tolarance asap
    if (dist <= min_dist) {
      return true;
    }
  }
  return false;
}

// Last minute changes: forget location, once we orient, move straight for the rings
void resp_to_orient() {
  if (state == STATE_ORIENTATION) {
    previous = state;
    state = STATE_FORWARD;
    hog_delay = millis();
    Serial.println("Orientation found – locking position");
  }
}

uint8_t test_for_wall() {
  if (state == STATE_CORNER) {
    Serial.println("Checking for wall");
    if (limitLeftTriggered) return true;
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

// Triggers when back sensor alone detects the centre line.
// The robot has veered left, so the front sensor misses first –
// transition to FORWARD and immediately enter correction if needed.
uint8_t test_for_center() {
  if (state == STATE_FIND_CENTRELINE) {
    int backVal = analogRead(LINE_BACK);
    Serial.print("Back Sensor: "); Serial.println(backVal);
    if (backVal > LINE_THRESHOLD) return true;
  }
  return false;
}
void resp_to_center() {
  if (state == STATE_FIND_CENTRELINE) {
    previous = state;
    state = STATE_FORWARD;
    Serial.println("Back sensor on centre line – moving forward");
  }
}
// Triggers when the front sensor loses the line while driving forward.
// Enters correction mode to veer right until the front sensor recovers.
uint8_t test_for_line_lost() {
  if (state == STATE_FORWARD) {
    int frontVal = analogRead(LINE_FRONT);
    Serial.print("Front Sensor: "); Serial.println(frontVal);
    if (frontVal <= LINE_THRESHOLD) return true;
  }
  return false;
}
void resp_to_line_lost() {
  if (state == STATE_FORWARD) {
    previous = state;
    state = STATE_CORRECT_LINE;
    Serial.println("Front sensor off line – correcting right");
  }
}
// Triggers when the front sensor reacquires the line during correction.
uint8_t test_for_line_recovered() {
  if (state == STATE_CORRECT_LINE) {
    int frontVal = analogRead(LINE_FRONT);
    Serial.print("Front Sensor (correcting): "); Serial.println(frontVal);
    if (frontVal > LINE_THRESHOLD) return true;
  }
  return false;
}
void resp_to_line_recovered() {
  if (state == STATE_CORRECT_LINE) {
    previous = state;
    state = STATE_FORWARD;
    Serial.println("Front sensor back on line – resuming forward");
  }
}
// Hog line check covers both FORWARD and CORRECT_LINE so the robot
// always stops at the hog line regardless of which state it is in.
uint8_t test_for_hog_line() {
  if (hog_delay + 2000 < millis()) {
    if (state == STATE_FORWARD || state == STATE_CORRECT_LINE) {
      int leftVal  = analogRead(HOG_LEFT);
      // int rightVal = analogRead(HOG_RIGHT);
      Serial.print("Hog Left: "); Serial.print(leftVal);
      // Serial.print("\t Hog Right: "); Serial.println(rightVal);
      if (leftVal > LINE_THRESHOLD) return true; //  || rightVal > LINE_THRESHOLD
    }
  }
  return false;
}
void resp_to_hog_lines() {
  if (state == STATE_FORWARD || state == STATE_CORRECT_LINE) {
    Serial.println("Hog line detected – stopping");
    previous = state;
    state = STATE_BACK;
    hog_delay = millis();
  }
}

// could add another, smaller hog delay incase of double trigger
bool test_for_backup() {
  if (state == STATE_BACK && hog_delay + 500 < millis()) {
    Serial.println("backing up");
    return true;
  }
  return false;
}

void resp_to_back() {
  if (state == STATE_BACK) {
    Serial.println("back up for hog line again");
    previous = state;
    state = STATE_SHOOT1;
  }
}

void checkGlobalEvents() {
  if (test_for_scan())           resp_to_scan();
  if (test_for_orient())         resp_to_orient();
  if (test_for_wall())           resp_to_wall();
  if (test_for_center())         resp_to_center();
  if (test_for_hog_line())       resp_to_hog_lines();    // checked before line_lost / recovered
  // if (test_for_line_lost())      resp_to_line_lost();
  // if (test_for_line_recovered()) resp_to_line_recovered();
  if (test_for_backup())         resp_to_back();
}

// Final code for shooting
// ------------------------------------------------------------
// Turn flywheels ON at the current speed
// ------------------------------------------------------------
void flywheelsOn() {
  digitalWrite(SHOOT_HI, HIGH);
  digitalWrite(SHOOT_LO, LOW);
  analogWrite(SHOOT_PWM, flySpeed);
}


// ------------------------------------------------------------
// Turn flywheels OFF
// ------------------------------------------------------------
void flywheelsOff() {
  analogWrite(SHOOT_PWM, 0);
  digitalWrite(SHOOT_HI, LOW);
  digitalWrite(SHOOT_LO, LOW);
}


// ------------------------------------------------------------
// Enable / disable stepper driver
// ------------------------------------------------------------
void stepperEnable(bool en) {
  digitalWrite(DRV_EN, en ? LOW : HIGH);
}


// ------------------------------------------------------------
// Feed the stepper a set number of steps
// ------------------------------------------------------------
void feedSteps(int steps) {

  digitalWrite(DRV_DIR, FEED_DIR);
  stepperEnable(true);

  for (int i = 0; i < steps; i++) {
    digitalWrite(DRV_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(DRV_STEP, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }

  //stepperEnable(false);
}


// ------------------------------------------------------------
// Full shooting sequence
// ------------------------------------------------------------
void shootOnce() {

  Serial.println("Flywheels ON");
  flywheelsOn();

  delay(FLY_SPINUP_MS);

  Serial.println("Feeding stepper");
  feedSteps(FEED_STEPS);

  Serial.println("Flywheels OFF");
  flywheelsOff();
}
