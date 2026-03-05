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

// ── Ultrasonic pins ──────────────────────────────────────────
#define TRIG_PIN_1  9
#define ECHO_PIN_1  10
#define TRIG_PIN_2  11
#define ECHO_PIN_2  12

// Shooter / feeder pins
#define DRV_EN      4    // enables/disables the DRV8825 stepper driver
#define SHOOT_PWM   5    // PWM speed control shared by both flywheel motors
#define SHOOT_HI    6    // shared flywheel motor direction input
#define SHOOT_LO    7    // shared flywheel motor direction input
#define DRV_STEP    8    // step pulse output to the DRV8825
#define DRV_DIR     13   // feeder stepper direction output

// Shooting settings
#define FLY_SPINUP_MS   1200   // wait time before feeding so flywheels reach speed
#define FLY_SPEED       200    // flywheel speed from 0 to 255
#define FEED_DIR        HIGH   // feeder rotation direction
#define FEED_STEPS      200    // number of step pulses for one feed
#define STEP_PULSE_US   800    // time between stepper pulse edges

// Wall distance thresholds (cm)
#define LEFT_WALL  25
#define BACK_WALL  15

// ── Line / Hog sensors ───────────────────────────────────────
#define LINE_FRONT  A2
#define LINE_BACK   A0
#define HOG_LEFT    A1
#define HOG_RIGHT   A3

// ── Limit switches ───────────────────────────────────────────
#define LIMIT_BACK  2
#define LIMIT_LEFT  3

// ── Constants ────────────────────────────────────────────────
#define LINE_THRESHOLD      550
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
    // Serial.print("[I2C] Peripheral responded: "); Serial.println(resp);
    return false;
  }
}

// ── Motor command wrappers ───────────────────────────────
void stopAll()       { sendMotorCommand(CMD_STOP);         }
void driveForward()  { sendMotorCommand(CMD_FORWARD);      }
void driveBackward() { sendMotorCommand(CMD_BACKWARD);     }
void strafeRight()   { sendMotorCommand(CMD_STRAFE_RIGHT); }
void strafeLeft()    { sendMotorCommand(CMD_STRAFE_LEFT);  }
void rotateRight()   { sendMotorCommand(CMD_ROTATE_RIGHT); }
void motorBackLeft() { sendMotorCommand(CMD_DIAG_BL);      }
void veerRight()     { sendMotorCommand(CMD_VEER_RIGHT);   }

// ── Throw helpers ──────────────────────────
void flywheelsOn() {
  digitalWrite(SHOOT_HI, HIGH);   // sets flywheel motors to the shooting direction
  digitalWrite(SHOOT_LO, LOW);
  analogWrite(SHOOT_PWM, FLY_SPEED); // drives both flywheel motors at the chosen speed
}

void flywheelsOff() {
  analogWrite(SHOOT_PWM, 0);      // removes power from both flywheel motors
  digitalWrite(SHOOT_HI, LOW);
  digitalWrite(SHOOT_LO, LOW);
}

void stepperEnable(bool en) {
  digitalWrite(DRV_EN, en ? LOW : HIGH);  // LOW enables the DRV8825, HIGH disables it
}

void feedSteps(int steps) {
  digitalWrite(DRV_DIR, FEED_DIR);  // sets which way the feeder stepper rotates
  stepperEnable(true);

  for (int i = 0; i < steps; i++) {
    digitalWrite(DRV_STEP, HIGH);   // one HIGH/LOW pulse = one step pulse to the driver
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(DRV_STEP, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }

  stepperEnable(false);
}
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

  pinMode(SHOOT_PWM, OUTPUT);   // shooter PWM pin for flywheel speed
  pinMode(SHOOT_HI, OUTPUT);    // shooter direction pin
  pinMode(SHOOT_LO, OUTPUT);    // shooter direction pin

  analogWrite(SHOOT_PWM, 0);
  digitalWrite(SHOOT_HI, LOW);
  digitalWrite(SHOOT_LO, LOW);

  pinMode(DRV_STEP, OUTPUT);
  pinMode(DRV_DIR, OUTPUT);
  pinMode(DRV_EN, OUTPUT);
  digitalWrite(DRV_STEP, LOW);
  digitalWrite(DRV_DIR, LOW);
  digitalWrite(DRV_EN, HIGH);   // keeps the stepper driver off until feeding

  flywheelsOff();               
  stepperEnable(false);  
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
      flywheelsOff();               
      stepperEnable(false);      
      break;
  }
}

// ────────────────────────────────────────────────────────────
//  State handlers
// ────────────────────────────────────────────────────────────
void handle_stop() {
  stopAll();
  flywheelsOff();      // makes sure the shooter motors are off in STOP
  stepperEnable(false); // makes sure the feeder driver is disabled in STOP
  if (previous == STATE_FORWARD) {
    Serial.println("Sensors done!");
  }
}

void handle_orientation()    { rotateRight(); } // spin for corner
void handle_corner()         { strafeLeft(); } // strafe into left wall; 
void handle_find_centreline(){ strafeRight(); }
void handle_forward()        { driveForward(); }
void handle_back()           { driveBackward(); } // timer based, need to implement; use millis()

// Shooters are driven by controller, handling flywheel DC and stepper
void handle_shoot1(){ 
  static bool started = false;
  static unsigned long t0 = 0;

   if (!started) {
    started = true;
    t0 = millis();

    stopAll();                 // stops the drivetrain before shooting
    Serial.println("SHOOT1: flywheels ON (spinup)");
    flywheelsOn();
  }

  if (millis() - t0 >= FLY_SPINUP_MS) {  // waits for the flywheels to get up to speed
    Serial.println("SHOOT1: feeding stepper");
    feedSteps(FEED_STEPS);               // rotates the feeder stepper to push one shot in

    Serial.println("SHOOT1: flywheels OFF");
    flywheelsOff();

    started = false;
    previous = state;
    state = STATE_STOP;        
  }
}
  
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

uint8_t test_for_center() {
  if (state == STATE_FIND_CENTRELINE) {
    int frontVal = analogRead(LINE_FRONT);
    int backVal  = analogRead(LINE_BACK);
    Serial.print("Front Sensor: "); Serial.print(frontVal);
    Serial.print("\t Back Sensor: "); Serial.println(backVal);
    if (frontVal > LINE_THRESHOLD && backVal > LINE_THRESHOLD) return true;
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
    if (leftVal > LINE_THRESHOLD && rightVal > LINE_THRESHOLD) return true;
  }
  return false;
}

void resp_to_hog_lines() {
  if (state == STATE_FORWARD) {
    Serial.println("Hog line detected – starting shoot1"); // sends the FSM into the shooting state
    previous = state;
    state = STATE_SHOOT1;
  }
}

void checkGlobalEvents() {
  if (test_for_orient())   resp_to_orient();
  if (test_for_wall())     resp_to_wall();
  if (test_for_center())   resp_to_center();
  if (test_for_hog_line()) resp_to_hog_lines();
}
