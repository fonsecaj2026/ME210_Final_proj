// ============================================================
//  ME210_Peripheral.ino  –  Peripheral Arduino (I2C Slave)
//  Handles: DC motor driver (AFMotor), I2C command reception
//  Receives a 1-byte command, executes it, replies "OK"/"FAIL"
// ============================================================

#include <Wire.h>
#include "AFMotor_R4.h"

// ── I2C address ──────────────────────────────────────────────
#define PERIPHERAL_ADDR  8   // must match controller

// ── Motor command bytes (must match controller) ──────────────
#define CMD_STOP          0
#define CMD_FORWARD       1
#define CMD_BACKWARD      2
#define CMD_STRAFE_RIGHT  3
#define CMD_ROTATE_RIGHT  4

// ── Motor objects ────────────────────────────────────────────
AF_DCMotor FL(1);
AF_DCMotor FR(4);
AF_DCMotor RL(2);
AF_DCMotor RR(3);

int fast_speed = 180;
int med_speed  = 130;
int slow_speed =  90;

// ── Response buffer ──────────────────────────────────────────
// onRequest() is called from ISR context – keep it simple.
// We stage the response string here after onReceive() runs.
volatile char responseBuffer[5] = "OK";   // "OK" or "FAIL"

// ────────────────────────────────────────────────────────────
//  Low-level motor functions
// ────────────────────────────────────────────────────────────
void motorStopAll() {
  FL.run(RELEASE);
  FR.run(RELEASE);
  RL.run(RELEASE);
  RR.run(RELEASE);
}

void motorSetAllSpeed(int spd) {
  FL.setSpeed(spd);
  FR.setSpeed(spd);
  RL.setSpeed(spd);
  RR.setSpeed(spd);
}

void motorDriveForward() {
  FL.setSpeed(med_speed);  FR.setSpeed(fast_speed);
  RL.setSpeed(med_speed);  RR.setSpeed(fast_speed);
  FL.run(FORWARD);  FR.run(FORWARD);
  RL.run(FORWARD);  RR.run(FORWARD);
}

void motorDriveBackward() {
  motorSetAllSpeed(fast_speed);
  FL.run(BACKWARD);  FR.run(BACKWARD);
  RL.run(BACKWARD);  RR.run(BACKWARD);
}

void motorStrafeRight() {
  motorSetAllSpeed(fast_speed);
  FL.run(FORWARD);   FR.run(BACKWARD);
  RL.run(BACKWARD);  RR.run(FORWARD);
}

void motorRotateRight() {
  motorSetAllSpeed(fast_speed);
  FL.run(FORWARD);   FR.run(BACKWARD);
  RL.run(FORWARD);   RR.run(BACKWARD);
}

// ────────────────────────────────────────────────────────────
//  I2C callbacks
// ────────────────────────────────────────────────────────────

// Called when the controller sends a command byte.
void onReceive(int numBytes) {
  if (numBytes < 1) {
    strncpy((char*)responseBuffer, "FAIL", 5);
    return;
  }

  uint8_t cmd = Wire.read();
  // Flush any extra bytes
  while (Wire.available()) Wire.read();

  bool ok = true;

  switch (cmd) {
    case CMD_STOP:
      motorStopAll();
      break;
    case CMD_FORWARD:
      motorDriveForward();
      break;
    case CMD_BACKWARD:
      motorDriveBackward();
      break;
    case CMD_STRAFE_RIGHT:
      motorStrafeRight();
      break;
    case CMD_ROTATE_RIGHT:
      motorRotateRight();
      break;
    default:
      // Unknown command
      motorStopAll();   // fail-safe: stop motors
      ok = false;
      break;
  }

  if (ok) {
    strncpy((char*)responseBuffer, "OK  ", 5);   // padded to 4 bytes
  } else {
    strncpy((char*)responseBuffer, "FAIL", 5);
    Serial.print("[Peripheral] Unknown cmd: "); Serial.println(cmd);
  }
}

// Called when the controller requests a response.
void onRequest() {
  // Send exactly 4 bytes so the master's requestFrom(addr, 4) is satisfied.
  Wire.write((const uint8_t*)responseBuffer, 4);
}

// ────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // maybe don't have serial
  while (!Serial) {}

  motorStopAll();

  Wire.begin(PERIPHERAL_ADDR);  // join I2C bus as slave
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.println("Peripheral ready – waiting for commands");
}

// ────────────────────────────────────────────────────────────
//  Main loop  (all work is done in I2C callbacks)
// ────────────────────────────────────────────────────────────
void loop() {
  // Nothing to do here – motor commands arrive via I2C interrupts.
  // You could add local fault-detection or a watchdog here if needed.
  delay(10);
}
