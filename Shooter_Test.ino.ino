// ============================================================
// Shooter_Test.ino
// Tests only the shooting mechanism on the controller Arduino
// ============================================================

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

#define FLY_SPINUP_MS   1200
#define FEED_DIR        HIGH
#define FEED_STEPS      200
#define STEP_PULSE_US   800


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

  stepperEnable(false);
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


// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {

  Serial.begin(9600);
  delay(300);

  // Flywheel motor driver pins
  pinMode(SHOOT_PWM, OUTPUT);
  pinMode(SHOOT_HI, OUTPUT);
  pinMode(SHOOT_LO, OUTPUT);

  // Stepper driver pins
  pinMode(DRV_EN, OUTPUT);
  pinMode(DRV_STEP, OUTPUT);
  pinMode(DRV_DIR, OUTPUT);

  // Safe startup state
  analogWrite(SHOOT_PWM, 0);
  digitalWrite(SHOOT_HI, LOW);
  digitalWrite(SHOOT_LO, LOW);

  digitalWrite(DRV_STEP, LOW);
  digitalWrite(DRV_DIR, LOW);
  digitalWrite(DRV_EN, HIGH);   // stepper disabled

  Serial.println("Shooter test ready (9600 baud)");
  Serial.println("Commands:");
  Serial.println("f = flywheels on");
  Serial.println("x = flywheels off");
  Serial.println("s = stepper feed");
  Serial.println("t = full shot");
  Serial.println("1 = slow speed");
  Serial.println("2 = medium speed");
  Serial.println("3 = fast speed");
}


// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------
void loop() {

  if (Serial.available() > 0) {

    char cmd = Serial.read();

    if (cmd == 'f') {
      Serial.print("Flywheels ON speed ");
      Serial.println(flySpeed);
      flywheelsOn();
    }

    else if (cmd == 'x') {
      Serial.println("Flywheels OFF");
      flywheelsOff();
    }

    else if (cmd == 's') {
      Serial.println("Stepper feed");
      feedSteps(FEED_STEPS);
    }

    else if (cmd == 't') {
      Serial.println("Full shot sequence");
      shootOnce();
    }

    else if (cmd == '1') {
      flySpeed = 120;
      Serial.println("Speed set to 120");
    }

    else if (cmd == '2') {
      flySpeed = 180;
      Serial.println("Speed set to 180");
    }

    else if (cmd == '3') {
      flySpeed = 220;
      Serial.println("Speed set to 220");
    }

  }
}
