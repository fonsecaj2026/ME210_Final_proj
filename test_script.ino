#include <AFMotor.h>

// DC Motors for Mecanum Wheels
AF_DCMotor FL(1);
AF_DCMotor FR(4);
AF_DCMotor RL(2);
AF_DCMotor RR(3);

int fast_speed  = 200;
int slow_speed  = 90;
;

void stopAll() {
  FL.run(RELEASE);
  FR.run(RELEASE);
  RL.run(RELEASE);
  RR.run(RELEASE);
}

void setAllSpeed(int spd){
  FL.setSpeed(spd);
  FR.setSpeed(spd);
  RL.setSpeed(spd);
  RR.setSpeed(spd);
}

void driveForward(){
  FL.setSpeed(160);
  FR.setSpeed(200);
  RL.setSpeed(160);
  RR.setSpeed(200);

  FL.run(FORWARD);
  FR.run(FORWARD);
  RL.run(FORWARD);
  RR.run(FORWARD);
}

void driveBackward(){
  FL.setSpeed(160);
  FR.setSpeed(200);
  RL.setSpeed(160);
  RR.setSpeed(200);

  FL.run(BACKWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(BACKWARD);
}

void strafeRight(){
  setAllSpeed(220);
  FL.run(FORWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
}

void rotateRight(){
  setAllSpeed(220);


  FL.run(FORWARD);
  FR.run(BACKWARD);
  RL.run(FORWARD);
  RR.run(BACKWARD);
}

void setup() {
  Serial.begin(9600);
  delay(200);

  Serial.println("Serial motor test ready.");
  Serial.println("Type: F, B, R, S (then press Enter).");
  stopAll();
}

void loop() {
  // ONLY do stuff when you type something in Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "F") {
      Serial.println("Forward");
      driveForward();
      delay(3000);
      stopAll();
      return;
    }
    else if (input == "B") {
      Serial.println("Backward");
      driveBackward();
      delay(2000);
      stopAll();
      return;
    }
    else if (input == "R") {
      Serial.println("Rotate Right");
      rotateRight();
      delay(5000);
      stopAll();
      return;
    }
    else if (input == "S") {
      Serial.println("Strafe Right");
      strafeRight();
      delay(3000);
      stopAll();
      return;
    }
    else if (input == "X") {
      Serial.println("Stop");
      stopAll();
      return;
    }
    else {
      Serial.println("Nothing happens with this command");
    }
  }
}