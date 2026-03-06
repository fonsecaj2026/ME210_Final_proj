#include <AFMotor.h>

// DC Motors for Mecanum Wheels
AF_DCMotor FL(2);
AF_DCMotor FR(1);
AF_DCMotor RL(3);
AF_DCMotor RR(4);

int fast_speed  = 220;
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
  FR.setSpeed(220);
  RL.setSpeed(200);
  RR.setSpeed(220);

  FL.run(FORWARD);
  FR.run(FORWARD);
  RL.run(FORWARD);
  RR.run(FORWARD);
}

void driveBackward(){
  FL.setSpeed(160);
  FR.setSpeed(220);
  RL.setSpeed(200);
  RR.setSpeed(220);

  FL.run(BACKWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(BACKWARD);
}

void strafeRight(){
  // setAllSpeed(255);
  FL.setSpeed(255);
  FR.setSpeed(255);
  RL.setSpeed(245);
  RR.setSpeed(245);

  FL.run(FORWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
}

// void strafeRight(){
//   setAllSpeed(255);
//   FL.run(FORWARD);
//   FR.run(BACKWARD);
//   RL.run(BACKWARD);
//   RR.run(FORWARD);
// }

void rotateRight(){
  FL.setSpeed(210);
  FR.setSpeed(210);
  RL.setSpeed(255);
  RR.setSpeed(255);


  FL.run(BACKWARD);
  FR.run(FORWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
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
      delay(5000);
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
