#include "AFMotor_R4.h"
#include <Metro.h>     

// DC Motors for Mecanuum Wheels 
AF_DCMotor FL(1);
AF_DCMotor FR(4);
AF_DCMotor RL(2);
AF_DCMotor RR(3);

int fast_speed  = 180;  
int med_speed = 130;
int slow_speed = 90;    

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
  FL.setSpeed(med_speed);
  FR.setSpeed(fast_speed);
  RL.setSpeed(med_speed);
  RR.setSpeed(fast_speed);

  FL.run(FORWARD);
  FR.run(FORWARD);
  RL.run(FORWARD);
  RR.run(FORWARD);
}

void driveBackward(){
  setAllSpeed(fast_speed);
  FL.run(BACKWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(BACKWARD);
}

void strafeRight(){
  setAllSpeed(fast_speed);
  FL.run(FORWARD);
  FR.run(BACKWARD);
  RL.run(BACKWARD);
  RR.run(FORWARD);
}

void rotateRight(){
  FL.setSpeed(fast_speed);
  FR.setSpeed(fast_speed);
  RL.setSpeed(fast_speed);
  RR.setSpeed(fast_speed);

  FL.run(FORWARD);
  FR.run(BACKWARD);
  RL.run(FORWARD);
  RR.run(BACKWARD);
}

// Ultrasonic
// Define pins for the left ultrasonic sensor (Sensor 1)
#define TRIG_PIN_1 9
#define ECHO_PIN_1 10

// Define pins for the back ultrasonic sensor (Sensor 2)
#define TRIG_PIN_2 11
#define ECHO_PIN_2 12

// Define wall distance thresholds
#define LEFT_WALL 45 // cm
#define BACK_WALL 35 // cm

// Center Line Follow
#define LINE_FRONT A1
#define LINE_BACK A3

// Hog line detection
#define HOG_LEFT A2
#define HOG_RIGHT A0

// Limit Switches
#define LIMIT_BACK 2
#define LIMIT_LEFT 3

// state definition 
typedef enum {
  STATE_STOP, STATE_ORIENTATION, STATE_CORNER, STATE_FIND_CENTRELINE, STATE_FORWARD,
  STATE_BACK, STATE_SHOOT1, STATE_SHOOT2, STATE_SHOOT3, STATE_RETURN_HOME
} States_t;

//definitions
#define LINE_THRESHOLD          300 
#define MOTOR_TIME_INTERVAL     3000
#define TURNING_TIME            2000
#define LED_TIME_INTERVAL       1000
#define TIMER_0            0
#define TIMER_1            0 

//variables 
States_t state;
States_t previous;
static Metro metTimer0 = Metro(LED_TIME_INTERVAL);
uint8_t onHogLine;
uint8_t onCenterLine;
uint8_t orientationFound;
uint8_t locationFound; 

//declaring functions 
void checkGlobalEvents();
void handle_stop();
void handle_orientation();
uint8_t test_for_orient(void);
void resp_to_orient(void);
uint8_t test_for_wall(void);
void resp_to_wall(void);
uint8_t test_for_center(void);
void resp_to_center(void);
uint8_t test_for_hog_line(void);
void resp_to_hog_lines(void);

// State variables
volatile bool limitBackTriggered = false;
volatile bool limitLeftTriggered = false;

// Interrupt Service Routines (ISR) for limit switches
void limitBackISR() {
  limitBackTriggered = true;  // Set the flag when the back limit switch is triggered
}

void limitLeftISR() {
  limitLeftTriggered = true;  // Set the flag when the left limit switch is triggered
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  // Serial.println("Hello, world!");

  // Set up limit switch pins with internal pull-down resistors
  pinMode(LIMIT_BACK, INPUT_PULLUP);  // Use INPUT_PULLDOWN for NO switches
  pinMode(LIMIT_LEFT, INPUT_PULLUP);

   // Attach interrupts for limit switches
  attachInterrupt(digitalPinToInterrupt(LIMIT_BACK), limitBackISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_LEFT), limitLeftISR, FALLING);

  // Set up pins
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(LINE_FRONT, INPUT);
  pinMode(LINE_BACK, INPUT);
  pinMode(HOG_LEFT, INPUT);
  pinMode(HOG_RIGHT, INPUT);

  onHogLine = 0;
  onCenterLine = 0;
  orientationFound = 0;
  locationFound = 0;

  stopAll();
  Serial.println("about to start");
  state = STATE_ORIENTATION;
}

void loop() {
  checkGlobalEvents();

  //manual debug control

  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n'); 
  //   input.trim();

  //   if (input == "F") {
  //     Serial.println("Going to beep!");
  //     driveForward(); 

  //     delay(3000);    
  //     FR.run(RELEASE); 
  //     FL.run(RELEASE);
  //     RL.run(RELEASE);
  //     RR.run(RELEASE); 
  //     return;          
  //   }
  //     //driveForward();
  //   else if (input == "B"){
  //     driveBackward();
  //     delay(2000);    
  //     FR.run(RELEASE); 
  //     FL.run(RELEASE);
  //     RL.run(RELEASE);
  //     RR.run(RELEASE); 
  //     return;
  //   }
  //   else if (input == "R"){
  //     rotateRight();
  //     delay(3000);    
  //     FR.run(RELEASE); 
  //     FL.run(RELEASE);
  //     RL.run(RELEASE);
  //     RR.run(RELEASE); 
  //     return;
  //   }
  //    else if (input == "S"){
  //     strafeLeft();
  //     delay(1000);    
  //     FR.run(RELEASE); 
  //     FL.run(RELEASE);
  //     RL.run(RELEASE);
  //     RR.run(RELEASE); 
  //     return;
  //   }
  //   else {
  //     Serial.println("Nothing happens with this command");
  //   }
  // }

  switch (state) {
    case STATE_STOP:            
      handle_stop(); 
    break;
    case STATE_ORIENTATION:   
      handle_orientation(); // spin until corner found
    break;
    case STATE_CORNER:          
      handle_corner(); // Drive into corner
    break;
    case STATE_FIND_CENTRELINE:
     handle_find_centreline(); // strafe over to line
    break;
    case STATE_FORWARD:  
      handle_forward(); // follow up to hog line
    break;
    case STATE_BACK:           
      handle_back(); // back up to shoot
    break;
    case STATE_SHOOT1:         
     handle_shoot1();
    break;
    case STATE_SHOOT2:          
      handle_shoot2(); 
    break;
    case STATE_SHOOT3:          
      handle_shoot3(); 
    break;
    case STATE_RETURN_HOME:    
       handle_return_home(); 
    break;
    default:
      Serial.println("What is this I do not even...");
    break;
  }
}

void handle_stop(){
  stopAll();
  if (previous == STATE_FORWARD) {
    Serial.println("Sensors done!");
  }
}

void handle_orientation(){
  rotateRight();
}

void handle_corner(){
  driveBackward();
}

void handle_find_centreline(){
  strafeRight();
}

void handle_forward(){
  driveForward();
}

void handle_back(){
  driveBackward();
}

void handle_shoot1(){ stopAll(); while(1){} }
void handle_shoot2(){ stopAll(); }
void handle_shoot3(){ stopAll(); }

void handle_return_home(){
  driveBackward();
}

// Function to calculate the distance from an ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
  // Send a pulse to the trigger pin to start measurement
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW);  

  // Read the duration of the pulse from the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (speed of sound = 343 m/s or 0.0343 cm/µs)
  long distance = duration * 0.0343 / 2;  

  return distance;
}


// This will pass once orientation is found
uint8_t test_for_orient(void) {
  if (state == STATE_ORIENTATION) {
    Serial.println("Searching for corner");

    // Debugging:
    // long left = getDistance(TRIG_PIN_1, ECHO_PIN_1);
    // long back = getDistance(TRIG_PIN_2, ECHO_PIN_2);

    // Serial.print("Left US: ");
    // Serial.print(left);
    // Serial.print("\t Back US: ");
    // Serial.println(back);

    // return false;

    // Sensors must be under dist threshold
    if (getDistance(TRIG_PIN_1, ECHO_PIN_1) < LEFT_WALL && getDistance(TRIG_PIN_2, ECHO_PIN_2) < BACK_WALL) {
      return true;
    }
  }
  return false;
}

void resp_to_orient(void) {
  if (state == STATE_ORIENTATION) {
    previous = state;
    state = STATE_CORNER;
    Serial.println("ori found, now locking position");
  }
}

uint8_t test_for_wall(void) {
  if (state == STATE_CORNER) {
    Serial.println("checking for wall");
    if (limitLeftTriggered && limitBackTriggered) {
      return true;
    }
  }
  return false;
}

void resp_to_wall(void) {
  if (state == STATE_CORNER) {
    previous = state;
    state = STATE_FIND_CENTRELINE;
    Serial.println("in corner, now going to center line");
  }
}

//
uint8_t test_for_center(void) {
  if (state == STATE_FIND_CENTRELINE) {
    // Read the analog values from both line sensors
    int frontSensorValue = digitalRead(LINE_FRONT);
    int backSensorValue = analogRead(LINE_BACK);

    // debug
    Serial.print("Front Sensor: ");
    Serial.print(frontSensorValue);
    Serial.print("\t Back Sensor: ");
    Serial.println(backSensorValue);

    // Back sensor needs to > threshold on line
    if (frontSensorValue && backSensorValue < LINE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void resp_to_center(void) {
  if (state == STATE_FIND_CENTRELINE) {
    previous = state;
    state = STATE_FORWARD;
    Serial.println("now on centerline");
  }
}

uint8_t test_for_hog_line(void) {
  if (state == STATE_FORWARD) {
    // Read the analog values from both hog line sensors
    int leftHogSensorValue = analogRead(HOG_LEFT);
    int rightHogSensorValue = analogRead(HOG_RIGHT);

    // Debugging: Print the sensor values to the Serial monitor (optional)
    Serial.print("Hog Left Sensor: ");
    Serial.print(leftHogSensorValue);
    Serial.print("\t Hog Right Sensor: ");
    Serial.println(rightHogSensorValue);

    // Check if the robot is on the hog line
    if (leftHogSensorValue < LINE_THRESHOLD && rightHogSensorValue < LINE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

void resp_to_hog_lines(void) {
  if (state == STATE_FORWARD) {
    Serial.println("omg im hitting the hogline");
    previous = state;

    state = STATE_STOP;

    Serial.println("no longer on hogline");
  }
}

void checkGlobalEvents(void) {

  // if (TestLedTimerExpired()) RespLedTimerExpired();
  // if (TestForKey()) RespToKey();
  // if (TestMoveExpired()) RespMoveExpired();
  // if (TestForLightOn())  RespToLightOn();
  // if (TestForLightOff()) RespToLightOff();

  if (test_for_orient()) resp_to_orient();
  if (test_for_wall()) resp_to_wall();
  if (test_for_center()) resp_to_center();
  if (test_for_hog_line()) resp_to_hog_lines();
}
