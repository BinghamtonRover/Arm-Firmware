#include <rocs.hpp>

/* Buttons this plans to use
 *  Joystick/D-pad + 2 buttons for IK
 *  2 buttons to twist gripper
 *  1 button to toggle gripper up/flat/down
 *  2 buttons to open and close gripper
 *  
 * Things to still add: 1, 3, 5, 7
 *  1. Step counting to ensure the motors aren't stalling and
 *  adjust speed based off that.
 *  2. Fine tune the sensorRead functions.
 *  3. Double check all logic to ensure accuracy.
 *  4. If we need to cut down on storage space then we can get rid
 *  of the enum StateType and just use an unsigned short instead 
 *  because 0 - 6 could be swapped for up, down, left, etc. just 
 *  by sacrificing a bit of readability.
 *  5. Set address for ROCS and pins to their real values.
 *  6. Figure out which is clockwise and counterclockwise for our
 *  speed controllers then change the calls of setDir accordingly
 *  
 *  7. openCloseFingers() needs a way to check if it's closed around
 *  an object, right now it just checks when the fingers touch
 *  8. Add in Limit checking
*/

#define m1Step 0
#define m1Dir 30
#define m2Step 2
#define m2Dir 1
#define m3Step 4
#define m3Dir 3
#define m4Step 28
#define m4Dir 14
#define m5Step 7
#define m5Dir 8
#define m6Step 29
#define m6Dir 12

#define microsPulseWidth 10
#define microsBetween 100

enum stateType {  
  forward,        
  backward,
  stay
};

stateType joint1 = stay;
stateType joint2 = stay;
stateType joint3 = stay;
stateType joint4 = stay;
stateType joint5 = stay;
stateType joint6 = stay;

stateType int2Enum(int i) {
  stateType ret;
  switch(i) {
    case 0:
      ret = forward;
      break;
    case 1:
      ret = backward;
      break;
    case 2:
    default: 
      ret = stay;
      break;
  }
  return ret;
}

void setDir(int motorNumber, int dir) {
switch(motorNumber) {
  case 1:
      digitalWrite(m1Dir, dir);
      break;
  case 2:
      digitalWrite(m2Dir, dir);
      break;
  case 3:
      digitalWrite(m3Dir, dir);
      break;
  case 4:
      digitalWrite(m4Dir, dir);
      break;
  case 5:
      digitalWrite(m5Dir, dir);
      break;
  case 6:
      digitalWrite(m6Dir, dir);
      break;
  }
}

void takeStep(int motorNumber) {
  switch(motorNumber) {
    case 1:
      digitalWrite(m1Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m1Step, LOW);
      break;
    case 2:
      digitalWrite(m2Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m2Step, LOW);
      break;
    case 3:
      digitalWrite(m3Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m3Step, LOW);
      break;
    case 4:
      digitalWrite(m4Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m4Step, LOW);
      break;
    case 5:
      digitalWrite(m5Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m5Step, LOW);
      break;
    case 6:
      digitalWrite(m6Step, HIGH);
      delayMicroseconds(microsPulseWidth);
      digitalWrite(m6Step, LOW);
      break;
  }
}

void write_handler(uint8_t reg, uint8_t val) {
  stateType newState = int2Enum(val);
  switch(reg) {
    case 1:
      joint1 = newState;
      break;
    case 2:
      joint2 = newState;
      break;
    case 3:
      joint3 = newState;
      break;
    case 4:
      joiNt4 = newState;
      break;
    case 5:
      joint5 = newState;
      break;
    case 6:
      joint6 = newState;
      break;
  }
}

void setup() {
  rocs::init(0x03, "Arm", 10); //Address still to be determined
  rocs::set_write_handler(write_handler);

  pinMode(m1Step, OUTPUT);
  pinMode(m1Dir, OUTPUT);
  pinMode(m2Step, OUTPUT);
  pinMode(m2Dir, OUTPUT);
  pinMode(m3Step, OUTPUT);
  pinMode(m3Dir, OUTPUT);
  pinMode(m4Step, OUTPUT);
  pinMode(m4Dir, OUTPUT);
  pinMode(m5Step, OUTPUT);
  pinMode(m5Dir, OUTPUT);
  pinMode(m6Step, OUTPUT);
  pinMode(m6Dir, OUTPUT);
  
}

void loop() {
  switch(joint1) {
    case forward:
      setDir(1,1);
      takeStep(1);
    break;
    case backward:
      setDir(1,0);
      takeStep(1);
    break;
  }
  switch(joint2) {
    case forward:
      setDir(2,1);
      takeStep(2);
    break;
    case backward:
      setDir(2,0);
      takeStep(2);
    break;
  }
  switch(joint3) {
    case forward:
      setDir(3,1);
      takeStep(3);
    break;
    case backward:
      setDir(3,0);
      takeStep(3);
    break;
  }
  switch(joint4) {
    case forward:
      setDir(4,1);
      takeStep(4);
    break;
    case backward:
      setDir(4,0);
      takeStep(4);
    break;
  }
  switch(joint5) {
    case forward:
      setDir(5,1);
      takeStep(5);
    break;
    case backward:
      setDir(5,0);
      takeStep(5);
    break;
  }
  switch(joint6) {
    case forward:
      setDir(6,1);
      takeStep(6);
    break;
    case backward:
      setDir(6,0);
      takeStep(6);
    break;
  }
  delayMicroseconds(microsBetween);
}
