#include <IK.h>
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
 *  7. openCloseFingers() needs a way to check if it's closed around
 *  an object, right now it just checks when the fingers touch
 *  8. Add in Limit checking
*/

#define J1Sensor 1 //Still have to figure out what pins are for what
#define J2Sensor 2 //Sensor pins must be analog, all others it doesn't
#define J3Sensor 3 //make a difference
#define J4Sensor 4
#define J5Sensor 5
#define J6Sensor 6

#define J1Lim1 19
#define J1Lim2 20
#define J2Lim1 21
#define J2Lim2 22
#define J3Lim1 23
#define J3Lim2 24
#define J4Lim1 25
#define J4Lim2 26
#define J6Lim1 27
#define J6Lim2 28

#define m1Step 7
#define m1Dir 8
#define m2Step 9
#define m2Dir 10
#define m3Step 11
#define m3Dir 12
#define m4Step 13
#define m4Dir 14
#define m5Step 15
#define m5Dir 16
#define m6Step 17
#define m6Dir 18

#define movementResolution 5
#define microsPulseWidth 10
#define microsBetween 100

#define accuracy .01 // This value is in radians

#define closedSensorValue = 0; //Must set to their real values once
#define openSensorValue = 1023; //we actually have it put together

bool gripperOpen;
bool gripperClosed;

double j1Angle;
double j2Angle;
double j3Angle;
double nextAngles[3] = {1, 1, 1};
double currentAngles[3];

double nextX;
double nextY;
double nextZ;

IK IK;

enum stateType {  //pretty sure we can use the same enum and handler for all of them
  forward,        //we'll just not have buttons assigned to setting the unnecessary 
  backward,       //parts of them
  left,
  right,
  up,
  down,
  stay
};

stateType mainMovement = stay;
stateType gripperFingers = stay;
stateType gripperRotation = stay;
stateType gripperLift = stay;

double sensorReadJ1(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

double sensorReadJ2(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

double sensorReadJ3(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

double sensorReadJ4(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

double sensorReadJ5(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

void sensorReadJ6(int sensorValue) {
  return ((double)sensorValue * TWO_PI) / 1023.0;
}

stateType int2Enum(int i) {
  stateType ret;
  switch(i) {
    case 0:
      ret = left;
      break;
    case 1:
      ret = right;
      break;
    case 2:
      ret = forward;
      break;
    case 3:
      ret = backward;
      break;
    case 4:
      ret = up;
      break;
    case 5:
      ret = down;
      break;
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

void step(int motorNumber) {
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
    case 0:
      mainMovement = newState;
      break;
    case 1:
      gripperLift = newState;
      break;
    case 2:
      gripperRotation = newState;
      break;
    case 3:
      gripperFingers = newState;
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

  pinMode(J1Sensor, INPUT);
  pinMode(J1Sensor, INPUT);
  pinMode(J1Sensor, INPUT);
  pinMode(J1Sensor, INPUT);
  pinMode(J1Sensor, INPUT);
  pinMode(J1Sensor, INPUT);

  pinMode(J1Lim1, INPUT_PULLUP);
  pinMode(J1Lim2, INPUT_PULLUP);
  pinMode(J2Lim1, INPUT_PULLUP);
  pinMode(J2Lim2, INPUT_PULLUP);
  pinMode(J3Lim1, INPUT_PULLUP);
  pinMode(J3Lim2, INPUT_PULLUP);
  pinMode(J4Lim1, INPUT_PULLUP);
  pinMode(J4Lim2, INPUT_PULLUP);
  pinMode(J6Lim1, INPUT_PULLUP);
  pinMode(J6Lim2, INPUT_PULLUP);
  
  
  currentAngles[0] = sensorReadJ1(analogRead(J1Sensor));
  currentAngles[1] = sensorReadJ2(analogRead(J2Sensor));
  currentAngles[2] = sensorReadJ3(analogRead(J3Sensor));
  
  IK.endPointPos(currentAngles);
}

void loop() {
  gripperLevel();
  runInverseKinematics();
  gripperLevel();
  gripperTwist();
  openCloseFingers();
}
