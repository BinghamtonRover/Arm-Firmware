#include <Motor.h>
#include <BURT_Constants.h>

/* The extent to which the gripper is open. 

 Ranges from 0 to 1, with 0 being closed (0% open) and 1 being open (100% open).
*/
double gripperOpen = 0;

/* The gripper's lift angle, separate from the arm.
	
 Ranges from -π to π, with negative pointing down.
*/
double gripperLiftAngle = 0;

/* The gripper's lift angle, separate from the arm. 
	
 Ranges from 0 to 2π, representing a whole circle.
*/
double gripperRotateAngle = 0;

Motor gripperPinchMotor = Motor(limits=BurtConstants::extendLimits);
Motor gripperLiftMotor, gripperRotateMotor;

void setup() {
  // initialize motors
  gripperPinchMotor.init();
  gripperLiftMotor.init();
  gripperRotateMotor.init();
}

void loop() {
  // write the current angles to each motor
  gripperPinchMotor.write(gripperOpen);  // TODO: convert % to angle
  gripperLiftMotor.write(gripperLiftAngle);
  gripperRotateMotor.write(gripperRotateAngle);
}

void calibrate();

void parseCommand(int command, double arg) {
  switch (command) {
    // Gripper controls:
    case 7:  // open gripper. arg is extent, between -1 and 1
      gripperPinchMotor.updateAngle(gripperOpen, gripperOpen + (BurtConstants::rotationSpeed * arg), BurtConstants::pinchLimits);
      break;
    case 8:  // rotate gripper. arg is extent, between -1 and 1
      gripperRotateMotor.updateAngle(gripperRotateAngle, gripperRotateAngle + (BurtConstants::rotationSpeed * arg), BurtConstants::gripperRotateLimits);
      break;
    case 9:  // lift gripper. arg is extent, between -1 and 1
      gripperLiftMotor.updateAngle(gripperLiftAngle, gripperLiftAngle + (BurtConstants::rotationSpeed * arg), BurtConstants::gripperLiftLimits);
      break;
    case 10:  // precise open. arg is direction, -1 or 1
      gripperPinchMotor.updateAngle(gripperOpen, gripperOpen + (BurtConstants::angleIncrement * arg), BurtConstants::pinchLimits);
      break;
    case 11:  // calibrate. No arg
      calibrate();
      break;
  }
}
