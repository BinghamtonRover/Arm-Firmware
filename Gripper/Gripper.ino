#include <BURT_arm_constants.h>
#include <BURT_arm_motor.h>

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

Motor gripperPinchMotor = Motor(BurtArmConstants::pinchLimits);
Motor gripperLiftMotor = Motor(BurtArmConstants::gripperLiftLimits);
Motor gripperRotateMotor = Motor(BurtArmConstants::gripperRotateLimits);

void setup() { }

void loop() { }

void calibrate();

void parseCommand(int command, double arg) {
  switch (command) {
    // Gripper controls:
    case 7:  // open gripper. arg is extent, between -1 and 1
      gripperPinchMotor.safeUpdate(gripperOpen + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 8:  // rotate gripper. arg is extent, between -1 and 1
      gripperRotateMotor.safeUpdate(gripperRotateAngle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 9:  // lift gripper. arg is extent, between -1 and 1
      gripperLiftMotor.safeUpdate(gripperLiftAngle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 10:  // precise open. arg is direction, -1 or 1
      gripperPinchMotor.safeUpdate(gripperOpen + (BurtArmConstants::angleIncrement * arg));
      break;
    case 11:  // calibrate. No arg
      calibrate();
      break;
  }
}
