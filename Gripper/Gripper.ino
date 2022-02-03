#include <BURT_arm_constants.h>
#include <BURT_arm_motor.h>

/* The extent to which the gripper is open. 

 Ranges from 0 to 1, with 0 being closed (0% open) and 1 being open (100% open).
*/
double openExtent = 0;

/* The gripper's lift angle, separate from the arm.
	
 Ranges from -π to π, with negative pointing down.
*/
double liftAngle = 0;

/* The gripper's lift angle, separate from the arm. 
	
 Ranges from 0 to 2π, representing a whole circle.
*/
double rotateAngle = 0;

Motor pinch = Motor(10, 23, 1300, BurtArmConstants::pinchLimits);
Motor lift = Motor(10, 23, 2200, BurtArmConstants::gripperLiftLimits);
Motor rotate = Motor(10, 23, 1300, BurtArmConstants::gripperRotateLimits);

double nextStallCheck = millis() + BurtArmConstants::stallCheckInterval;

void setup() { }

void loop() {
  while (!pinch.isFinished() || !rotate.isFinished() || !lift.isFinished()) {
    double currentTime = millis();
    if (currentTime >= nextStallCheck) {
      nextStallCheck = currentTime + BurtArmConstants::stallCheckInterval;
      if (pinch.didStall()) pinch.calibrate();
      if (rotate.didStall()) rotate.calibrate();
      if (lift.didStall()) lift.calibrate();
    }
  }
}

void calibrate() {
  pinch.calibrate();
  rotate.calibrate();
  lift.calibrate();
}

void parseCommand(int command, double arg) {
  switch (command) {
    // Gripper controls:
    case 7:  // open gripper. arg is extent, between -1 and 1
      pinch.safeUpdate(openExtent + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 8:  // rotate gripper. arg is extent, between -1 and 1
      rotate.safeUpdate(rotateAngle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 9:  // lift gripper. arg is extent, between -1 and 1
      lift.safeUpdate(liftAngle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 10:  // precise open. arg is direction, -1 or 1
      pinch.safeUpdate(openExtent + (BurtArmConstants::angleIncrement * arg));
      break;
    case 11:  // calibrate. No arg
      calibrate();
      break;
  }
}
