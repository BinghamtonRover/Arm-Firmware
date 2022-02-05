#include <BURT_arm_constants.h>
#include <BURT_arm_motor.h>

Motor pinch = Motor(10, 23, 1300, BurtArmConstants::pinchLimits);
Motor lift = Motor(10, 23, 2200, BurtArmConstants::gripperLiftLimits);
Motor rotate = Motor(10, 23, 1300, BurtArmConstants::gripperRotateLimits);

void setup() { }

void loop() {
  while (!pinch.isFinished() || !rotate.isFinished() || !lift.isFinished()) {
    pinch.fixPotentialStall();
    rotate.fixPotentialStall();
    lift.fixPotentialStall();
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
      pinch.safeUpdate(open.angle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 8:  // rotate gripper. arg is extent, between -1 and 1
      rotate.safeUpdate(rotate.angle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 9:  // lift gripper. arg is extent, between -1 and 1
      lift.safeUpdate(lift.angle + (BurtArmConstants::rotationSpeed * arg));
      break;
    case 10:  // precise open. arg is direction, -1 or 1
      pinch.safeUpdate(open.angle + (BurtArmConstants::angleIncrement * arg));
      break;
    case 11:  // calibrate. No arg
      calibrate();
      break;
  }
}
