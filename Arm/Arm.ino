#include <BURT_arm_constants.h>
#include <BURT_arm_IK.h>
#include <BURT_arm_motor.h>

/* The angles of the joints of the arm. 

 [armSwivelAngle] controls the rotation of the arm, [armExtendAngle] controls
 the base joint, and [armLiftAngle] controls the middle joint of the arm.

 These angles can be either manually controlled or automatically calculated 
 from the IK algorithm by passing in the desired coordinates of the gripper. 
 See [gripperX], [gripperY], and [gripperZ] and [IK::calculateAngles] for more.
*/
double armSwivelAngle = 0, armExtendAngle = 0, armLiftAngle = 0;

/* The (x, y, z) coordinates of the gripper. 

 These coordinates are passed to the IK algorithms which return angles for the 
 joints of the arm, [armSwivelAngle], [armExtendAngle], and [armLiftAngle]. 
*/ 
double gripperX = 0, gripperY = 0, gripperZ = 0;

Motor armSwivelMotor = Motor(BurtArmConstants::swivelLimits);
Motor armExtendMotor = Motor(BurtArmConstants::extendLimits);
Motor armLiftMotor = Motor(BurtArmConstants::liftLimits);

void setup() { }

void loop() { }

void updatePosition(double newX, double newY, double newZ) {
	/* Updates the (x, y, z) coordinates of the gripper, provided it is safe. */
	Angles newAngles = BurtArmIK::calculateAngles(newX, newY, newZ);
	armSwivelMotor.safeUpdate(newAngles.theta);
	armExtendMotor.safeUpdate(newAngles.B);
	armLiftMotor.safeUpdate(newAngles.C);
	gripperX = newX; gripperY = newY; gripperZ = newZ;
}

void calibrate() { }

void parseCommand(int command, double arg) {
	switch (command) {
		case 1:  // swivel arm. arg is extent, between -1 and 1
			updatePosition(gripperX + BurtArmConstants::movementSpeed * arg, gripperY, gripperZ);
			break;
		case 2:  // lift arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY, gripperZ + BurtArmConstants::movementSpeed * arg);
			break;
		case 3:  // extend arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY + BurtArmConstants::movementSpeed * arg, gripperZ); 
			break;
		case 4:  // precise swivel. arg is direction, -1 or 1
			armSwivelMotor.safeUpdate(armSwivelAngle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 5:  // precise lift. arg is direction, -1 or 1
			armLiftMotor.safeUpdate(armLiftAngle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 6:  // precise extend. arg is direction, -1 or 1
			armExtendMotor.safeUpdate(armExtendAngle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 11:  // calibrate. No arg
			calibrate();
			break;
	}
}
