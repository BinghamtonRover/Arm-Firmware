#include <BURT_arm_constants.h>
#include <BURT_arm_IK.h>
#include <BURT_arm_motor.h>

/* The angles of the joints of the arm. 

 [swivelAngle] controls the rotation of the arm, [extendAngle] controls
 the base joint, and [liftAngle] controls the middle joint of the arm.

 These angles can be either manually controlled or automatically calculated 
 from the IK algorithm by passing in the desired coordinates of the gripper. 
 See [gripperX], [gripperY], and [gripperZ] and [IK::calculateAngles] for more.
*/

/* The (x, y, z) coordinates of the gripper. 

 These coordinates are passed to the IK algorithms which return angles for the 
 joints of the arm, [swivel], [extend], and [lift]. 
*/ 
double gripperX = 0, gripperY = 0, gripperZ = 0;

Motor swivel = Motor(10, 23, 2200, BurtArmConstants::swivelLimits);
Motor extend = Motor(10, 23, 2200, BurtArmConstants::extendLimits);
Motor lift = Motor(10, 23, 2200, BurtArmConstants::liftLimits);

void setup() { }

void loop() {
	while (!swivel.isFinished() || !extend.isFinished() || !lift.isFinished()) {
		swivel.fixPotentialStall();
		extend.fixPotentialStall();
		lift.fixPotentialStall();
	}
}

void updatePosition(double newX, double newY, double newZ) {
	/* Updates the (x, y, z) coordinates of the gripper, provided it is safe. */
	Angles newAngles = BurtArmIK::calculateAngles(newX, newY, newZ);
	swivel.safeUpdate(newAngles.theta);
	extend.safeUpdate(newAngles.B);
	lift.safeUpdate(newAngles.C);
	gripperX = newX; gripperY = newY; gripperZ = newZ;
}

void calibrate() {
	swivel.calibrate();
	extend.calibrate();
	lift.calibrate();
}

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
			swivel.safeUpdate(swivel.angle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 5:  // precise lift. arg is direction, -1 or 1
			lift.safeUpdate(lift.angle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 6:  // precise extend. arg is direction, -1 or 1
			extend.safeUpdate(extend.angle + (BurtArmConstants::angleIncrement * arg));
			break;
		case 11:  // calibrate. No arg
			calibrate();
			break;
	}
}
