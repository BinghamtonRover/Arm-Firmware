#include <IK.h>
#include <Motor.h>
#include <BURT_Constants.h>

#define Motor

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

Motor armSwivelMotor, armExtendMotor, armLiftMotor;
TMC5160Stepper armSwivelMotor = newDriver(cs_pin, )

// 2200mA for the arms + gripper lift
// 1300mA for the gripper rotate + pinch

void setup() {
	// initialize motors
	armSwivelMotor.init();
	armExtendMotor.init();
	armLiftMotor.init();
}

void loop() {
	// write the current angles to each motor
	armSwivelMotor.write(armSwivelAngle);
	armExtendMotor.write(armExtendAngle);
	armLiftMotor.write(armLiftAngle);
}

void updatePosition(double newX, double newY, double newZ) {
	/* Updates the (x, y, z) coordinates of the gripper, provided it is safe. */
	Angles newAngles = IK::calculateAngles(newX, newY, newZ);
	bool success = armSwivelMotor.updateAngle(newAngles.theta)
		&& armExtendMotor.updateAngle(newAngles.B)
		&& armLiftMotor.updateAngle(newAngles.C);

	// Safe to assign new position and angles
	if (success) {
		gripperX = newX; gripperY = newY; gripperZ = newZ;
	}
}

void calibrate();

void parseCommand(int command, double arg) {
	switch (command) {
		case 1:  // swivel arm. arg is extent, between -1 and 1
			updatePosition(gripperX + BurtConstants::movementSpeed * arg, gripperY, gripperZ);
			break;
		case 2:  // lift arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY, gripperZ + BurtConstants::movementSpeed * arg);
			break;
		case 3:  // extend arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY + BurtConstants::movementSpeed * arg, gripperZ); 
			break;
		case 4:  // precise swivel. arg is direction, -1 or 1
			armSwivelMotor.updateAngle(armSwivelAngle + (BurtConstants::angleIncrement * arg));
			break;
		case 5:  // precise lift. arg is direction, -1 or 1
			armLiftMotor.updateAngle(armLiftAngle + (BurtConstants::angleIncrement * arg));
			break;
		case 6:  // precise extend. arg is direction, -1 or 1
			armExtendMotor.updateAngle(armExtendAngle + (BurtConstants::angleIncrement * arg));
			break;
		case 11:  // calibrate. No arg
			calibrate();
			break;
	}
}
