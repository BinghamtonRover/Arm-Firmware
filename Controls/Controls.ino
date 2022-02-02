#include "IK.h"
#include "motor.h"

/* The speed at which the arm will move in (x, y, z) space. 

 Does not necessarily correlate to the angular speed of the joints, since those
 are controlled separately by an IK algorithm based on these points. 

 Multiply by a factor between -1 and 1 to obtain a new speed and direction.
*/
#define MOVEMENT_SPEED 1

/* The default amount of radians at which to manually adjust the motors. 

 Multiply by either 1 or -1 to change the direction of the motion.
*/
#define ANGLE_INCREMENT 1

/* The default speed at which to rotate motors. 
 
 Multiply by a factor between -1 and 1 to obtain a new speed and direction.
*/
#define ROTATION_SPEED 1

/* The maximum difference in joint positions between "frames". */
#define MAX_DELTA 1

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

Motor armSwivelMotor, armExtendMotor, armLiftMotor;
Motor gripperPinchMotor, gripperLiftMotor, gripperRotateMotor;

void setup() {
	// initialize motors
	armSwivelMotor.init();
	armExtendMotor.init();
	armLiftMotor.init();
	gripperPinchMotor.init();
	gripperLiftMotor.init();
	gripperRotateMotor.init();
}

void loop() {
	// write the current angles to each motor
	armSwivelMotor.write(armSwivelAngle);
	armExtendMotor.write(armExtendAngle);
	armLiftMotor.write(armLiftAngle);
	gripperPinchMotor.write(gripperOpen);  // TODO: convert % to angle
	gripperLiftMotor.write(gripperLiftAngle);
	gripperRotateMotor.write(gripperRotateAngle);
}

bool updateAngle(double& currentAngle, double newAngle, double limits[2]) {
	/* Updates an angle if and only if it is safe to do so. */
	if (
		abs(currentAngle - newAngle) > MAX_DELTA ||  // cannot move too far
		newAngle < limits[0] || newAngle > limits[1]  // must be within limits
	) return false;
	currentAngle = newAngle;
	return true;
}

void updatePosition(double newX, double newY, double newZ) {
	/* Updates the (x, y, z) coordinates of the gripper, provided it is safe. */
	Angles newAngles = IK::calculateAngles(newX, newY, newZ);
	bool success = updateAngle(armSwivelAngle, newAngles.theta, IK::thetaLimits)
		&& updateAngle(armExtendAngle, newAngles.B, IK::bLimits)
		&& updateAngle(armLiftAngle, newAngles.C, IK::cLimits);

	// Safe to assign new position and angles
	if (success) {
		gripperX = newX; gripperY = newY; gripperZ = newZ;
	}
}

void calibrate();

void parseCommand(int command, double arg) {
	switch (command) {
		// Arm controls:
		case 1:  // swivel arm. arg is extent, between -1 and 1
			updatePosition(gripperX + MOVEMENT_SPEED * arg, gripperY, gripperZ);
			break;
		case 2:  // lift arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY, gripperZ + MOVEMENT_SPEED * arg);
			break;
		case 3:  // extend arm. arg is extent, between -1 and 1
			updatePosition(gripperX, gripperY + MOVEMENT_SPEED * arg, gripperZ); 
			break;
		case 4:  // precise swivel. arg is direction, -1 or 1
			updateAngle(armSwivelAngle, armSwivelAngle + (ANGLE_INCREMENT * arg), IK::thetaLimits);
			break;
		case 5:  // precise lift. arg is direction, -1 or 1
			updateAngle(armLiftAngle, armLiftAngle + (ANGLE_INCREMENT * arg), IK::cLimits);
			break;
		case 6:  // precise extend. arg is direction, -1 or 1
			updateAngle(armExtendAngle, armExtendAngle + (ANGLE_INCREMENT * arg), IK::bLimits);
			break;

		// Gripper controls:
		case 7:  // open gripper. arg is extent, between -1 and 1
			updateAngle(gripperOpen, gripperOpen + (ROTATION_SPEED * arg), IK::pinchLimits);
			break;
		case 8:  // rotate gripper. arg is extent, between -1 and 1
			updateAngle(gripperRotateAngle, gripperRotateAngle + (ROTATION_SPEED * arg), IK::gripperRotateLimits);
			break;
		case 9:  // lift gripper. arg is extent, between -1 and 1
			updateAngle(gripperLiftAngle, gripperLiftAngle + (ROTATION_SPEED * arg), IK::gripperLiftLimits);
			break;
		case 10:  // precise open. arg is direction, -1 or 1
			updateAngle(gripperOpen, gripperOpen + (ANGLE_INCREMENT * arg), IK::pinchLimits);
			break;

		// Misc
		case 11:  // calibrate. No arg
			calibrate();
			break;
	}
}
