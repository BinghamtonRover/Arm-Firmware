#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"
#include "src/ik/IK.h"

#include "pinouts.h"

// When the joints are at their maximum angles, these are the coordinates of the gripper.
#define CALIBRATED_X 0
#define CALIBRATED_Y 366.420
#define CALIBRATED_Z 1117.336

// CanBus IDs
#define CAN_SWIVEL_ID 0
#define CAN_EXTEND_ID 0
#define CAN_LIFT_ID 0
#define CAN_PRECISE_SWIVEL_ID 0
#define CAN_PRECISE_EXTEND_ID 0
#define CAN_PRECISE_LIFT_ID 0
#define CAN_CALIBRATE_ID 0

/* The (x, y, z) coordinates of the gripper. 

 These coordinates are passed to the IK algorithms which return angles for the 
 joints of the arm, [swivel], [shoulder], and [elbow]. 
*/ 
double gripperX = 0, gripperY = 0, gripperZ = 0;

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");
	delay(1000);

	// can.setup();

	swivel.presetup();
	shoulder.presetup();
	elbow.presetup();

	swivel.setup();
	shoulder.setup();
	elbow.setup();
	delay(10);

	calibrate();
	Serial.println("Arm subsystem ready");
}

void loop() {
	// while (!swivel.isFinished() || !shoulder.isFinished() || !elbow.isFinished()) {
	// 	swivel.fixPotentialStall();
	// 	shoulder.fixPotentialStall();
	// 	elbow.fixPotentialStall();
	// }
	// BurtCan::update();
	// delay(1000);

	// Temporary Serial Monitor interface for testing
	String input = Serial.readString();
	parseSerialCommand(input);
	delay(500);
}

void updatePosition(double newX, double newY, double newZ) {
	/* Updates the (x, y, z) coordinates of the gripper, provided it is safe. */
	Serial.print("Trying to go to:  ");
	Serial.print(newX);
	Serial.print(" ");	
	Serial.print(newY);
	Serial.print(" ");	
	Serial.println(newZ);
	Coordinates newCoordinates(newX, newY, newZ);
	Angles newAngles = ArmIK::calculateAngles(newCoordinates);
	Serial.print("IK finished: ");
	Serial.print(newAngles.theta);
	Serial.print(", ");
	Serial.print(newAngles.B);
	Serial.print(", ");	
	Serial.println(newAngles.C);	
	swivel.moveTo(newAngles.theta);
	shoulder.moveTo(newAngles.B);
	elbow.moveTo(newAngles.C);
	gripperX = newX; gripperY = newY; gripperZ = newZ;
}

void calibrate() {
	swivel.calibrate();
	shoulder.calibrate();
	elbow.calibrate();
	gripperX = CALIBRATED_X;
	gripperY = CALIBRATED_Y;
	gripperZ = CALIBRATED_Z;
}

/* Handler functions for the controller inputs. 
 
 Many messages will send an argument from -1 to 1. However, [CanMessage]s can
 only send unsigned integers. Because of this, we need to convert to float and
 subtract 1, since the message will be from [0, 2].
*/

void swivelHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	gripperX += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void extendHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	gripperY += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void liftHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	gripperZ += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void preciseSwivelHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	swivel.moveBy(ArmConstants::angleIncrement * arg);
}

void preciseLiftHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	elbow.moveBy(ArmConstants::angleIncrement * arg);
}

void preciseExtendHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	shoulder.moveBy(ArmConstants::angleIncrement * arg);
}

void calibrateHandler(const CanMessage& message) {
	calibrate();
}

void parseSerialCommand(String input) {
	Serial.println("User inputted: " + input);
	if (input == "calibrate") calibrate();

	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	int delimiter2 = input.indexOf(" ", delimiter + 1);
	if (delimiter2 != -1) {  // was given an x, y, z command
		float x = input.substring(0, delimiter).toFloat();
		float y = input.substring(delimiter + 1, delimiter2).toFloat();
		float z = input.substring(delimiter2).toFloat();
		Serial.print("Moving to: (");
			Serial.print(x); Serial.print(", ");
			Serial.print(y); Serial.print(", ");
			Serial.print(z); 
		Serial.println(").");
		gripperX += x;
		gripperY += y;
		gripperZ += z;
		updatePosition(gripperX, gripperY, gripperZ);
		return;
	}
	String command = input.substring(0, delimiter);
	float arg = input.substring(delimiter + 1).toFloat();

	if (command == "swivel") {
		gripperX += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	} else if (command == "shoulder") {
		gripperY += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	} else if (command == "elbow") {
		gripperZ += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	} else if (command == "precise-swivel") swivel.moveBy(arg);
	else if (command == "precise-elbow") elbow.moveBy(arg);
	else if (command == "precise-shoulder") shoulder.moveBy(arg);
	else Serial.println("Unrecognized command");
}
