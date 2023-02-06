#include <BURT_arm_constants.h>
#include <BURT_arm_IK.h>
#include <BURT_arm_motor.h>
#include <BURT_can.h>

/* TODO: 
 * Properly integrate CAN protocol using new structs. This uses the old version of CAN.
 * Modify CAN protocol to allow manual and IK control (see handler function)
 * Add proper parseSerial function to parse data over Serial (not strings!)
 * Detect stalls/slips
*/

// Pinouts
#define SWIVEL_EN_PIN 35
#define SWIVEL_CS_PIN 10
#define SWIVEL_LS_PIN 0
#define SWIVEL_STEPS_PER_180 200000
#define SWIVEL_SPEED 51200
#define SWIVEL_ACCEL 51200

#define EXTEND_EN_PIN 40 // 34
#define EXTEND_CS_PIN 36 // 37
#define EXTEND_LS_PIN 0
#define EXTEND_STEPS_PER_180 220000
#define EXTEND_SPEED 51200
#define EXTEND_ACCEL 51200

// DEMO: Two parallel motors for the shoulder joint, in opposite directions.
#define EXTEND2_EN_PIN 34 // 40
#define EXTEND2_CS_PIN 37 // 36
#define EXTEND2_LS_PIN 0
#define EXTEND2_STEPS_PER_180 -220000

// Limits for the joints, in terms of radians from their limit switches
// DEMO: All joints are limited to +- 30 degrees. Because we check from the 
// limit switches, this means the limits are [0, 60] degrees, or [0, pi/3] rad
#define SWIVEL_MIN 0
#define SWIVEL_MAX PI
#define SWIVEL_CURRENT 1200

#define EXTEND_MIN 0
#define EXTEND_MAX PI
#define EXTEND_CURRENT 2800

// CanBus IDs
#define DEMO_HANDLER_ID 1

#define MAX_X 440
#define MAX_Y 440
#define MAX_Z 440
#define MIN_RAD 0
#define MAX_RAD PI

#define DEBUG true

/* The (x, y, z) coordinates of the gripper. 

 These coordinates are passed to the IK algorithms which return angles for the 
 joints of the arm, [swivel], [extend], and [lift]. 
*/ 

StepperMotor swivel = StepperMotor(SWIVEL_CS_PIN, SWIVEL_EN_PIN, SWIVEL_LS_PIN, SWIVEL_CURRENT, SWIVEL_MIN, SWIVEL_MAX, SWIVEL_STEPS_PER_180, SWIVEL_SPEED, SWIVEL_ACCEL, "Swivel");
StepperMotor extend1 = StepperMotor(EXTEND_CS_PIN, EXTEND_EN_PIN, EXTEND_LS_PIN, EXTEND_CURRENT, EXTEND_MIN, EXTEND_MAX, EXTEND_STEPS_PER_180, EXTEND_SPEED, EXTEND_ACCEL, "Extend1");
StepperMotor extend2 = StepperMotor(EXTEND2_CS_PIN, EXTEND2_EN_PIN, EXTEND2_LS_PIN, EXTEND_CURRENT, EXTEND_MIN, EXTEND_MAX, EXTEND2_STEPS_PER_180, EXTEND_SPEED, EXTEND_ACCEL, "Extend2");

Angles zeroAngles(0, PI/2, PI);
Coordinates origin, position;

void demoHandler(const CanMessage& message) {
	float data[8];
	for (int index = 0; index < 8; index++) {
		data[index] = message.buf[index] - 100;
	}
	float x = (data[0] / 100) * MAX_X;
	float y = (data[1] / 100) * MAX_Y;
	float z = (data[2] / 100) * MAX_Z;

	// This code moves to the given coordinates:
	Coordinates offset(x, y, z);
	updatePosition(offset);

	// This code moves to the given angles:
	// float theta = (data[0] / 100) * SWIVEL_MAX;
	// float B = (data[2] / 100) * EXTEND_MAX;
	// swivel.moveTo(theta);
	// extend1.moveTo(B);
	// extend2.moveTo(B);	
}

void setup() {
	Serial.begin(9600);
	
	pinMode(SWIVEL_CS_PIN, OUTPUT);
	pinMode(EXTEND_CS_PIN, OUTPUT);
	pinMode(EXTEND2_CS_PIN, OUTPUT);
	digitalWrite(SWIVEL_CS_PIN, HIGH);
	digitalWrite(EXTEND_CS_PIN, HIGH);
	digitalWrite(EXTEND2_CS_PIN, HIGH);

	swivel.setup();
	extend1.setup();
	extend2.setup();
	Serial.println("Finished stepper motor initialization.");

	BurtCan::setup();
	BurtCan::registerHandler(DEMO_HANDLER_ID, demoHandler);
	Serial.println("Finished CAN bus initialization.");

	origin = ArmIK::calculatePosition(zeroAngles);
	position = origin;
	Serial.print("Origin: ");
	origin.print();

	delay(5000);
	Serial.println("Testing...");
	parseSerialCommand("debug-swivel 150000");
	delay(5000);
	parseSerialCommand("debug-swivel -150000");
	delay(10000);
	parseSerialCommand("debug-swivel 0");
	delay(5000);
	parseSerialCommand("debug-extend -200000");
}

void loop() {
	if (DEBUG) {
		String input = Serial.readString();
		parseSerialCommand(input);
	}

	BurtCan::update();
}

/// Updates the (x, y, z) coordinates of the gripper, provided it is safe.
void updatePosition(Coordinates offset) {
	Coordinates destination = origin + offset;
	Serial.print("\nTrying to go to: ");
	destination.print();
	Angles newAngles = ArmIK::calculateAngles(destination);
	if (newAngles.isFailure()) {
		Serial.println("IK failed -- Ignoring.");
		return;
	}
	Serial.print("IK finished: ");
	newAngles.print();
	swivel.moveTo(newAngles.theta);
	extend1.moveTo(newAngles.B);
	extend2.moveTo(newAngles.B);
	position = destination;
}

void parseSerialCommand(String input) {
	// Supported commands: 
	// 	x y z: move to origin + (x, y, z)
	// 	reset: move to origin
	// 	debug-swivel n: swivel to n steps 
	// 	debug-extend n: extend to n steps 
	// 	precise-swivel: swivel by n radians (without IK)
	// 	precise-extend: extend by n radians (without IK)

	if (input == "") return;
	Serial.println("User inputted: " + input);

	// Parse "x y z" command
	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	int delimiter2 = input.indexOf(" ", delimiter + 1);
	if (delimiter2 != -1) {  // was given an x, y, z command
		float x = input.substring(0, delimiter).toFloat();
		float y = input.substring(delimiter + 1, delimiter2).toFloat();
		float z = input.substring(delimiter2).toFloat();
		Coordinates destination(x, y, z);
		updatePosition(destination);
		return;
	}
	// Parse commands of the form "command n"
	String command = input.substring(0, delimiter);
	float arg = input.substring(delimiter + 1).toFloat();

	if (command == "precise-swivel") {
		swivel.moveTo(arg);
	} else if (command == "precise-extend") {
		extend1.moveTo(arg);
		extend2.moveTo(arg);
	} else if (command == "debug-swivel") { 
		swivel.debugMoveToStep(arg); 
	} else if (command == "debug-extend") { 
		extend1.debugMoveToStep(arg);
		extend2.debugMoveToStep(-arg);
	} else if (command == "reset") {
		updatePosition(Coordinates());
	} else Serial.println("Unrecognized command");
}
