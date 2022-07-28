#include <BURT_arm_constants.h>
#include <BURT_arm_motor.h>
#include <BURT_can.h>

// Pinouts
#define ROTATE_EN_PIN 35
#define ROTATE_CS_PIN 10

#define LIFT_EN_PIN 34
#define LIFT_CS_PIN 37

#define PINCH_EN_PIN 40
#define PINCH_CS_PIN 36

#define ROTATE_MIN -2 * PI
#define ROTATE_MAX 2 * PI

#define LIFT_MIN 0 
#define LIFT_MAX 2.004

#define PINCH_MIN 0
#define PINCH_MAX 1.867

// Limits for the joints, in terms of radians from their limit switches
#define ROTATE_LIMIT INFINITY  // can swivel in endless circles
#define PINCH_LIMIT 1.867
#define LIFT_LIMIT 2.004

// CAN bus IDs
#define CAN_ROTATE_ID 0
#define CAN_PINCH_ID 0
#define CAN_LIFT_ID 0
#define CAN_PRECISE_PINCH_ID 0
#define CAN_CALIBRATE_ID 0

// Misc
#define LOW_CURRENT 1300
#define HIGH_CURRENT 2200


Motor rotate = Motor(ROTATE_CS_PIN, ROTATE_EN_PIN, LOW_CURRENT, ROTATE_MIN, ROTATE_MAX, 99.51);
Motor pinch = Motor(PINCH_CS_PIN, PINCH_EN_PIN, LOW_CURRENT, PINCH_MIN, PINCH_MAX, 199.02);
Motor lift = Motor(LIFT_CS_PIN, LIFT_EN_PIN, HIGH_CURRENT, LIFT_MIN, LIFT_MAX, 29.16);

void setup() { 
	Serial.begin(9600);
	Serial.println("Finished stepper motor initialization.");
	// BurtCan::setup();
	// BurtCan::registerHandler(CAN_ROTATE_ID, rotateHandler);
	// BurtCan::registerHandler(CAN_PINCH_ID, pinchHandler);
	// BurtCan::registerHandler(CAN_LIFT_ID, liftHandler);
	// BurtCan::registerHandler(CAN_PRECISE_PINCH_ID, precisePinchHandler);
	// BurtCan::registerHandler(CAN_CALIBRATE_ID, calibrateHandler);
	// Serial.println("Finished CAN bus initialization.");
}

void loop() {
	while (!pinch.isFinished() || !rotate.isFinished() || !lift.isFinished()) {
		pinch.fixPotentialStall();
		rotate.fixPotentialStall();
		lift.fixPotentialStall();
	}
	// delay(1000);

	// Temporary Serial Monitor interface for testing
	String input = Serial.readString();
	parseSerialCommand(input);
	delay(500);
}

void calibrate() {
	pinch.calibrate();
	rotate.calibrate();
	lift.calibrate();
}

/* Handler functions for the controller inputs. 
 
 Many messages will send an argument from -1 to 1. However, [CanMessage]s can
 only send unsigned integers. Because of this, we need to convert to float and
 subtract 1, since the message will be from [0, 2].
*/

void pinchHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	pinch.safeUpdate(pinch.angle + (BurtArmConstants::rotationSpeed * arg));
}

void rotateHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	rotate.safeUpdate(rotate.angle + (BurtArmConstants::rotationSpeed * arg));
}

void liftHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	lift.safeUpdate(lift.angle + (BurtArmConstants::rotationSpeed * arg));
}

void precisePinchHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	pinch.safeUpdate(pinch.angle + (BurtArmConstants::angleIncrement * arg));
}

void calibrateHandler(const CanMessage& message) {
	calibrate();
}

void parseSerialCommand(String input) {
	/* 
		pinch: [-1, 1]
		rotate: [-1, 1]
		lift: [-1, 1]
		precisePinch: {-1, 1}
		calibrate;
	*/

	if (input == "calibrate") calibrate();

	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	String command = input.substring(0, delimiter);
	String part2 = input.substring(delimiter + 1);
	float arg = part2.toFloat();
	if (arg < -1 || arg > 1) {
		Serial.println("Argument must be between -1 and 1");
		return;
	}

	if (command == "pinch") pinch.safeUpdate(pinch.angle + arg);
	else if (command == "rotate") rotate.safeUpdate(rotate.angle + arg);
	else if (command == "precise-pinch") {
		pinch.safeUpdate(pinch.angle + (BurtArmConstants::angleIncrement * arg));
	} else {
		Serial.println("Unknown command: " + command);
	}

}
