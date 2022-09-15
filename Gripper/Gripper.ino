#include <BURT_arm_constants.h>
#include <BURT_arm_IK.h>
#include <BURT_arm_motor.h>
#include <BURT_can.h>

/* TODO: 
 - handle gyroscope inputs
 - handle recoil
*/

// Pinouts

// Motor driver 2
#define LIFT_EN_PIN 34
#define LIFT_CS_PIN 37
#define LIFT_LS_PIN 9

// Motor driver 3
#define GRIP_EN_PIN 40
#define GRIP_CS_PIN 36
#define GRIP_LS_PIN 8

// Limits for the joints, in terms of radians from their limit switches
// DEMO: All joints are limited to +- 30 degrees. Because we check from the 
// limit switches, this means the limits are [0, 60] degrees, or [0, pi/3] rad
#define LIFT_MIN 0
#define LIFT_MAX PI
#define LIFT_CURRENT 400
#define LIFT_STEPS_PER_180 750000
#define LIFT_SPEED 51200*2
#define LIFT_ACCEL 51200*2

#define GRIP_MIN 0
#define GRIP_MAX PI
#define GRIP_CURRENT 800
#define GRIP_STEPS_PER_180 750000
#define GRIP_SPEED 51200*2
#define GRIP_ACCEL 51200*2

// When the joints are at their maximum angles, these are the coordinates of the gripper.
#define CALIBRATED_X 0
#define CALIBRATED_Y 366.420
#define CALIBRATED_Z 1117.336

// CanBus IDs
#define DEMO_HANDLER_ID 1

#define MAX_X 440
#define MAX_Y 440
#define MAX_Z 440
#define MIN_RAD 0
#define MAX_RAD PI

#define DEBUG true

// CanBus Signal IDs
#define CAN_SIGNAL_DATA_PACKET_1 0x1C
#define CAN_SIGNAL_DATA_PACKET_2 0x1D

// Misc
#define LOW_CURRENT 1300
#define HIGH_CURRENT 2200

/* The (x, y, z) coordinates of the gripper. 

 These coordinates are passed to the IK algorithms which return angles for the 
 joints of the arm, [swivel], [extend], and [lift]. 
*/ 
StepperMotor lift = StepperMotor(LIFT_CS_PIN, LIFT_EN_PIN, LIFT_LS_PIN, LIFT_CURRENT, LIFT_MIN, LIFT_MAX, LIFT_STEPS_PER_180, LIFT_SPEED, LIFT_ACCEL, "Lift");
StepperMotor grip = StepperMotor(GRIP_CS_PIN, GRIP_EN_PIN, GRIP_LS_PIN, GRIP_CURRENT, GRIP_MIN, GRIP_MAX, GRIP_STEPS_PER_180, GRIP_SPEED, GRIP_ACCEL, "Grip");

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

	Coordinates offset(x, y, z);
	updatePosition(offset);

	// Serial.println()
}

void setup() {
	Serial.begin(9600);
	
	pinMode(LIFT_CS_PIN, OUTPUT);
	digitalWrite(LIFT_CS_PIN, HIGH);

	lift.setup();
	grip.setup();
	Serial.println("Finished stepper motor initialization.");
	BurtCan::setup();
	BurtCan::registerHandler(CAN_ROTATE_ID, rotateHandler);
	BurtCan::registerHandler(CAN_PINCH_ID, pinchHandler);
	BurtCan::registerHandler(CAN_LIFT_ID, liftHandler);
	BurtCan::registerHandler(CAN_PRECISE_PINCH_ID, precisePinchHandler);
	BurtCan::registerHandler(CAN_CALIBRATE_ID, calibrateHandler);
	Serial.println("Finished CAN bus initialization.");
}

void loop() {
	if (DEBUG) {
		String input = Serial.readString();
		parseSerialCommand(input);
	}
	// delay(1000);
	broadcastPackets();

	// Temporary Serial Monitor interface for testing
	String input = Serial.readString();
	parseSerialCommand(input);
	delay(500);
}

/**
 * @brief A simple struct to help with storing 16 bit numbers as individual bytes.
 */
struct split_uint16
{
	uint8_t msb;
	uint8_t lsb;
};
/**
 * @brief Splits a 16 bit number into 2 bytes (most significant byte and least significant byte)
 * 
 * @param val The number to split
 * @return struct split_uint16
 */
struct split_uint16
splitInt(uint16_t val)
{
	struct split_uint16 split = {val >> 8, val & 0xff};
	return split;
}

/**
 * Will send out data packets in accordance with the "CAN Codes" google doc specification.
 */
void broadcastPackets()
{
	// The angles (represented) as floats must be stored as 2 bytes each to send over CAN. Since we know that the radians will be between 0-2pi, we can divide by 2pi to get the number as a proportion, then take 5 decimals of precision by multiplying and truncating.
	struct split_uint16 rotate_current = splitInt((uint16_t)(rotate.getCurrent() / (2 * PI) * 100000));
	struct split_uint16 rotate_target = splitInt((uint16_t)(rotate.getAngle() / (2 * PI) * 100000));
	struct split_uint16 pinch_current = splitInt((uint16_t)(pinch.getCurrent() / (2 * PI) * 100000));
	struct split_uint16 pinch_target = splitInt((uint16_t)(pinch.getAngle() / (2 * PI) * 100000));
	struct split_uint16 lift_current = splitInt((uint16_t)(lift.getCurrent() / (2 * PI) * 100000));
	struct split_uint16 lift_target = splitInt((uint16_t)(lift.getAngle() / (2 * PI) * 100000));
	uint8_t data1[8] = {rotate_current.lsb, rotate_current.msb, rotate_target.lsb, rotate_target.msb, pinch_current.lsb, pinch_current.msb, pinch_target.lsb, pinch_target.msb};
	BurtCan::send(CAN_SIGNAL_DATA_PACKET_1, data1);

	// The following flags are determined according to the "CAN Codes" google doc.
	uint8_t motor_info = 0b00000000;
	if (!rotate.isFinished())
		motor_info |= 0b00100000;
	if (rotate.readLimitSwitch())
		motor_info |= 0b00010000;
	if (!pinch.isFinished())
		motor_info |= 0b00001000;
	if (pinch.readLimitSwitch())
		motor_info |= 0b00000100;
	if (!lift.isFinished())
		motor_info |= 0b00000010;
	if (lift.readLimitSwitch())
		motor_info |= 0b00000001;
	uint8_t data2[8] = {lift_current.lsb, lift_current.msb, lift_target.lsb, lift_target.msb, motor_info, 0, 0, 0};
	BurtCan::send(CAN_SIGNAL_DATA_PACKET_2, data2);
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
	pinch.moveTo(pinch.angle + (ArmConstants::rotationSpeed * arg));
}

void rotateHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	rotate.moveTo(rotate.angle + (ArmConstants::rotationSpeed * arg));
}

void liftHandler(const CanMessage& message) {
	float arg = (float) message.buf[0] - 1;
	lift.moveTo(lift.angle + (ArmConstants::rotationSpeed * arg));
}

	// CAN is not working on the gripper board. See [setup].
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
	lift.moveTo(newAngles.C - (PI/2));
	position = destination;
}

void parseSerialCommand(String input) {
	// Supported commands: 
	// 	x y z: move to origin + (x, y, z)
	// 	reset: move to origin
	// 	debug-lift n: lift to n steps 
	// 	debug-rotate n: rotate to n steps 
	// 	precise-lift: lift by n radians (without IK)
	// 	precise-rotate: rotate by n radians (without IK)

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

	if (command == "precise-lift") {
		lift.moveTo(arg);
	} else if (command == "debug-lift") { 
		lift.debugMoveToStep(arg); 
	} else if (command == "debug-grip") {
		grip.debugMoveToStep(arg);
	} else if (command == "reset") {
		updatePosition(Coordinates());
	} else Serial.println("Unrecognized command");
}
