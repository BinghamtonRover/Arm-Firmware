#include <BURT_arm_constants.h>
#include <BURT_arm_IK.h>
#include <BURT_arm_motor.h>
#include <BURT_can.h>

/* TODO:
 - handle gyroscope inputs
 - handle recoil
*/

// Pinouts
#define SWIVEL_EN_PIN 35
#define SWIVEL_CS_PIN 10
#define SWIVEL_LS_PIN 7

#define EXTEND_EN_PIN 40 // 34
#define EXTEND_CS_PIN 36 // 37
#define EXTEND_LS_PIN 8

#define LIFT_EN_PIN 34 // 40
#define LIFT_CS_PIN 37 // 36
#define LIFT_LS_PIN 9

// Limits for the joints, in terms of radians from their limit switches
#define SWIVEL_MIN -2 * PI // INFINITY // can swivel in endless circles
#define SWIVEL_MAX 2 * PI
#define SWIVEL_GEARBOX 46.656

#define EXTEND_MIN 2.00 // -0.698
#define EXTEND_MAX 2.18
#define EXTEND_GEARBOX -331.816

#define LIFT_MIN 0.698
#define LIFT_MAX 2.443
#define LIFT_GEARBOX 216

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

// CanBus Signal IDs
#define CAN_SIGNAL_DATA_PACKET_1 0x1C
#define CAN_SIGNAL_DATA_PACKET_2 0x1D

// Misc
#define MOTOR_CURRENT 3000 // current ratings for stepper motors

/* The (x, y, z) coordinates of the gripper.

 These coordinates are passed to the IK algorithms which return angles for the
 joints of the arm, [swivel], [extend], and [lift].
*/
double gripperX = 0, gripperY = 0, gripperZ = 0;

StepperMotor swivel = StepperMotor(SWIVEL_CS_PIN, SWIVEL_EN_PIN, SWIVEL_LS_PIN, MOTOR_CURRENT, SWIVEL_MIN, SWIVEL_MAX, SWIVEL_GEARBOX);
StepperMotor extend = StepperMotor(EXTEND_CS_PIN, EXTEND_EN_PIN, EXTEND_LS_PIN, MOTOR_CURRENT, EXTEND_MIN, EXTEND_MAX, EXTEND_GEARBOX);
StepperMotor lift = StepperMotor(LIFT_CS_PIN, LIFT_EN_PIN, LIFT_LS_PIN, MOTOR_CURRENT, LIFT_MIN, LIFT_MAX, LIFT_GEARBOX);

void setup()
{
	pinMode(SWIVEL_CS_PIN, OUTPUT);
	pinMode(EXTEND_CS_PIN, OUTPUT);
	pinMode(LIFT_CS_PIN, OUTPUT);
	digitalWrite(SWIVEL_CS_PIN, HIGH);
	digitalWrite(EXTEND_CS_PIN, HIGH);
	digitalWrite(LIFT_CS_PIN, HIGH);

	swivel.setup();
	// extend.setup();
	lift.setup();
	Serial.begin(9600);
	calibrate();
	Serial.println("Finished stepper motor initialization.");

	BurtCan::setup();
	BurtCan::registerHandler(CAN_SWIVEL_ID, swivelHandler);
	BurtCan::registerHandler(CAN_EXTEND_ID, extendHandler);
	BurtCan::registerHandler(CAN_LIFT_ID, liftHandler);
	BurtCan::registerHandler(CAN_PRECISE_SWIVEL_ID, preciseSwivelHandler);
	BurtCan::registerHandler(CAN_PRECISE_EXTEND_ID, preciseExtendHandler);
	BurtCan::registerHandler(CAN_PRECISE_LIFT_ID, preciseLiftHandler);
	Serial.println("Finished CAN bus initialization.");
	// Serial.println("Skipped CAN bus initialization.");
}

void loop()
{
	// while (!swivel.isFinished() || !extend.isFinished() || !lift.isFinished()) {
	// 	swivel.fixPotentialStall();
	// 	extend.fixPotentialStall();
	// 	lift.fixPotentialStall();
	// }
	BurtCan::update();
	// delay(1000);

	broadcastPackets();

	// Temporary Serial Monitor interface for testing
	String input = Serial.readString();
	parseSerialCommand(input);
	delay(500);
}

void reducedPrecisionFloat(float val, uint8_t arr[])
{
	uint16_t convertedNum = (uint16_t)(val * 100 + 0.5);
	arr[0] = (uint8_t)convertedNum << 8;
	arr[1] = (uint8_t)convertedNum & 0xff;
}

struct split_uint16
{
	uint8_t msb;
	uint8_t lsb;
}

struct split_uint16
splitInt(uint16_t val)
{
	struct split_uint16 split;
	split.msb = val >> 8;
	split.lsb = val & 0xff;
	return split;
}

/**
 * Will send out data packets in accordancec with the "CAN Codes" google doc specification.
 */
void broadcastPackets()
{
	// The angles (represented) as floats must be stored as 2 bytes each to send over CAN. So we multiply the floats by 10000, then round to a 16 bit integer.
	struct split_uint16 swivel_current = splitInt((uint16_t)(swivel.current * 10000 + 0.5));
	struct split_uint16 swivel_target = splitInt((uint16_t)(swivel.angle * 10000 + 0.5));
	struct split_uint16 extend_current = splitInt((uint16_t)(extend.current * 10000 + 0.5));
	struct split_uint16 extend_target = splitInt((uint16_t)(extend.angle * 10000 + 0.5));
	struct split_uint16 lift_current = splitInt((uint16_t)(lift.current * 10000 + 0.5));
	struct split_uint16 lift_target = splitInt((uint16_t)(lift.angle * 10000 + 0.5));
	uint8_t data1[8] = {swivel_current.lsb, swivel_current.msb, swivel_target.lsb, swivel_target.msb, extend_current.lsb, extend_current.msb, extend_target.lsb, extend_target.msb};
	BurtCan::send(CAN_SIGNAL_DATA_PACKET_1, data1);

	// The following flags are determined according to the "CAN Codes" google doc.
	uint8_t motor_info = 0b00000000;
	if (!swivel.isFinished)
		motor_info |= 0b00100000;
	if (swivel.readLimitSwitch())
		motor_info |= 0b00010000;
	if (!extend.isFinished)
		motor_info |= 0b00001000;
	if (extend.readLimitSwitch())
		motor_info |= 0b00000100;
	if (!lift.isFinished)
		motor_info |= 0b00000010;
	if (lift.readLimitSwitch())
		motor_info |= 0b00000001;
	uint8_t data2[8] = {lift_current.lsb, lift_current.msb, lift_target.lsb, lift_target.msb, motor_info, 0, 0, 0};
	BurtCan::send(CAN_SIGNAL_DATA_PACKET_2, data2);
}

void updatePosition(double newX, double newY, double newZ)
{
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
	extend.moveTo(newAngles.B);
	lift.moveTo(newAngles.C);
	gripperX = newX;
	gripperY = newY;
	gripperZ = newZ;
}

void calibrate()
{
	swivel.calibrate();
	extend.calibrate();
	lift.calibrate();
	gripperX = CALIBRATED_X;
	gripperY = CALIBRATED_Y;
	gripperZ = CALIBRATED_Z;
}

/* Handler functions for the controller inputs.

 Many messages will send an argument from -1 to 1. However, [CanMessage]s can
 only send unsigned integers. Because of this, we need to convert to float and
 subtract 1, since the message will be from [0, 2].
*/

void swivelHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	gripperX += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void extendHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	gripperY += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void liftHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	gripperZ += ArmConstants::movementSpeed * arg;
	updatePosition(gripperX, gripperY, gripperZ);
}

void preciseSwivelHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	swivel.moveBy(ArmConstants::angleIncrement * arg);
}

void preciseLiftHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	lift.moveBy(ArmConstants::angleIncrement * arg);
}

void preciseExtendHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	extend.moveBy(ArmConstants::angleIncrement * arg);
}

void calibrateHandler(const CanMessage &message)
{
	calibrate();
}

void parseSerialCommand(String input)
{
	Serial.println("User inputted: " + input);
	if (input == "calibrate")
		calibrate();

	int delimiter = input.indexOf(" ");
	if (delimiter == -1)
		return;
	int delimiter2 = input.indexOf(" ", delimiter + 1);
	if (delimiter2 != -1)
	{ // was given an x, y, z command
		float x = input.substring(0, delimiter).toFloat();
		float y = input.substring(delimiter + 1, delimiter2).toFloat();
		float z = input.substring(delimiter2).toFloat();
		Serial.print("Moving to: (");
		Serial.print(x);
		Serial.print(", ");
		Serial.print(y);
		Serial.print(", ");
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

	if (command == "swivel")
	{
		gripperX += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	}
	else if (command == "extend")
	{
		gripperY += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	}
	else if (command == "lift")
	{
		gripperZ += ArmConstants::movementSpeed * arg;
		updatePosition(gripperX, gripperY, gripperZ);
	}
	else if (command == "precise-swivel")
		swivel.moveBy(arg);
	else if (command == "precise-lift")
		lift.moveBy(arg);
	else if (command == "precise-extend")
		extend.moveBy(arg);
	else
		Serial.println("Unrecognized command");
}
