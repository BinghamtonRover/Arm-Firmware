#include <BURT_arm_constants.h>
#include <BURT_arm_motor.h>
#include <BURT_can.h>

// Pinouts
#define ROTATE_EN_PIN 35
#define ROTATE_CS_PIN 10
#define ROTATE_LS_PIN 7

#define LIFT_EN_PIN 34
#define LIFT_CS_PIN 37
#define LIFT_LS_PIN 8

#define PINCH_EN_PIN 40
#define PINCH_CS_PIN 36
#define PINCH_LS_PIN 9

#define ROTATE_MIN -2 * PI
#define ROTATE_MAX 2 * PI

#define LIFT_MIN 0
#define LIFT_MAX 2.004

#define PINCH_MIN 0
#define PINCH_MAX 1.867

// Limits for the joints, in terms of radians from their limit switches
#define ROTATE_LIMIT INFINITY // can swivel in endless circles
#define PINCH_LIMIT 1.867
#define LIFT_LIMIT 2.004

// CAN bus IDs
#define CAN_ROTATE_ID 0
#define CAN_PINCH_ID 0
#define CAN_LIFT_ID 0
#define CAN_PRECISE_PINCH_ID 0
#define CAN_CALIBRATE_ID 0
#define CAN_SEND_PACKET_ID 0

// Sensor Pins
#define TEMP1_PIN 0
#define TEMP2_PIN 0
#define TEMP3_PIN 0
#define GYRO_X_PIN 0
#define GYRO_Y_PIN 0
#define GYRO_Z_PIN 0

// Misc
#define LOW_CURRENT 1300
#define HIGH_CURRENT 2200

StepperMotor rotate = StepperMotor(ROTATE_CS_PIN, ROTATE_EN_PIN, ROTATE_LS_PIN, LOW_CURRENT, ROTATE_MIN, ROTATE_MAX, 99.51);
StepperMotor pinch = StepperMotor(PINCH_CS_PIN, PINCH_EN_PIN, PINCH_LS_PIN, LOW_CURRENT, PINCH_MIN, PINCH_MAX, 199.02);
StepperMotor lift = StepperMotor(LIFT_CS_PIN, LIFT_EN_PIN, LIFT_LS_PIN, HIGH_CURRENT, LIFT_MIN, LIFT_MAX, 29.16);

void setup()
{
	pinMode(TEMP1_PIN, INPUT);
	pinMode(TEMP2_PIN, INPUT);
	pinMode(TEMP3_PIN, INPUT);
	pinMode(GYRO_X_PIN, INPUT);
	pinMode(GYRO_Y_PIN, INPUT);
	pinMode(GYRO_Z_PIN, INPUT);

	Serial.begin(9600);
	Serial.println("Finished stepper motor initialization.");
	BurtCan::setup();
	BurtCan::registerHandler(CAN_ROTATE_ID, rotateHandler);
	BurtCan::registerHandler(CAN_PINCH_ID, pinchHandler);
	BurtCan::registerHandler(CAN_LIFT_ID, liftHandler);
	BurtCan::registerHandler(CAN_PRECISE_PINCH_ID, precisePinchHandler);
	BurtCan::registerHandler(CAN_CALIBRATE_ID, calibrateHandler);
	Serial.println("Finished CAN bus initialization.");
}

void loop()
{
	while (!pinch.isFinished() || !rotate.isFinished() || !lift.isFinished())
	{
		pinch.fixPotentialStall();
		rotate.fixPotentialStall();
		lift.fixPotentialStall();
	}
	BurtCan::update();
	// delay(1000);

	// Temporary Serial Monitor interface for testing
	String input = Serial.readString();
	parseSerialCommand(input);
	delay(500);
}

void calibrate()
{
	pinch.calibrate();
	rotate.calibrate();
	lift.calibrate();
}

void sendCANPacket()
{
	uint8_t TEMP1 = analogRead(TEMP1_PIN);
	uint8_t TEMP2 = analogRead(TEMP2_PIN);
	uint8_t TEMP3 = analogRead(TEMP3_PIN);
	uint8_t GYROX = analogRead(GYRO_X_PIN);
	uint8_t GYROY = analogRead(GYRO_Y_PIN);
	uint8_t GYROZ = analogRead(GYRO_Z_PIN);
	uint8_t data[] = {TEMP1, TEMP2, TEMP3, GYROX, GYROY, GYROZ, 0, 0};
	BurtCan::send(CAN_SEND_PACKET_ID, data);
}

/* Handler functions for the controller inputs.

 Many messages will send an argument from -1 to 1. However, [CanMessage]s can
 only send unsigned integers. Because of this, we need to convert to float and
 subtract 1, since the message will be from [0, 2].
*/

void pinchHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	pinch.moveTo(pinch.angle + (ArmConstants::rotationSpeed * arg));
}

void rotateHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	rotate.moveTo(rotate.angle + (ArmConstants::rotationSpeed * arg));
}

void liftHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	lift.moveTo(lift.angle + (ArmConstants::rotationSpeed * arg));
}

void precisePinchHandler(const CanMessage &message)
{
	float arg = (float)message.buf[0] - 1;
	pinch.moveTo(pinch.angle + (ArmConstants::angleIncrement * arg));
}

void calibrateHandler(const CanMessage &message)
{
	calibrate();
}

void parseSerialCommand(String input)
{
	/*
		pinch: [-1, 1]
		rotate: [-1, 1]
		lift: [-1, 1]
		precisePinch: {-1, 1}
		calibrate;
	*/

	if (input == "calibrate")
		calibrate();

	int delimiter = input.indexOf(" ");
	if (delimiter == -1)
		return;
	String command = input.substring(0, delimiter);
	String part2 = input.substring(delimiter + 1);
	float arg = part2.toFloat();
	if (arg < -1 || arg > 1)
	{
		Serial.println("Argument must be between -1 and 1");
		return;
	}

	if (command == "pinch")
		pinch.moveTo(pinch.angle + arg);
	else if (command == "rotate")
		rotate.moveTo(rotate.angle + arg);
	else if (command == "precise-pinch")
	{
		pinch.moveTo(pinch.angle + (ArmConstants::angleIncrement * arg));
	}
	else
	{
		Serial.println("Unknown command: " + command);
	}
}
