#include "src/tmc/BURT_TMC.h"

#define SPEED 0x000327e7
#define ACCEL 0x00030d4d

// HREI S1: Wrist lift
StepperMotorPins liftPins = {
	chipSelect: 10,
	enable: 35,
	limitSwitch: 7,
};

StepperMotorConfig liftConfig = {
	name: "lift",
	current: 2000,
	minLimit: -PI / 2,
	maxLimit: 0,
	gearboxRatio: 20,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor lift(liftPins, liftConfig);

// =============================================

// HREI S2: Wrist rotate
StepperMotorPins rotatePins = {
	chipSelect: 37,
	enable: 34,
	limitSwitch: 0,
};

StepperMotorConfig rotateConfig = {
	name: "rotate",
	current: 500,
	minLimit: -INFINITY,
	maxLimit: INFINITY,
	gearboxRatio: 100,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor rotate(rotatePins, rotateConfig);

// =============================================

// HREI S3: Wrist pinch
StepperMotorPins pinchPins = {
	chipSelect: 36,
	enable: 40,
	limitSwitch: 9,
};

StepperMotorConfig pinchConfig = {
	name: "pinch",
	current: 500,
	minLimit: 0,
	maxLimit: PI / 4,
	gearboxRatio: 100,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor pinch(pinchPins, pinchConfig);
