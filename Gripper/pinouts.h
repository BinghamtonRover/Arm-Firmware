#include "src/tmc/BURT_TMC.h"

#define SPEED 0x000327e7
#define ACCEL 0x00030d4d

// HREI S1: Wrist lift
StepperMotorPins liftPins = {
	chipSelect: 10,
	enable: 35,
	// limitSwitch: 7,
};

StepperMotorConfig liftConfig = {
	name: "lift",
	current: 2000,
	speed: SPEED * 3,
	accel: ACCEL,
	// minLimit: -PI / 2,
	// maxLimit: 0,
  minLimit: -INFINITY,
  maxLimit: INFINITY,
	// limitSwitchDirection: 1,
	gearboxRatio: 100,
};

LimitSwitch liftLimit;

StepperMotor lift(liftPins, liftConfig, liftLimit);

// =============================================

// HREI S2: Wrist rotate
StepperMotorPins rotatePins = {
	chipSelect: 37,
	enable: 34,
	// limitSwitch: -1,
};

StepperMotorConfig rotateConfig = {
	name: "rotate",
	current: 2000,
	speed: SPEED * 3,
	accel: ACCEL,
	minLimit: -INFINITY,
	maxLimit: INFINITY,
	gearboxRatio: 100,
};

LimitSwitch rotateLimit;

StepperMotor rotate(rotatePins, rotateConfig, rotateLimit);

// =============================================

// HREI S3: Wrist pinch
StepperMotorPins pinchPins = {
	chipSelect: 36,
	enable: 40,
	// limitSwitch: -1,  // 9
};

StepperMotorConfig pinchConfig = {
	name: "pinch",
	current: 2000,
	// minLimit: 0,
	// maxLimit: PI / 4,
	speed: SPEED,
	accel: ACCEL,
  minLimit: -INFINITY,
  maxLimit: INFINITY,
	// limitSwitchDirection: 1,
	gearboxRatio: 100,
};

LimitSwitch pinchLimit;

StepperMotor pinch(pinchPins, pinchConfig, pinchLimit);
