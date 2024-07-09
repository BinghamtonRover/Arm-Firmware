#include "src/tmc/BURT_TMC.h"

// HREI S1: Wrist lift
StepperMotorPins liftPins = {
	enable: 35,
	chipSelect: 10,
};

StepperMotorConfig liftConfig = {
	name: "lift",
	current: 2000,
	speed: 600'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 100,
};

StepperMotor lift(liftPins, liftConfig);

// =============================================

// HREI S2: Wrist rotate
StepperMotorPins rotatePins = {
	enable: 34,
	chipSelect: 37,
};

StepperMotorConfig rotateConfig = {
	name: "rotate",
	current: 2000,
	speed: 600'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 100,
};

StepperMotor rotate(rotatePins, rotateConfig);

// =============================================

// HREI S3: Wrist pinch
StepperMotorPins pinchPins = {
	enable: 40,
	chipSelect: 36,
};

StepperMotorConfig pinchConfig = {
	name: "pinch",
	current: 2000,
	speed: 200'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 100,
};

StepperMotor pinch(pinchPins, pinchConfig);
