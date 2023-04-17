#include "src/tmc/BURT_TMC.h"

#define SPEED 0x000327e7
#define ACCEL 0x00030d4d

StepperMotorPins swivelPins = {
	chipSelect: 10,
	enable: 35,
	limitSwitch: 0,
};

StepperMotorConfig swivelConfig = {
	name: "swivel",
	current: 2000, 
	minLimit: -2 * PI,
	maxLimit: 2 * PI,
	stepsPerRotation: 200 * 5 * 256,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor swivel(swivelPins, swivelConfig);

// =============================================

StepperMotorPins shoulderPins = {
	chipSelect: 37,
	enable: 34,
	limitSwitch: 8,
};

StepperMotorConfig shoulderConfig = {
	name: "shoulder",
	current: 2000,
	minLimit: 0,
	maxLimit: PI / 4,
	stepsPerRotation: 200 * 400 * 256,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor shoulder(shoulderPins, shoulderConfig);

// =============================================

StepperMotorPins elbowPins = {
	chipSelect: 36,
	enable: 40,
	limitSwitch: 9,
};

StepperMotorConfig elbowConfig = {
	name: "elbow",
	current: 2000,
	minLimit: -PI / 4,
	maxLimit: PI / 6,
	stepsPerRotation: 200 * 400 * 256,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor elbow(elbowPins, elbowConfig);
