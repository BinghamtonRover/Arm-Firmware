#include "src/tmc/BURT_TMC.h"

#define SPEED 0x000327e7
#define ACCEL 0x00030d4d

StepperMotorPins swivelPins = {
	chipSelect: 10,
	enable: 35,
	limitSwitch: -1,
};

StepperMotorConfig swivelConfig = {
	name: "swivel",
	current: 2000,
	limitSwitchPosition: 0,
	minLimit: -INFINITY,  // 0,
	maxLimit: INFINITY,  // 2*PI,
	isPositive: true,
	limitSwitchDirection: -1,
	gearboxRatio: 5,
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
	limitSwitchPosition: 1.71042,
	minLimit: -INFINITY, // 0,
	maxLimit: INFINITY,  // 1.71042,
	isPositive: false,
	limitSwitchDirection: -1,
	gearboxRatio: 400,
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
	limitSwitchPosition: PI / 4,
	minLimit: -INFINITY, // PI / 4,
	maxLimit: INFINITY,  // PI,
	isPositive: true,
	limitSwitchDirection: -1,
	gearboxRatio: 400,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor elbow(elbowPins, elbowConfig);
