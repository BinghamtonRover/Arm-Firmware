#include "src/tmc/BURT_TMC.h"

#define SPEED 0x000327e7
#define ACCEL 0x00030d4d

StepperMotorPins swivelPins = {
	chipSelect: 10,
	enable: 35,
};

StepperMotorConfig swivelConfig = {
	name: "swivel",
	current: 2000,
	speed: SPEED,
	accel: ACCEL,
	minLimit: -INFINITY,  // 0,
	maxLimit: INFINITY,  // 2*PI,
	isPositive: true,
	gearboxRatio: 47,
};

LimitSwitch swivelLimit;

StepperMotor swivel(swivelPins, swivelConfig, swivelLimit);

// =============================================

StepperMotorPins shoulderPins = {
	chipSelect: 37,
	enable: 34,
};

StepperMotorConfig shoulderConfig = {
	name: "shoulder",
	current: 2000,
	speed: SPEED,
	accel: ACCEL,
	minLimit: -INFINITY, // 0,
	maxLimit: INFINITY,  // 1.71042,
	isPositive: false,
	gearboxRatio: 400,
};

LimitSwitch shoulderLimit = {
	pin: -1,  // 8
	triggeredValue: HIGH,
	direction: -1,
	position: 0,
};

StepperMotor shoulder(shoulderPins, shoulderConfig, shoulderLimit);

// =============================================

StepperMotorPins elbowPins = {
	chipSelect: 36,
	enable: 40,
};

StepperMotorConfig elbowConfig = {
	name: "elbow",
	current: 2000,
	speed: SPEED,
	accel: ACCEL,
	minLimit: -INFINITY, // PI / 4,
	maxLimit: INFINITY,  // PI,
	isPositive: true,
	gearboxRatio: 400,
};

LimitSwitch elbowLimit = {
	pin: -1,  // 9
	direction: -1,
	position: PI / 4,
};

StepperMotor elbow(elbowPins, elbowConfig, elbowLimit);
