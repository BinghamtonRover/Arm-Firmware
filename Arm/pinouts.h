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
	minLimit: 0,
	maxLimit: PI / 2,
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
	minLimit: -3 * PI / 4,
	maxLimit: -PI / 4,
	gearboxRatio: 400,
	speed: SPEED,
	accel: ACCEL,
};

StepperMotor elbow(elbowPins, elbowConfig);
