#include "src/tmc/BURT_TMC.h"

StepperMotorPins swivelPins = {
	enable: 35,
	chipSelect: 10,
};

StepperMotorConfig swivelConfig = {
	name: "swivel",
	current: 2000,
	speed: 200'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 47,
};

StepperMotor swivel(swivelPins, swivelConfig);

// =============================================

StepperMotorPins shoulderPins = {
	enable: 34,
	chipSelect: 37,
};

StepperMotorConfig shoulderConfig = {
	name: "shoulder",
	current: 2000,
	speed: 200'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 400 * -1,
};

LimitSwitch shoulderLimit = {
	pin: -1,  // 8
	triggeredValue: HIGH,
	direction: -1,
	position: PI / 2,
};

// TODO: re-enable limit switch
StepperMotor shoulder(shoulderPins, shoulderConfig, shoulderLimit);

// =============================================

StepperMotorPins elbowPins = {
	enable: 40,
	chipSelect: 36,
};

StepperMotorConfig elbowConfig = {
	name: "elbow",
	current: 2000,
	speed: 200'000,
	acceleration: 200'000,
	stepsPerUnit: microstepsPerRadian * 400 * -1,
};

LimitSwitch elbowLimit = {
	pin: -1,  // 9
	direction: -1,
	position: PI / 2,
};

// TODO: re-enable limit switch
StepperMotor elbow(elbowPins, elbowConfig, elbowLimit);
