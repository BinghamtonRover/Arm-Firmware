#include "BURT_arm_motor.h"

double clamp(double x, double upper, double lower) {
	return min(upper, max(x, lower));
}

// TODO: 
int Motor::radToSteps(double angle) { return angle; }

Motor::Motor(int chipSelectPin, int enablePin, int current, const double _limits[2]) : 
	driver{newDriver(chipSelectPin, enablePin, current, 51200*60, 51200*8)} 
{
	// The arrays are two different types: const double* and double[2]. 
	// So we use this workaround to copy the data instead.
	limits[0] = _limits[0]; limits[1] = _limits[1];
	nextStallCheck = millis() + STALL_CHECK_INTERVAL;
}

bool Motor::didStall() { return driver.status_sg(); }

bool Motor::isFinished() { return driver.XTARGET() == driver.XACTUAL(); }

// TODO:
void Motor::calibrate() { Serial.println("Hm, try now"); }

void Motor::safeUpdate(double newAngle) {
	/* Bounds the angle we are updating to.

	 The lower and upper bounds are already determined by [limits]. Additionally,
	 we have to account for [BurtArmConstants::maxDelta]. To do so, we respect the
	 bounds set by [limits], but are more conservative when exceeding [maxDelta].
	*/
	double lowerBound = max(limits[0], angle - BurtArmConstants::maxDelta);
	double upperBound = min(limits[1], angle + BurtArmConstants::maxDelta);
	angle = clamp(newAngle, lowerBound, upperBound);
	driver.XTARGET(radToSteps(angle));
}

void Motor::fixPotentialStall() {
	double currentTime = millis();
	if (currentTime < nextStallCheck) return;
	nextStallCheck = currentTime + STALL_CHECK_INTERVAL;
	if (didStall()) calibrate();
}

