#include "BURT_arm_motor.h"

void Motor::Motor(double limits[2]) {
	// TODO: initialize motor
	this.limits = limits;
}

bool Motor::didStall() { return driver.status_sg(); }
void Motor::calibrate() { Serial.println("Hm, try now"); }

void Motor::safeUpdate(double newAngle) {
	/* Bounds the angle we are updating to.

	 The lower and upper bounds are already determined by [limits]. Additionally,
	 we have to account for [BurtConstants::maxDelta]. To do this, we respect the 
	 bounds set by [limits], but are more conservative when exceeding [maxDelta].
	*/
	double lowerBound = max(limits[0], angle - BurtConstants::maxDelta);
	double upperBound = min(limits[1], angle + BurtConstants::maxDelta);
	angle = clamp(newAngle, lowerBound, upperBound);
}

