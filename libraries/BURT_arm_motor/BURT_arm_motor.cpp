#include "BURT_arm_motor.h"

double clamp(double x, double upper, double lower) {
	return min(upper, max(x, lower));
}

Motor::Motor(const double _limits[2]) {
	// TODO: initialize motor
	limits[0] = _limits[0]; limits[1] = _limits[1];
	// limits = _limits;
	// this.driver = newDriver(10, 23, 1700, 51200*60, 51200*8);
}

bool Motor::didStall() { return false; /* return driver.status_sg(); */ }
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
}

