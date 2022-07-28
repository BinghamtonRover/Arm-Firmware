/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_arm_motor.h"

float clamp(float x, float upper, float lower) {
	Serial.print("Clamping between ");
	Serial.print(upper);
	Serial.print(" and ");
	Serial.print(lower);	
	if (x > upper) return upper;
	else if (x < lower) return lower;
	else return x;
	// return min(upper, max(x, lower));
}

int Motor::radToSteps(float newAngle) { 
	Serial.print("Rotating to angle: ");
	Serial.println(newAngle);
	float steps = newAngle * 51200/(2 * PI) * gearboxRatio;
	Serial.print("Rotating to steps: ");
	Serial.println(steps);
	return steps;
}

bool Motor::didStall() { return driver.status_sg(); }

void Motor::setup() {
	// TODO: Decide if this is needed: 
	// -->
	pinMode(enablePin, OUTPUT);
	pinMode(chipSelectPin, OUTPUT);
	digitalWrite(chipSelectPin, HIGH);
	digitalWrite(enablePin, LOW);
	// <--
	driver.begin();
	driver.reset();
	TMC5160Stepper::IOIN_t ioin{ driver.IOIN() };
	if (ioin.version == 0xFF || ioin.version == 0) {
    Serial.print("Driver communication error with CS pin: ");
    Serial.println(chipSelectPin);
    while(true);
	} else if (ioin.sd_mode) {
    Serial.println("Driver is hardware configured for Step & Dir mode, CS pin is ");
    Serial.println(chipSelectPin);
    while(true);
	} else if (ioin.drv_enn) {
    Serial.println("Driver is not hardware enabled");
    Serial.println(chipSelectPin);
    while(true);
	}

	digitalWrite(enablePin, HIGH);  // disable driver to clear the cache
	delay(1000);
	digitalWrite(enablePin, LOW);   // re-enable drive, to start loading in parameters

	int maxVelocity = 51200 * MOTOR_SPEED;
	int accel = 51200 * MOTOR_ACCELERATION;
	driver.GSTAT(7);
	driver.rms_current(current);
	driver.tbl(2);
	driver.toff(9);
	driver.pwm_freq(1);
	driver.microsteps(256);
	driver.a1(accel);
	driver.v1(maxVelocity);
	driver.AMAX(accel);
	driver.VMAX(maxVelocity);
	driver.DMAX(accel);
	driver.d1(accel);
	driver.vstop(100);
	driver.vstart(100);
	driver.RAMPMODE(0);

	nextStallCheck = millis() + STALL_CHECK_INTERVAL;
}

// TODO:
void Motor::calibrate() { 
	driver.XACTUAL(radToSteps(maxLimit));
	angle = maxLimit;
	Serial.println("Hm, try now");
}

bool Motor::isFinished() { return driver.XTARGET() == driver.XACTUAL(); }

void Motor::fixPotentialStall() {
	double currentTime = millis();
	if (currentTime < nextStallCheck) return;
	nextStallCheck = currentTime + STALL_CHECK_INTERVAL;
	// if (didStall()) calibrate();
	if (didStall()) {  // for SAR testing
		Serial.println("Motor stalled. Please restart");
		while (true);
	}
}

void Motor::writeAngle(float newAngle) {
	/* Bounds the angle we are updating to.

	 The lower and upper bounds are already determined by [limits]. Additionally,
	 we have to account for [BurtArmConstants::maxDelta]. To do so, we respect the
	 bounds set by [limits], but are more conservative when exceeding [maxDelta].
	*/
	// double lowerBound = max(0, angle - BurtArmConstants::maxDelta);
	// double upperBound = min(limit, angle + BurtArmConstants::maxDelta);
	// angle = clamp(newAngle, lowerBound, upperBound);
	Serial.print("Current angle: ");
	Serial.println(angle);
	Serial.print("Trying to get to: ");
	Serial.println(newAngle);	
	angle = clamp(newAngle, maxLimit, minLimit);
	Serial.print("Clamped to: ");
	Serial.println(angle);
	driver.XTARGET(radToSteps(angle));
}

void Motor::moveRadians(float radians) {
	Serial.print("Currently at: ");
	Serial.println(angle);
	writeAngle(angle + radians);
}

// The following close bracket marks the file for Doxygen
/*! \} */
