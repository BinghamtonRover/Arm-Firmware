/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_arm_motor.h"

int StepperMotor::radToSteps(float newAngle) { 
	Serial.print("Rotating to angle: ");
	Serial.println(newAngle);
	float steps = newAngle * 51200/(2 * PI) * gearboxRatio;
	Serial.print("Rotating to steps: ");
	Serial.println(steps);
	return steps;
}

bool StepperMotor::didStall() { return driver.status_sg(); }

// TODO: Investigate parameters used here. 
// See https://github.com/BinghamtonRover/arm-firmware/issues/6
void StepperMotor::setup() {
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

// TODO: Make this calibrate. 
// See https://github.com/BinghamtonRover/arm-firmware/issues/7
void StepperMotor::calibrate() { 
	driver.XACTUAL(radToSteps(maxLimit));
	angle = maxLimit;
	Serial.println("Hm, try now");
}

bool StepperMotor::isFinished() { return driver.XTARGET() == driver.XACTUAL(); }

// TODO: Decide what can be done about a stall. 
void StepperMotor::fixPotentialStall() {
	double currentTime = millis();
	if (currentTime < nextStallCheck) return;
	nextStallCheck = currentTime + STALL_CHECK_INTERVAL;
	if (didStall()) {
		Serial.println("StepperMotor stalled. Please restart");
		while (true);
	}
}

/// Bounds the angle automatically.
/// 
/// The lower and upper bounds are already determined by #limits. Additionally, we have to account
/// for BurtArmConstants::maxDelta. To do so, we respect the bounds set by #limits, but are more
/// conservative when exceeding #maxDelta.
void StepperMotor::moveTo(float newAngle) {
	double lowerBound = max(0, angle - BurtArmConstants::maxDelta);
	double upperBound = min(limit, angle + BurtArmConstants::maxDelta);
	angle = constrain(newAngle, lowerBound, upperBound);
	Serial.print("Current angle: ");
	Serial.println(angle);
	Serial.print("Trying to get to: ");
	Serial.println(newAngle);	
	angle = constrain(newAngle, maxLimit, minLimit);
	Serial.print("Clamped to: ");
	Serial.println(angle);
	driver.XTARGET(radToSteps(angle));
}

void StepperMotor::moveBy(float radians) {
	Serial.print("Currently at: ");
	Serial.println(angle);
	moveTo(angle + radians);
}

void StepperMotor::debugMoveSteps(int steps) {
	driver.XTARGET(steps);
}

// The following close bracket marks the file for Doxygen
/*! \} */
