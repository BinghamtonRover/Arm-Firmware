/*! @file 
 * \addtogroup Motor 
 * \{
*/

#include "BURT_arm_motor.h"

int StepperMotor::radToSteps(float newAngle) { 
	float steps = newAngle * stepsPer180/PI;
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
	pinMode(limitSwitchPin,INPUT_PULLUP);
	digitalWrite(chipSelectPin, HIGH);
	digitalWrite(enablePin, LOW);

	Serial.print("Initializing pins: ");
	Serial.print(enablePin);
	Serial.print(" and ");
	Serial.println(chipSelectPin);
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

Serial.print("Speed/accel: ");
Serial.print(speed);
Serial.print(" m/s, ");
Serial.print(accel);
Serial.println("m/s^2");
driver.GSTAT(7);
	driver.rms_current(current);
	driver.tbl(2);
	driver.toff(9);
	driver.pwm_freq(1);
	driver.a1(accel);
	driver.v1(speed);
	driver.AMAX(accel);
	driver.VMAX(speed);
	driver.DMAX(accel);
	driver.d1(accel);
	driver.vstop(100);
	driver.vstart(100);
	driver.RAMPMODE(0);

	nextStallCheck = millis() + STALL_CHECK_INTERVAL;
}

void StepperMotor::calibrate() { 
	if (limitSwitchPin != 0) {
		while(!readLimitSwitch()) moveBy(-PI/100);
		driver.XACTUAL(radToSteps(minLimit));
	}
	angle = minLimit;
	currentSteps = radToSteps(minLimit);
	Serial.println("StepperMotor calibrated.");
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
/// The lower and upper bounds are already determined by #minLimit and #maxLimit. Additionally, we 
/// have to account for ArmConstants::maxDelta. To do so, we respect the bounds set by #minLimit 
/// and #maxLimit, but are more conservative when exceeding ArmConstants::maxDelta.
void StepperMotor::moveTo(float newAngle) {
	// TODO: Logic to constrain by maxDelta, currently broken?
	// double lowerBound = max(minLimit, angle - ArmConstants::maxDelta);
	// double upperBound = min(maxLimit, angle + ArmConstants::maxDelta);
	// angle = constrain(newAngle, lowerBound, upperBound);
	Serial.print("[moveTo: " + name + "] Moving from ");
	Serial.print(angle);
	Serial.print(" to ");
	Serial.println(newAngle);	
	if (newAngle > maxLimit || newAngle < minLimit) { 
		Serial.print("  ERROR: Out of bounds (valid inputs are ");
		Serial.print(minLimit);
		Serial.print(" to ");
		Serial.print(maxLimit);
		Serial.println(").");
		return; 
	}
	angle = newAngle;
	currentSteps = radToSteps(angle);
	Serial.print("  Moving to: ");
	Serial.print(currentSteps);
	Serial.print(" steps (range is [-100,000 100,000]) -- ");
	Serial.println(minLimit);
	if (IS_CONNECTED) driver.XTARGET(currentSteps);
	else Serial.println("Motor not connected, not moving");
}

void StepperMotor::moveBy(float radians) {
	Serial.print("[moveBy] ");
	Serial.print(name);
	Serial.print(" is currently at: ");
	Serial.println(angle);
	moveTo(angle + radians);
}

void StepperMotor::debugMoveToStep(int destination) {
	Serial.print("Moving ");
	Serial.print(name);
	Serial.print(" from ");
	Serial.print(currentSteps);
	Serial.print(" to ");
	Serial.print(destination);
	Serial.println(" steps");
	driver.XTARGET(destination);
	currentSteps = destination;
}

bool StepperMotor::readLimitSwitch() {
	return limitSwitchPin != 0 && digitalRead(limitSwitchPin) == LOW;
}

// The following close bracket marks the file for Doxygen
/*! \} */
