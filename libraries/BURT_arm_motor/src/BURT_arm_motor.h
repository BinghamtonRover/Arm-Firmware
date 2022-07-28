/*! @file 
 * \addtogroup Motor 
 * \{
*/

#ifndef burt_arm_motor_h
#define burt_arm_motor_h

#include <Arduino.h>
#include <BURT_arm_constants.h>
#include <TMCStepper.h>

#define STALL_CHECK_INTERVAL 100  // ms
#define MOTOR_SPEED 5
#define MOTOR_ACCELERATION 1

class Motor { 
	private: 
		byte chipSelectPin, enablePin;
		short current;
		float minLimit, maxLimit, gearboxRatio, angle, nextStallCheck;
		TMC5160Stepper driver;

		int radToSteps(float radians);
		bool didStall();

	public: 
		Motor(byte chipSelectPin, byte enablePin, int current, float minLimit, float maxLimit, float gearboxRatio) : 
			chipSelectPin(chipSelectPin),
			enablePin(enablePin),
			current(current),
			minLimit(minLimit),
			maxLimit(maxLimit),
			gearboxRatio(gearboxRatio),
			driver(TMC5160Stepper(SPI, chipSelectPin, 0.075)) { }

	public: 
		void setup();  // initializes the motor
		void calibrate();  // calibrates the motor
		bool isFinished();  // checks if the motor has reached its destination
		void fixPotentialStall();  // terminates if a stall is detected
		void writeAngle(float newAngle);  // rotates the motor to a specific angle
		void moveRadians(float radians);  // rotates the motor by a specific angle
		void moveSteps(int steps) {
			driver.XTARGET(steps);
		}
};

#endif
// The following close bracket marks the file for Doxygen
/*! \} */
