#ifndef burt_arm_motor_h
#define burt_arm_motor_h

#include <BURT_arm_constants.h>
#include <TMC_utils.h>

/* How often, in milliseconds, to check whether the motors have stalled. */
#define STALL_CHECK_INTERVAL 100;

class Motor { 
	public: 
		static int radToSteps(double angle);

		/* Creates an instance of a motor. 

		 Pass in the motor's limits (in radians) here. The [updateAngle] method
		 will be sure to never pass these angles.
	 */
		Motor(int chipSelectPin, int enablePin, int current, const double _limits[2]);

		/* Returns true if the motor has reached its target. */
		bool isFinished();

		/* Calibrates the motor to a known angle. */
		void calibrate();

		/* Updates the angle to the nearest safe value. */
		void safeUpdate(double newAngle);

		/* Periodically checks if the motor has stalled and calibrates if needed. */
		void fixPotentialStall();

		/* The current angle the motor is trying to reach. */
		double angle = 0;

	private: 
		TMC5160Stepper driver;
		double nextStallCheck;
		double limits[2];

		/* Returns true if the motor stalled recently. */
		bool didStall();
};

#endif
