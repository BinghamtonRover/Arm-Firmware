#ifndef burt_arm_motor_h
#define burt_arm_motor_h

#include <BURT_arm_constants.h>
#include <TMC_utils.h>

class Motor { 
	public: 
		/* Creates an instance of a motor. 

		 Pass in the motor's limits (in radians) here. The [updateAngle] method
		 will be sure to never pass these angles.
	 */
		Motor(const double _limits[2]);

		/* Returns true if the motor stalled recently. */
		bool didStall();

		/* Calibrates the motor to a known angle. */
		void calibrate();

		/* Updates the angle to the nearest safe value. */
		void safeUpdate(double newAngle);

	private: 
		// TMC5160Stepper driver = NULL;
		double angle = 0;
		double limits[2] = {-1, -1};
};

#endif
