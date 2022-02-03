#ifndef burt_arm_motor_h
#define burt_arm_motor_h

// #include 

class Motor { 
	public: 
		/* Creates an instance of a motor. 

		 Pass in the motor's limits (in radians) here. The [updateAngle] method
		 will be sure to never pass these angles.
	 */
		Motor(double limits[2]);

		/* Returns true if the motor stalled recently. */
		bool didStall();

		/* Calibrates the motor to a known angle. */
		void calibrate();

		/* Updates the angle to the nearest safe value. */
		void safeUpdate(double newAngle);

	private: 
		TMC5160Stepper driver;
		double angle = 0;
		double limits[2];
};

#endif
