#ifndef burt_arm_constants
#define burt_arm_constants

#ifndef PI
#define PI 3.141592653589793
#endif

class BurtArmConstants {
	public: 
		/* The speed at which the arm will move in (x, y, z) space. 

		 Does not necessarily correlate to the angular speed of the joints, since those
		 are controlled separately by an IK algorithm based on these points. 

		 Multiply by a factor between -1 and 1 to obtain a new speed and direction.
		*/
		static constexpr double movementSpeed = 1;

		/* The default amount of radians at which to manually adjust the motors. 

		 Multiply by either 1 or -1 to change the direction of the motion.
		*/
		static constexpr double angleIncrement = 1;

		/* The default speed at which to rotate motors. 
		 
		 Multiply by a factor between -1 and 1 to obtain a new speed and direction.
		*/
		// static constexpr double rotationSpeed = 1;

		/* The maximum difference in joint positions between "frames". */
		static constexpr double maxDelta = 1;

	public:
		/* The intervals (in radians) between which the joints can rotate. */
		static constexpr double extendLimits[2] = { -100,  100};
		static constexpr double liftLimits[2] = { -2.61799, 1.8326 }; 
		static constexpr double swivelLimits[2] = { -2.35619, 2.44346 };
		// static constexpr double pinchLimits[2] = {0, 1};
		// static constexpr double gripperLiftLimits[2] = {-PI, PI};
		// static constexpr double gripperRotateLimits[2] = {0, 2 * PI};
};

#endif
