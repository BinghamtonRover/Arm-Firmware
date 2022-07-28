/*! @file 
 * \addtogroup Constants 
 * \{
*/

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
		static constexpr double rotationSpeed = 1;

		/* The maximum difference in joint positions between "frames". */
		static constexpr double maxDelta = 10000;

		/* The intervals (in radians) between which the joints can rotate. 

		 For some reason, defining them here results in a linking error in the IDE, 
		 so we define them in the .cpp file instead.
		*/
		static double extendLimits[2];
		static double liftLimits[2];
		static double swivelLimits[2];
		static double pinchLimits[2];
		static double gripperLiftLimits[2];
		static double gripperRotateLimits[2];
};

#endif

// The following close bracket marks the file for Doxygen
/*! \} */
