/*! @file 
 * \addtogroup Constants 
 * \{
*/

#ifndef burt_arm_constants
#define burt_arm_constants

#ifndef PI
/// The constant PI, to 15 decimal places. 
/// 
/// Fun fact, this is as precise as even NASA gets. See 
/// https://www.jpl.nasa.gov/edu/news/2016/3/16/how-many-decimals-of-pi-do-we-really-need/
#define PI 3.141592653589793
#endif

/// A container class to hold constants used while working with the arm. 
class ArmConstants {
	public: 
		/// The speed at which the arm will move in (x, y, z) space. 
		/// 
		/// Does not necessarily correlate to the angular speed of the joints, since those are 
		/// controlled separately based on #rotationSpeed. Think of this as the speed of the reticle
		/// that controls the arm (and specifically, the gripper). Units unknown. 
		/// 
		/// Multiply by a positive or negative percentage to get a velocity. 
		static constexpr double movementSpeed = 1;

		/// The default amount of radians at which to manually adjust the motors. 
		/// 
		/// This value dictates how "choppy" the motors will feel in precision mode. The smaller the
		/// value, the finer the movements. Units unknown. 
		/// 
		/// Multiply by either 1 or -1 to change the direction of the motion.
		static constexpr double angleIncrement = 1;

		/// The default speed at which to rotate the stepper motors. 
		/// 
		/// This value directly influences how fast the arm feels, as every movement of the stepper
		/// motors is clamped to be smaller than this value. Units unknown. 
		/// 
		/// Multiply by a positive or negative percentage to get a velocity. 
		static constexpr double rotationSpeed = 1;

		/// The maximum difference in joint positions between "frames". Units unknown.
		/// 
		/// CAUTION: Exceeding this value may result in injury or damage to the arm. 
		static constexpr double maxDelta = 10000;

		/* 
		 For some reason, defining them here results in a linking error in the IDE, 
		 so we define them in the .cpp file instead.
		*/
		static double extendLimits[2];               ///< The bounds for the shoulder joint
		static double liftLimits[2];                ///< The bounds for the elbow joint
		static double swivelLimits[2];             ///< The bounds for the base
		static double pinchLimits[2];             ///< The bounds for the gripper's pinch
		static double gripperLiftLimits[2];      ///< The bounds for the gripper's lift
		static double gripperRotateLimits[2];   ///< The bounds for the gripper's rotation
};

#endif

// The following close bracket marks the file for Doxygen
/*! \} */
