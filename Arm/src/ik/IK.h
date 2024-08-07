// The following line is for Doxygen:
/*! @file 
 * \addtogroup IK 
 * \{
*/

#ifndef burt_arm_ik_h
#define burt_arm_ik_h

#include <Arduino.h>  // <-- includes math.h
#include "constants.h"
#include "../arm.pb.h"

/// Defines the angles of the arm. 
/// 
/// Refer to the diagram in the README for a better view. 
struct Angles { 
	/// The rotation of the base (swivel)
	double theta;

	/// The angle between the base and the humerus (ie, the shoulder joint)
	double B;

	/// The angle between the humerus and the forearm (ie, the elbow joint)
	double C;

	bool isFailure() { return isnan(theta) || isnan(B) || isnan(C); }

	void println() {
		Serial.print("Theta: ");
		Serial.print(theta);
		Serial.print(", B: ");
		Serial.print(B);
		Serial.print(", C: ");
		Serial.println(C);
	}
};

void printCoordinates(Coordinates coordinates);

/// A helper class to handle inverse kinematics calculations for the robot arm. 
/// 
/// The arm functions similarly to a human arm, with the "humerus" (a) attached to 
/// the "body" (chassis) by the "shoulder" joint (measured by angle B). Similarly, 
/// the "forearm" (b) is attached to the "humerus" by the "elbow" joint (measured
/// by angle C). Be sure to check out the diagram titled `ik.png` as well as try 
/// out the interactive demo at https://www.desmos.com/calculator/i8grld5pdu.
/// Those two should and the analogy get you in the right mindset for this library.
/// All code uses variable names corresponding to the diagram.
/// 
/// There are two functions that essentially do forward- and inverse-kinematics.
/// The arm sizes are known, and you call either function depending on what you need. 
/// - #calculatePosition gets the position of the gripper from the joint angles
/// - #calculateAngles gets the angles of the joints for a given point destination
/// 
/// Both methods are static, no need to create an instance. It does, however,
/// mean that it is the caller's responsibility to keep track of the current state:
/// joint angles and position of the gripper (although technically, just the joint 
/// angles are necessary, the position can be calculated by #calculatePosition).
/// 
/// For more details, see: 
/// - The README.md file in this repository, including the diagrams therein
/// - This [interactive Desmos graph](https://www.desmos.com/calculator/i8grld5pdu) that demonstrates the math in action.
/// - This [paper](https://www.researchgate.net/publication/251743615_Triangulation_A_new_algorithm_for_Inverse_Kinematics) that explains a simple algorithm for inverse kinematics
class ArmIK {
	/// The length of the "humerus" (attached to the rover), in millimeters. 
	static constexpr double a = 667.46;

	/// The length of the "forearm" (attached to the gripper), in millimeters. 
	static constexpr double b = 637.504;

	/* Maximum error tolerance. 

	 The IK algorithm will double-check its answer and if the values differ by
	 more than this amount, will return a failure. 
	*/

	/// The maximum error tolerance for the IK algorithm. 
	/// 
	/// The algorithm will double-check its answer and if the values differ by more than
	/// this amount, it will return #failure, which means the math is wrong somewhere.
	static constexpr double tolerance = 0.1;

	public:
		/// The value returned to indicate a failed computation.
		/// 
		/// After every call to #calculateAngles, check if the result is equal to this value.
		/// If it is, ignore the result and warn the operator. 
		static Angles failure;

		/// Computes the current position of the end-effector (hand).
		/// 
		/// Alternatively, think of this function as returning the position of the imaginary 
		/// reticle guiding the arm. #calculateAngles is the opposite of this function, 
		/// which uses the reticle to determine the angles of the physical joints.
		static Coordinates calculatePosition(Angles angles);

		/// Computes the angles of the joints needed to move the gripper to the given point. 
		/// 
		/// Imagine the coordinates as representing the position of an imaginary reticle. No
		/// matter where the gripper is currently positioned, the operator can always try to 
		/// move to a completely different position, denoted by the reticle. This function
		/// bridges the gap between where the arm currently is, and where it wants to be. 
		/// 
		/// The function indicates failure by returning #failure. If this 
		/// happens, it means it was impossible to move to the requested position.
		static Angles calculateAngles(Coordinates coordinates);
};

#endif
// The following close bracket marks the file for Doxygen
/*! \} */
