#ifndef ik_h
#define ik_h

#include <math.h>

/* This library handles calculations regarding the robotic arm.

 The arm functions similarly to a human arm, with the "humerus" (a) attached to 
 the "body" (chassis) by the "shoulder" joint (measured by angle B). Similarly, 
 the "forearm" (b) is attached to the "humerus" by the "elbow" joint (measured
 by angle C). Be sure to check out the diagram titled `ik.png` as well as try 
 out the interactive demo at https://www.desmos.com/calculator/i8grld5pdu.
 Those two should and the analogy get you in the right mindset for this library.
 All code uses variable names corresponding to the diagram.

 There are two functions that essentially do forward- and inverse-kinematics.
 The arm sizes are known, and you call either function depending on what you need. 
 - [calculatePosition] gets the position of the gripper from the joint angles
 - [calculateAngles] gets the angles of the joints for a given point destination

 Both methods are static, no need to create an instance of [IK]. It does, however,
 mean that it is the caller's responsibility to keep track of the current state:
 joint angles and position of the gripper (although technically, just the joint 
 angles are necessary, the position can be calculated by [calculatePosition]).
*/

class IK {
	// Length of the arms in any units, so long as they're consistent. 
	// TODO: Update these
	static constexpr double a = 10;  // length of the "humerus", attached to the rover
	static constexpr double b = 5;  // length of the "forearm", attached to the humerus

	/* The intervals (in radians) between which the joints can rotate.
	
	 If the IK algorithm calculates an angle outside of these bounds, it will 
	 return a failure.
	*/ 
	// TODO: Update these
	static constexpr double bLimits[2] = { -100,  100};
	static constexpr double cLimits[2] = { -2.61799, 1.8326 }; 
	static constexpr double thetaLimits[2] = { -2.35619, 2.44346 };

	/* Maximum error tolerance. 

	 The IK algorithm will double-check its answer and if the values differ by
	 more than this amount, will return a failure. 
	*/
	static constexpr double tolerance = 0.1;

	/* The value returned to indicate a failed computation. */
	static constexpr std::tuple<double, double, double> failure = {-1, -1, -1};

	public:
		/* Computes the current position of the end-effector (hand). */
		static std::tuple<double, double, double> calculatePosition(double theta, double B, double C);

		/* Computes the angles theta, B, and C to move the gripper to (x, y, z).

		 The function indicates failure by returning [failure]. If this 
		 happens, it means it was impossible to move to the requested position.
		*/
		static std::tuple<double, double, double> calculateAngles(double x, double y, double z);
};

#endif
