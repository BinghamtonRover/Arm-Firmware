#ifndef ik_h
#define ik_h

#include <math.h>

/* This library handles calculations regarding the robotic arm.

 The arm functions similarly to a human arm, with the "humerus" (a) attached to 
 the "body" (chassis) by the "shoulder" joint (measured by angle B). Similarly, 
 the "forearm" (b) is attached to the "humerus" by the "elbow" joint (measured
 by angle C). Be sure to check out the diagram titled `diagram.png` as well as
 try out the interactive demo at https://www.desmos.com/calculator/i8grld5pdu.
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
	static constexpr double a = 10;  // length of the "humerus", attached to the rover
	static constexpr double b = 5;  // length of the "forearm", attached to the humerus

	/* The intervals (in radians) between which the joints can rotate.
	
	 If the IK algorithm calculates an angle outside of these bounds, it will 
	 return a failure.
	*/ 
	static constexpr double bLimits[2] = { -100,  100};
	static constexpr double cLimits[2] = { -2.61799, 1.8326 }; 
	static constexpr double j3Limits[2] = { -2.35619, 2.44346 };

	/* Maximum error tolerance. 

	 The IK algorithm will double-check its answer and if the values differ by
	 more than this amount, will return a failure. 
	*/
	static constexpr double tolerance = 0.1;

	public:
		/* Calculates the current (x, y) position from angles B and C */
		static std::tuple<double, double> calculatePosition(double B, double C);

		/* Calculates the desired angles B and C to arrive at (x, y).

		 The function indicates failure by returning -1 for both values. If this 
		 happens, it means it was impossible to move to the requested position.
		*/
		static std::tuple<double, double> calculateAngles(double x, double y);
};

#endif
