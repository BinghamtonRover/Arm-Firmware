#include <stdio.h>
#include <tuple>

#include "IK.h"

#define PI 3.141592653589793

std::tuple<double, double> IK::calculatePosition(double B, double C) {
	/* Computes the current position of the end-effector (hand).
	
	 We can do this using basic vector math: Treat all the arms as vectors and 
	 add them together to get the position of the end effector.
 */

	double absoluteC = B - PI + C;  // gets angle C relative to the X-axis
	double x1 = a * cos(B);
	double y1 = a * sin(B);
	double x2 = b * cos(absoluteC);
	double y2 = b * sin(absoluteC);
	return {x1 + x2, y1 + y2};
}

std::tuple<double, double> IK::calculateAngles(double x, double y) {
	/* Computes the values for angles B and C to move the gripper to (x, y).

	 This function uses the Triangulation algorithm. Essentially, we use the Law 
	 of Cosines combined with the lengths of the arms to compute the angles of the
	 joints, working backwards from the (x, y) destination. 

	 Returns a failure (angles of -1) if the math doesn't check out ([tolerance]),
	 or the computed angles are out-of-bounds for the rover (like [bLimits]).

	 For more details, see:
	 - the diagram labeled `ik.png` in this repository
	 - the interactive demo at https://www.desmos.com/calculator/i8grld5pdu
	 - this paper from the inventors of the algorithm: 
	   https://www.researchgate.net/publication/251743615_Triangulation_A_new_algorithm_for_Inverse_Kinematics
	*/

	// Use the law of cosines to determine the two angles.
	double c = sqrt(y*y + x*x);
	double C  = acos( (a*a + b*b - c*c) / (2*a*b) );
	double B1 = acos( (a*a - b*b + c*c) / (2*a*c) );
	double B2 = atan(y / x);
	double B  = B1 + B2;

	// Double-check the logic by solving for the supposed position.
	auto [computedX, computedY] = calculatePosition(B, C);
	double deltaX = abs(computedX - x);
	double deltaY = abs(computedY - y);
	if (deltaX > tolerance || deltaY > tolerance) return {-1, -1};

	// Check for forbidden angles
	if (B < bLimits[0] || B > bLimits[1]) return {-1, -1};
	else if (C < cLimits[0] || C > cLimits[1]) return {-1, -1};

	return {B, C};
}
