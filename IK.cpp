#include <stdio.h>
#include <tuple>

#include "IK.h"

#define PI 3.141592653589793

std::tuple<double, double, double> IK::calculatePosition(double theta, double B, double C) {
	/* Computes the current position of the end-effector (hand).
	
	 We can do this using basic vector math: Treat all the arms as vectors and 
	 add them together to get the position of the end effector.

	 However, since we're working in the KZ plane, we need to translate k to x 
	 and y. We can do this by using [theta] and trig in the XY plane. 
 */

	// First, get (k, z) by using vectors.
	double absoluteC = B - PI + C;  // gets angle C relative to the X-axis
	double k1 = a * cos(B);
	double z1 = a * sin(B);
	double k2 = b * cos(absoluteC);
	double z2 = b * sin(absoluteC);
	double z = z1 + z2;
	double k = k1 + k2;

	// Now we use k and theta to get x and y.
	double y = k * sin(theta);
	double x = k * cos(theta);
	return {x, y, z};
}

std::tuple<double, double, double> IK::calculateAngles(double x, double y, double z) {
	/* Computes the angles theta, B, and C to move the gripper to (x, y, z).

	 This function uses the Triangulation algorithm. Essentially, we use the Law 
	 of Cosines combined with the lengths of the arms to compute the angles of the
	 joints, working backwards from the (x, y) destination. 

	 Except, we want to work in 3 dimensions, not 2. It's actually pretty simple: 
	 - take a top-down view of the arm (ie, the XY plane)
	 - find the direction (theta) and distance (k) to the target from this view
	 - switch back to a profile view (XZ or YZ plane)
	 - rotate along the Z-axis to make K the horizontal axis (now the KZ plane)
	 - solve for (k, z) using normal triangulation in this new 2D space. 

	 Returns [failure] (angles of -1) if the math doesn't check out ([tolerance]),
	 or the computed angles are out-of-bounds for the rover (like [bLimits]).

	 For more details, see:
	 - the README of this repository
	 - the interactive demo at https://www.desmos.com/calculator/i8grld5pdu
	 - this paper from the inventors of the algorithm: 
	   https://www.researchgate.net/publication/251743615_Triangulation_A_new_algorithm_for_Inverse_Kinematics
	*/

	// Compute the angle and magnitude on the XY plane
	double theta = atan2(y, x);
	double k = sqrt(x*x + y*y);

	// Now we can triangulate with k as our horizontal axis and Z as our vertical.
	// Use the law of cosines to determine the two angles.
	double c = sqrt(k*k + z*z);
	double C  = acos( (a*a + b*b - c*c) / (2*a*b) );
	double B1 = acos( (a*a - b*b + c*c) / (2*a*c) );
	double B2 = atan2(z, k);
	double B  = B1 + B2;

	// Double-check the logic by solving for the supposed position.
	auto [computedX, computedY, computedZ] = calculatePosition(theta, B, C);
	double deltaX = abs(computedX - x);
	double deltaY = abs(computedY - y);
	double deltaZ = abs(computedZ - z);
	if (deltaX > tolerance || deltaY > tolerance || deltaZ > tolerance) 
		return failure;

	// Check for forbidden angles
	if (theta < thetaLimits[0] || theta > thetaLimits[1]) return failure;
	else if (B < bLimits[0] || B > bLimits[1]) return failure;
	else if (C < cLimits[0] || C > cLimits[1]) return failure;

	return {theta, B, C};
}
