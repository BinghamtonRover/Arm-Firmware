// The following line is for Doxygen:
/*! @file 
 * \addtogroup IK 
 * \{
*/

#include "IK.h"

Angles ArmIK::failure = Angles(0, 0, 0);

void printCoordinates(Coordinates coordinates) {
	Serial.print("(");
	Serial.print(coordinates.x);
	Serial.print(", ");	
	Serial.print(coordinates.y);
	Serial.print(", ");	
	Serial.print(coordinates.z);
	Serial.println(").");
}

/// Computes the current position of the end-effector (hand).
/// 
/// We can do this using basic vector math: Treat all the arms as vectors and add them together 
/// to get the position of the end effector. However, since we're working in the KZ plane, we 
/// need to translate k to x and y. We can do this by using [theta] and trig in the XY plane. 
Coordinates ArmIK::calculatePosition(Angles angles) {
	// First, get (k, z) by using vectors.
	float absoluteC = angles.B - PI + angles.C;  // gets angle C relative to the X-axis
	float k1 = a * cos(angles.B);
	float z1 = a * sin(angles.B);
	float k2 = b * cos(absoluteC);
	float z2 = b * sin(absoluteC);
	float z = z1 + z2;
	float k = k1 + k2;

	// Now we use k and theta to get x and y.
	float y = k * sin(angles.theta);
	float x = k * cos(angles.theta);

	// Pack and return the values.
	Coordinates result = {x: x, y: y, z: z};
	return result; 
}

/// Computes the angles theta, B, and C to move the gripper to (x, y, z).
/// 
/// This function uses the Triangulation algorithm. Essentially, we use the Law of Cosines combined
/// with the lengths of the arms to compute the angles of the joints, working backwards from the 
/// (x, y) destination.
/// 
/// Except, we want to work in 3 dimensions, not 2. It's actually a pretty simple adjustment: 
/// - take a top-down view of the arm (ie, the XY plane)
/// - find the direction (theta) and distance (k) to the target from this view
/// - switch back to a profile view (XZ or YZ plane)
/// - rotate along the Z-axis to make K the horizontal axis (now the KZ plane)
/// - solve for (k, z) using normal triangulation in this new 2D space. 
/// 
/// Be sure to thoroughly read the README for an intuitive understanding of the process.
/// 
/// Returns #failure if the math doesn't check out (see #tolerance).
Angles ArmIK::calculateAngles(Coordinates coordinates) {
	// Compute the angle and magnitude on the XY plane
	double x = coordinates.x, y = coordinates.y, z = coordinates.z;
	double theta = atan2(y, x);
	double k = sqrt(x*x + y*y);

	// Now we can triangulate with k as our horizontal axis and Z as our vertical.
	// Use the law of cosines to determine the two angles.
	double c = sqrt(k*k + z*z);
	double C  = acos( (a*a + b*b - c*c) / (2*a*b) );
	double b1 = acos( (a*a - b*b + c*c) / (2*a*c) );
	double b2 = atan2(z, k);
	double B  = b1 + b2;

	// Double-check the logic by solving for the supposed position.
	Angles angles(theta, B, C);
	Coordinates position = calculatePosition(angles);
	double deltaX = fabs(position.x - x);
	double deltaY = fabs(position.y - y);
	double deltaZ = fabs(position.z - z);
	if (deltaX > tolerance || deltaY > tolerance || deltaZ > tolerance) 
		return ArmIK::failure;

	// Pack and return the values.
	Angles result(theta, B, C);
	return result;
}

// The following close bracket marks the file for Doxygen
/*! \} */
