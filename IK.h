#ifndef ik_h
#define ik_h

#include <cmath>
#include <iostream>

/* Usage:
	To be honest I'm not 100% sure how the integration with arduino works, but the getAngles function you put in your desired x,y,z
	coordinates and the current angle measures (in radians for now) and changes the newAngles array to what the joint angles relative
	to straight should be (in radians for now).
	Just make sure to set everything in the things to set section to fit the arm.
*/

/* Problems still to fix:
	-   J1 is assumed to not have any limits (as of right now we don't know the limits of J1 because they come from the other
		stuff on the rover) might even make the whole limiting more complicated because J1 limits will likely be dependent on
		positions of J2 and J3.
	-	atan2 will always return angles between -pi/2 and pi/2, but the joints have more freedom than that. We need to be able to
		tell which quadrant the point we are trying to get to is in and do the transformation to get there or something like that
	-   Probably bad structure IDK how to structure stuff with the class stuff.
	-	Not super sure I'm returning the angles the right way, Idrk how the interfacing with arduino works
*/

class IK
{
	double getYPrime(double x, double y);
	double getIntersectionY1(double a, double b, double c, double d);
	double getIntersectionY2(double a, double b, double c, double d);
	double getIntersectionX1(double a, double b, double c, double d);
	double getIntersectionX2(double a, double b, double c, double d);
	double getAngle3(double y1, double z1, double y, double z);
	void chooseOption(double currentAngles[]);
	void failure(double currentAngles[]);

	//Things to set

	double L2 = 800; //set these values to the lengths L2 and L3
	double L3 = 800;
	double j1Limits[2] = { -100,  100}; //set these values to the limits (in radians and relative to flat of the joints. 
	double j2Limits[2] = { -2.61799, 1.8326 }; //Lower limit first 
	double j3Limits[2] = { -2.35619, 2.44346 };//Upper limit second

	public:
		double newAngles[3];
		double o1Angles[2];
		double o2Angles[2];
		double Z1;
		double Z2;
		double Y1;
		double Y2;
		double yPrime;
		double finalX1;
		double finalY1;
		double finalZ1;
		double finalX2;
		double finalY2;
		double finalZ2;
		double endPointX;
		double endPointY;
		double endPointZ;
		void getAngles(double x, double y, double z, double currentAngles[]);
		void doubleCheck();
		void endPointPos(double currentAngles[]);

};

#endif
