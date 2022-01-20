#ifndef ik_h
#define ik_h

#include <math.h>

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
	-   Probably bad structure IDK how to structure stuff with the class stuff.
*/

class IK
{
	double getYPrime(double x, double y);
	void failure();
	void chooseOption();

	//Things to set

	double L2 = 800; //set these values to the lengths L2 and L3
	double L3 = 800;
	double j1Limits[2] = { -100,  100}; //set these values to the limits (in radians and relative to flat of the joints. 
	double j2Limits[2] = { -2.61799, 1.8326 }; //Lower limit first 
	double j3Limits[2] = { -2.35619, 2.44346 };//Upper limit second

	private: 
		double o1Angles[2];
		double o2Angles[2];

	public:
		double x, y, z;
		double oldX, oldY, oldZ;

		void getAngles(double x, double y, double z);
		void doubleCheck();
		void endPointPos();
};

#endif
