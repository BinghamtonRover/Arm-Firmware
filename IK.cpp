#include "IK.h"

//Check rotation validity first

double IK::getYPrime(double x, double y) {
	return sqrt(x * x + y * y);
}

void IK::failure() {
	x = oldX;
	y = oldY;
	z = oldZ;
}

void IK::chooseOption() {
	if (x >= j1Limits[0] && x <= j1Limits[1]) {
		if (((fabs(y - o1Angles[0])) + (fabs(z - o1Angles[1]))) <= (((fabs(y - o2Angles[0])) + (fabs(z - o2Angles[1]))))) {
			if (o1Angles[0] >= j2Limits[0] && o1Angles[0] <= j2Limits[1] && o1Angles[1] >= j3Limits[0] && o1Angles[1] <= j3Limits[1]) {
				y = o1Angles[0];
				z = o1Angles[1];
			}
			else if (o2Angles[0] >= j2Limits[0] && o2Angles[0] <= j2Limits[1] && o2Angles[1] >= j3Limits[0] && o2Angles[1] <= j3Limits[1]) {
				y = o2Angles[0];
				z = o2Angles[1];
			}
			else failure();
		}

		else {
			if (o2Angles[0] >= j2Limits[0] && o2Angles[0] <= j2Limits[1] && o2Angles[1] >= j3Limits[0] && o2Angles[1] <= j3Limits[1]) {
				y = o2Angles[0];
				z = o2Angles[1];
			}
			else failure();
		}
	}
	else failure();
}

void IK::doubleCheck() {
	// TODO: These values aren't used!
	double finalZ1 = sin(o1Angles[0]) * L2 + sin(o1Angles[1]) * L3;
	double finalZ2 = sin(o2Angles[0]) * L2 + sin(o2Angles[1]) * L3;

	double finalY1 = sin(x) * (cos(o1Angles[0]) * L2 + cos(o1Angles[1]) * L3);
	double finalY2 = sin(x) * (cos(o2Angles[0]) * L2 + cos(o2Angles[1]) * L3);

	double finalX1 = cos(x) * (cos(o1Angles[0]) * L2 + cos(o1Angles[1]) * L3);
	double finalX2 = cos(x) * (cos(o2Angles[0]) * L2 + cos(o2Angles[1]) * L3);
}

void IK::getAngles(double newX, double newY, double newZ) {
	oldX = x;
	oldY = y;
	oldZ = z;
	x = atan2(newY, newX);

	chooseOption();
	doubleCheck();
}

void IK::endPointPos() {
	// TODO: These values aren't used!
	double endPointZ = sin(y) * L2 + sin(z) * L3;
	double endPointY = sin(x) * (cos(y) * L2 + cos(z) * L3);
	double endPointX = cos(x) * (cos(y) * L2 + cos(z) * L3);
}
