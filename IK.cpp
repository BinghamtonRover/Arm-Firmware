#include "IK.h"

//Check rotation validity first

double IK::getYPrime(double x, double y) {
	return sqrt(x * x + y * y);
}

double IK::getIntersectionY1(double a, double b, double c, double d) {
	double D = sqrt((c - a) * (c - a) + (d - b) * (d - b));
	double gamma = (1.0 / 4.0) * sqrt((D + L2 + L3) * (D + L2 - L3) * (D - L2 + L3) * (-D + L2 + L3));
	double tool1 = (a + c) / 2.0;
	double tool2 = (c - a) * (L2 * L2 - L3 * L3) / (2.0 * D * D);
	double tool3 = 2.0 * (b - d / (D * D)) * gamma;
	return (tool1 + tool2 + tool3);
}

double IK::getIntersectionY2(double a, double b, double c, double d) {
	double D = sqrt((c - a) * (c - a) + (d - b) * (d - b));
	double gamma = (1.0 / 4.0) * sqrt((D + L2 + L3) * (D + L2 - L3) * (D - L2 + L3) * (-D + L2 + L3));
	double tool1 = (a + c) / 2.0;
	double tool2 = (c - a) * (L2 * L2 - L3 * L3) / (2 * D * D);
	double tool3 = 2.0 * ((b - d) / (D * D)) * gamma;
	return (tool1 + tool2 - tool3);
}

double IK::getIntersectionX1(double a, double b, double c, double d) {
	double D = sqrt((c - a) * (c - a) + (d - b) * (d - b));
	double gamma = (1.0 / 4.0) * sqrt((D + L2 + L3) * (D + L2 - L3) * (D - L2 + L3) * (-D + L2 + L3));
	double tool1 = (b + d) / 2.0;
	double tool2 = (d - b) * (L2 * L2 - L3 * L3) / (2.0 * D * D);
	double tool3 = 2.0 * ((a - c) / (D * D)) * gamma;
	return (tool1 + tool2 - tool3);
}

double IK::getIntersectionX2(double a, double b, double c, double d) {
	double D = sqrt((c - a) * (c - a) + (d - b) * (d - b));
	double gamma = (1.0 / 4.0) * sqrt((D + L2 + L3) * (D + L2 - L3) * (D - L2 + L3) * (-D + L2 + L3));
	double tool1 = (b + d) / 2.0;
	double tool2 = (d - b) * ((L2 * L2) - (L3 * L3)) / (2.0 * D * D);
	double tool3 = 2 * ((a - c) / (D * D)) * gamma;
	return (tool1 + tool2 + tool3);
}

double IK::getAngle3(double y1, double z1, double y, double z) {
	return atan2((z - z1), (y - y1));
}

void IK::failure(double currentAngles[]) {
	newAngles[0] = currentAngles[0];
	newAngles[1] = currentAngles[1];
	newAngles[2] = currentAngles[2];
	std::cout << "This position could not be reached\n";
}

void IK::chooseOption(double currentAngles[]) {
	if (newAngles[0] >= j1Limits[0] && newAngles[0] <= j1Limits[1]) {
		if (((fabs(currentAngles[1] - o1Angles[0])) + (fabs(currentAngles[2] - o1Angles[1]))) <= (((fabs(currentAngles[1] - o2Angles[0])) + (fabs(currentAngles[2] - o2Angles[1]))))) {
			if (o1Angles[0] >= j2Limits[0] && o1Angles[0] <= j2Limits[1] && o1Angles[1] >= j3Limits[0] && o1Angles[1] <= j3Limits[1]) {
				newAngles[1] = o1Angles[0];
				newAngles[2] = o1Angles[1];
			}
			else if (o2Angles[0] >= j2Limits[0] && o2Angles[0] <= j2Limits[1] && o2Angles[1] >= j3Limits[0] && o2Angles[1] <= j3Limits[1]) {
				newAngles[1] = o2Angles[0];
				newAngles[2] = o2Angles[1];
			}
			else failure(currentAngles);
			}
		}

		else {
			if (o2Angles[0] >= j2Limits[0] && o2Angles[0] <= j2Limits[1] && o2Angles[1] >= j3Limits[0] && o2Angles[1] <= j3Limits[1]) {
				newAngles[1] = o2Angles[0];
				newAngles[2] = o2Angles[1];
			}
			else if ()
			else failure(currentAngles);
		}
	}
	else failure(currentAngles);
}

void IK::doubleCheck() {
	finalZ1 = sin(o1Angles[0]) * L2 + sin(o1Angles[1]) * L3;
	finalZ2 = sin(o2Angles[0]) * L2 + sin(o2Angles[1]) * L3;

	finalY1 = sin(newAngles[0]) * (cos(o1Angles[0]) * L2 + cos(o1Angles[1]) * L3);
	finalY2 = sin(newAngles[0]) * (cos(o2Angles[0]) * L2 + cos(o2Angles[1]) * L3);

	finalX1 = cos(newAngles[0]) * (cos(o1Angles[0]) * L2 + cos(o1Angles[1]) * L3);
	finalX2 = cos(newAngles[0]) * (cos(o2Angles[0]) * L2 + cos(o2Angles[1]) * L3);
}

void IK::getAngles(double x, double y, double z, double currentAngles[]) {
	//checkPossible();
	newAngles[0] = atan2(y, x);

	yPrime = getYPrime(x, y);

	Y1 = getIntersectionX1(0, 0, z, yPrime);
	Y2 = getIntersectionX2(0, 0, z, yPrime);

	Z1 = getIntersectionY1(0, 0, z, yPrime);
	Z2 = getIntersectionY2(0, 0, z, yPrime);

	o1Angles[0] = atan2(Z1, Y1);
	o1Angles[1] = getAngle3(Y1, Z1, yPrime, z);

	o2Angles[0] = atan2(Z2, Y2);
	o2Angles[1] = getAngle3(Y2, Z2, yPrime, z);

	chooseOption(currentAngles);
	doubleCheck();
}

void IK::endPointPos(double currentAngles[]) {
	endPointZ = sin(currentAngles[1]) * L2 + sin(currentAngles[2]) * L3;

	endPointY = sin(currentAngles[0]) * (cos(currentAngles[1]) * L2 + cos(currentAngles[2]) * L3);

	endPointX = cos(currentAngles[0]) * (cos(currentAngles[1]) * L2 + cos(currentAngles[2]) * L3);
}
