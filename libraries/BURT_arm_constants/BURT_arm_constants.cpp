#include "BURT_arm_constants.h"

double BurtArmConstants::extendLimits[2] = { -100,  100};
double BurtArmConstants::liftLimits[2] = { -2.61799, 1.8326 }; 
double BurtArmConstants::swivelLimits[2] = { -2.35619, 2.44346 };
double BurtArmConstants::pinchLimits[2] = {0, 1};
double BurtArmConstants::gripperLiftLimits[2] = {-PI, PI};
double BurtArmConstants::gripperRotateLimits[2] = {0, 2 * PI};
