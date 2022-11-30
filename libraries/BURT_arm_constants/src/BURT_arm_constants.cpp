/*! @file 
 * \addtogroup Constants 
 * \{
*/

#include "BURT_arm_constants.h"

// Real arm limits
// double ArmConstants::extendLimits[2] = { -100,  100};
// double ArmConstants::liftLimits[2] = { -2.61799, 1.8326 }; 
// double ArmConstants::swivelLimits[2] = { -2.35619, 2.44346 };
// double ArmConstants::pinchLimits[2] = {0, 1};
// double ArmConstants::gripperLiftLimits[2] = {-PI, PI};
// double ArmConstants::gripperRotateLimits[2] = {0, 2 * PI};

// Demo arm limits
double ArmConstants::extendLimits[2] = { 0, PI / 3};
double ArmConstants::liftLimits[2] = { 0, PI / 3}; 
double ArmConstants::swivelLimits[2] = { 0, PI / 3};
double ArmConstants::pinchLimits[2] = { 0, PI / 3};
double ArmConstants::gripperLiftLimits[2] = { 0, PI / 3};
double ArmConstants::gripperRotateLimits[2] = { 0, PI / 3};

// The following close bracket marks the file for Doxygen
/*! \} */
