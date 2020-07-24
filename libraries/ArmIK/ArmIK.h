#include <Stepper.h>
#include <Section.h>

/*
  ArmIK.h - Library for controling Rover ARM in 3d space.
  Created by Joe Babbitt, July 2020.
  For express use by Binghamton University Rover Team
*/
#ifndef ArmIK_h
#define ArmIK_h

#include "Arduino.h"

class ArmIK
{
  public:
    ArmIK(Stepper m1, float len1, Stepper m2, float len2, Stepper m3, float len3, float height); //NEED TO ADD FEEDBACK
    void init();
    float *getPositionABS();
    void moveToABS(float x, float y, float z);
    // void moveToREL();
  private:
    int _x;
    int _y;
    int _z;
    int _height;
    Section _sections[3];
};

#endif
