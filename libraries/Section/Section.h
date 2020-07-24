#include <Stepper.h>
#include <math.h>

/*
  Section.h - Library for controling Rover ARM in 3d space [Segment portion].
  Created by Joe Babbitt, July 2020.
  For express use by Binghamton University Rover Team
*/
#ifndef Section_h
#define Section_h

#include "Arduino.h"

class Section
{
  public:
    Section(); 
    void init(Stepper motor, float len, bool isZ); //NEED TO ADD FEEDBACK
    float getAngle();
    void getPosition(float (*ret)[3]);
    void getEndPosition(float (*ret)[3]);
    void updatePosition(float x, float y, float z);
    void calculatePosition(float x, float y, float z);
  private:
    float _angle;
    float _angleZ;
    float _len;
    float _positionX;
    float _positionY;
    float _positionZ;
    float _endPointX;
    float _endPointY;
    float _endPointZ;
    bool _isZ;
};

#endif
