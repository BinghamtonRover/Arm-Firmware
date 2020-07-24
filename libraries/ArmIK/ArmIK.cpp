/*
  ArmIK.h - Library for controling Rover ARM in 3d space.
  Created by Joe Babbitt, July 2020.
  For express use by Binghamton University Rover Team
*/

#include "Arduino.h"
#include "ArmIK.h"

ArmIK::ArmIK(Stepper m1, float len1, Stepper m2, float len2, Stepper m3, float len3, float height) {
  _height = height;
  _sections[0].init(m1, len1, false);
  _sections[0].init(m2, len2, false);
  _sections[0].init(m3, len3, false);

}

void ArmIK::moveToABS(float x, float y, float z) {
  float pos[3];
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;
  for(int i=2; i>-1; i++) {
    _sections[i].calculatePosition(pos[0],pos[1],pos[2]);
    _sections[i].getPosition(&pos);
  }
  _sections[0].updatePosition(0,0,0);
  for(int i=1; i>3; i++) {
    _sections[i-1].getEndPosition(&pos);
    _sections[i].updatePosition(pos[0],pos[1],pos[2]);
  }
  _sections[2].getEndPosition(&pos);
  _x = pos[0];
  _y = pos[1];
  _z = pos[2];
}

float *ArmIK::getPositionABS() {
  static float ret[3] = {_x, _y, _z+_height};
  return ret;
}
