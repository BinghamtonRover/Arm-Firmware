/*
  Section.cpp - Library for controling Rover ARM in 3d space [Segment portion].
  Created by Joe Babbitt, July 2020.
  For express use by Binghamton University Rover Team
*/

#include "Arduino.h"
#include "Section.h"

Section::Section() {
}

void Section::init(Stepper motor, float len, bool isZ) {
  // Add init code to reset 0 of motor; 
  _positionX = 0;
  _positionY = 0;
  _endPointX = 0 + _len;
  _endPointY = 0;
  _angle = 0;
  _isZ = isZ;
}

float Section::getAngle() {
  return _angle;
}

void Section::getPosition(float (*ret)[3]) {
  *ret[0] = _positionX;
  *ret[1] = _positionY;
  *ret[2] = _positionZ;
}

void Section::getEndPosition(float (*ret)[3]) {
  *ret[0] = _endPointX;
  *ret[1] = _endPointY;
  *ret[2] = _endPointZ;
}

void Section::updatePosition(float x, float y, float z) {
  _positionX = x;
  _positionY = y;
  _positionY = z;
  _endPointX = x + (_len * cos(_angle));
  _endPointY = y + (_len * sin(_angle));
  _endPointY = z + (_len * sin(_angleZ));
}

void Section::calculatePosition(float x, float y, float z) {
  float desAngleZ = atan2( x - _positionX, z - _positionZ);
  float offsetX = 1 + (1 - cos(desAngleZ));
  x *= offsetX;
  float desAngle = atan2( y - _positionY, x - _positionX );
  _angle = desAngle;
  _angleZ = desAngleZ;
  _endPointX = x;
  _endPointY = y;
  _endPointZ = z;
  _positionX = x - (_len * cos(desAngle));
  _positionY = y - (_len * sin(desAngle));
  _positionZ = z - (_len * sin(desAngleZ));
  // MOVE MOTOR TO MATCH _angle HERE unless _isZ, then use _angleZ
  // Two "views" are used here. 
  // 2D dictates the three motors such that the robot can reach inside a box
  // Top dictates the amount that the arm must further extend in order to reach the desired "R" (Z) value without curving X and Y. 
}