/* Computes and saves the angles required to move to the given position 

 Since 3D is confusing, let's define our axes from the operator's perspective:
 - The X-axis: left(-) and right(+)
 - The Y-axis: down(-) and up(+)
 - The Z-axis: backward(-) and forward(+)

 This function sets [shoulderAngle], [elbowAngle], and [wristAngle] such
 that the end effector -- the hand -- is at the given position. Since the
 controls work in two dimensions at a time, we have two ways of moving the arm: 
   1. forward and backwards, such that the hand is only moving in the Z-axis.
   2. horizontal and vertical, such that the Z-position remains constant.

  For motion 1 we need to use IK, but for motion 2 it would suffice to simply
  rotate the base at the shoulder (ie, swivel the arm).
*/

void setCoordinates() {
  switch(mainMovement) {
    case forward:
      nextX += movementResolution;
      break;
    case backward:
      nextX -= movementResolution;
      break;
    case up:
      nextZ += movementResolution;
      break;
    case down:
      nextZ -= movementResolution;
      break;
    case left:
      nextY -= movementResolution;
      break;
    case right:
      nextY += movementResolution;
      break;
    case stay:
      break;
  }
}

void runInverseKinematics() {
  setCoordinates();
  
  currentAngles[0] = sensorReadJ1(analogRead(J1Sensor));
  currentAngles[1] = sensorReadJ2(analogRead(J2Sensor));
  currentAngles[2] = sensorReadJ3(analogRead(J3Sensor));
  
  IK.getAngles(nextX, nextY, nextZ, currentAngles);

  nextAngles[0] = IK.newAngles[0];
  nextAngles[1] = IK.newAngles[1];
  nextAngles[2] = IK.newAngles[2];
  
  while(abs(currentAngles[0] - nextAngles[0]) >= accuracy || abs(currentAngles[1] - nextAngles[1]) >= accuracy || abs(currentAngles[2] - nextAngles[2]) >= accuracy) {
      if (abs(currentAngles[0] - nextAngles[0]) >= accuracy) {
        if (currentAngles[0] - nextAngles[0] > accuracy) {
          setDir(1,1);
          step(1);
        }
        else  if(currentAngles[0] - nextAngles[0] < accuracy) {
          setDir(1,0);
          step(1);
        }
      }
      if (abs(currentAngles[1] - nextAngles[1]) >= accuracy) {
        if (currentAngles[1] - nextAngles[1] >= accuracy) {
          setDir(2,1);
          step(2);
        }
        else  if(currentAngles[1] - nextAngles[0] < accuracy) {          
          setDir(2,0);
          step(2);
        }
      }
      if (abs(currentAngles[2] - nextAngles[2]) >= accuracy) {
        if (currentAngles[2] - nextAngles[2] > accuracy) {
          setDir(3,1);
          step(3);
        }
        else  if(currentAngles[2] - nextAngles[0] < accuracy) {
          setDir(3,0);
          step(3);
        }
      }
      currentAngles[0] = sensorReadJ1(analogRead(J1Sensor));
      currentAngles[1] = sensorReadJ2(analogRead(J2Sensor));
      currentAngles[2] = sensorReadJ3(analogRead(J3Sensor));
      delayMicroseconds(microsBetween);
  }
}
