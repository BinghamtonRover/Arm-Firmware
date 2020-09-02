//33-34

 
 setCoordinates() {
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
        else {
          setDir(1,0);
          step(1);
        }
      }
      if (abs(currentAngles[1] - nextAngles[1]) >= accuracy) {
        if (currentAngles[1] - nextAngles[1] >= accuracy) {
          setDir(2,1);
          step(2);
        }
        else {
          setDir(2,0);
          step(2);
        }
      }
      if (abs(currentAngles[2] - nextAngles[2]) >= accuracy) {
        if (currentAngles[2] - nextAngles[2] > accuracy) {
          setDir(3,1);
          step(3);
        }
        else {
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
