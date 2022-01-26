void gripperLevel() {
  double level = -(sensorReadJ2(analogRead(J2Sensor)) + sensorReadJ3(analogRead(J3Sensor)));
  double gripperAngle = sensorReadJ4(analogRead(J4Sensor));
  switch(gripperLift) {
    case left: //straight down
      while (abs(gripperAngle - (level - HALF_PI)) >= accuracy) {
        if (gripperAngle - (level - HALF_PI) >= accuracy) {
          setDir(4,0);
          step(4);
        }
        else {
          setDir(4,1);
          step(4);
        }
        delayMicroseconds(microsBetween);
      }
      gripperAngle = sensorReadJ4(analogRead(J4Sensor));
      break;
    case right: //flat
      while (abs(gripperAngle - level) >= accuracy) {
        if (gripperAngle - level >= accuracy) {
          setDir(4,0);
          step(4);
        }
        else {
          setDir(4,1);
          step(4);
        }
        gripperAngle = sensorReadJ4(analogRead(J4Sensor));
        delayMicroseconds(microsBetween);
      }
      break;
    default: //straight up
      while (abs(gripperAngle - (level + HALF_PI)) >= accuracy) {
        if (gripperAngle - (level + HALF_PI) >= accuracy) {
          setDir(4,0);
          step(4);
        }
        else {
          setDir(4,1);
          step(4);
        }
        gripperAngle = sensorReadJ4(analogRead(J4Sensor));
        delayMicroseconds(microsBetween);
      }
      break;
  }
}

void gripperTwist() {
  switch(gripperRotation) {
    case left:
      setDir(5,0);
      step(5);
      delayMicroseconds(microsBetween);
      break;
    case right:
      setDir(5,1);
      step(5);
      delayMicroseconds(microsBetween);
      break;
    default:
      break;
  }
}

void openCloseFingers() {
  sensorReadJ6(analogRead(J6Sensor));
  switch(gripperFingers) {
    case left:
      while (!gripperClosed) {
        setDir(6,0);
        step(6);
      }
      break;
    case right:
      while (!gripperOpen) {
        setDir(6,1);
        step(6);
      }
      break;
    default:
      break;
  }
}
