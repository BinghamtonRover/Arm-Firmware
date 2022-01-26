# ArmIK
Arm Inverse Kinematics.

 To use create the object with no constructor so just IK name;
 
 ## name.getAngles(double X, double Y, double Z, double currentAngles[]) will calculate and set the angles to get to the point given by coordinates X, Y, Z
 
 There will be two options, and a recommended option
 name.o1Angles, name.o2Angles, and name.newAngles are the options o1 and o2Angles are the two possibilities and newAngles is the reccomendation.
 Since there is only one possibility for the base rotation (XY direction) o1Angles and o2Angles only list the two joints in the YZ direction.
 name.newAngles (the recommended one) lists all three. 
 Recommendation is based off of which one can the arm get to within it's limits and if both work it recommends the one that requires the least movement.
 
 ## name.endPointPos(double currentAngles[]) will set name.endPointX, name.endPointY, and name.endPointZ to where the end of the arm (base of the gripper) will be
 based on the current angles (currentAngles should be an array of three doubles 
  
 ## name.doubleCheck() 
 I used it for basic testing. It just does the math for each of the calculated angle options to see where the  
 
 ## There are a bunch of other functions within IK, but they are all just used in getAngles() and shouldn't come up in the Arduino side
