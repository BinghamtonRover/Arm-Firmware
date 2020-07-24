# ArmIK
Arm Inverse Kinematics.

 My goal was to make this pretty plug and play for us. It's just a simple library with high level interfaces that is easy to implement.

## Status
* IK algo is untested but *in theory* works.
* Motor control is not implemented since motors have not been finalized 
* No feedback exists to locate the joints in 3D space, waiting on solution for closed loop feedback.

## How to use
1. Import the `ArmIK` and `Section` libraries to your IDE
2. Import `ArmIK` into your `ino` file
3. Create an instance of an ArmIK object

An example instantiation is available in `ArmIK.ino`.

## Structure
Two libraries are included to be used as you see fit. 
### ArmIK
The ArmIK library is the parent library that allows you to create an ARM object and move it based on an IK model. It has the following methods
```
ArmIK (instantiation):
    Creates an ArmIK object

    Params:
    m1, m2, m3 : Stepper motors for each joint (Stepper)
    len1, len2, len3 : Length of each joint (float)
    height : height off the ground of the arm (float)

moveToABS:
    Moves the arm to the desired position in 3d space.
    0,0,0 is considered the ground directly in front of the arm at it's originating joint, centered from the top down.
    
    Params:
    x, y, z : self-explainitory (float)

getPostitionABS:
    returns the x,y,z position as a array of floats (length 3).

    Params:
    none
```
### Section
The Section library is the child library that manages all of the limbs of the arm.
```
Section (instantiation):
    ignore

    Params:
    none

init:
    sets up the limb to be used.

    Params:
    motor : self-explainitory (Stepper)
    len : length of limb (float)
    isZ : is the limb responsible for rotating the arm in the Z axis (bool)

getAngle:
    gets the angle (radian) of the joint (float)

    Params:
    none

getPosition:
    gets the X,Y,Z position of the joints (float[])

    Params:
    ret : return float array, must be of length 3. (User is responsible for memory usage)

getEndPosition:
    gets the X,Y,Z position of the end of each limb (float[])

    Params:
    ret : return float array, must be of length 3. (User is responsible for memory usage)

updatePosition:
    Moves the limb to best possible position in 3d space to get to target.

    Params:
    x, y, z : 3d coordinates (float)

calculatePosition:
    Generates the most optimisitc point at which the limb could exist to reach it's goal (whether that be the desiered point or a child limb).
    This will also EVENTUALLY update the actual motors to reach their correct position.

    Params:
    x, y, z : 3d coordinates (float)
```