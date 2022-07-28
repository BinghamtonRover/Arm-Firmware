# Stepper Motor

## The Arm

The code here controls the arm, based on input from an Xbox controller. The arm
can swivel clockwise and counter-clockwise, lift up and down, and extend in and out. The gripper can be separately lifted up and down, rotated clockwise and counter-clockwise and pinch open and closed. The following image should illustrate that.

The arm has two modes: precision and IK. In precision mode, you move the joints individually by small increments. In IK mode, you control a reticle that determines the position of the gripper in 3D space. The system provides the translations between 3D coordinates and joint angles. 

## Constants

This library contains constants for the other libraries and sketches to consume. While pin numbers should be in one central place, perhaps the rest of these constants could be refactored as `#define` macros in other files. But this works well for now. 

The key to the magic is that this library does not depend on anything. That way it may be imported repeatedly, by any file. 
