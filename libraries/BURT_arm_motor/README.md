# Stepper Motor

## The Arm

The code here controls the arm, based on input from an Xbox controller. The arm
can swivel clockwise and counter-clockwise, lift up and down, and extend in and out. The gripper can be separately lifted up and down, rotated clockwise and counter-clockwise and pinch open and closed. The following image should illustrate that.

The arm has two modes: precision and IK. In precision mode, you move the joints individually by small increments. In IK mode, you control a reticle that determines the position of the gripper in 3D space. The system provides the translations between 3D coordinates and joint angles. 

## The stepper motors

The arm uses TMC 5160 stepper motors, which have a lot of features but are complex and hard to use without adding bloat to a `.ino` sketch. This library serves to simplify all the aspects of oeprating a TMC 5160 stepper motor with the `StepperMotor` class. It's implementation builds on [`TMCStepper`](https://github.com/teemuatlut/TMCStepper/tree/Release_v1) library, which it uses as a backend. 

## Usage

Read the full documentation for details, but here are a few basics:

### Initialize the motor

Declare your motor as a variable and initialize in `setup`: 

```cpp
StepperMotor myMotor = StepperMotor(chipSelectPin, enablePin, rmsCurrent, minBound, maxBound, gearboxRatio);

void setup() {
	myMotor.setup();
	myMotor.calibrate();
}
```

### Maintaining the motor

The motor takes a while to move to its destination, and may stall along the way. Add some boilerplate to your `loop` to handle these cases:

```cpp
void loop() {
	myMotor.fixPotentialStall();  // check and act on stalls
	if (!myMotor.isFinished()) return;  // still en-route
	// Now you can safely move the motor
}
```

### Moving the motor

You have three options for moving the motor:

1. Move by a given number of steps (debug):
```cpp
myMotor.debugMoveSteps(50);  // move 50 steps as a test
```

2. Move by a given number of radians: 
```cpp
myMotor.moveBy(PI);  // move a half rotation
```

3. Move to a given rotation:
```cpp
myMotor.moveTo(0);  // back to the home position
```
