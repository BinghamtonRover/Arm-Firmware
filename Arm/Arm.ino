#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"
#include "src/ik/IK.h"

#include "src/arm.pb.h"
#include "pinouts.h"

#define ARM_COMMAND_ID 0x23
#define USE_SERIAL_MONITOR true

/// When the joints are at their minimum angles (limit switches), these are the coordinates of the gripper.
Coordinates calibratedPosition = {x: 0, y: 366.420, z: 1117.336};

/// The (x, y, z) coordinates of the gripper. 
///
/// These coordinates are passed to the IK algorithms which return angles for the 
/// joints of the arm: [swivel], [shoulder], and [elbow]. 
Coordinates gripperPosition;

void handleCommand(const uint8_t* buffer, int length) {
  auto command = BurtProto::decode<ArmCommand>(buffer, length, ArmCommand_fields);

  // General commands
  if (command.stop) stopAllMotors();
  if (command.calibrate) calibrateAllMotors();

  // Debug control: Move by individual steps
  if (command.move_swivel_steps != 0) swivel.debugMoveBySteps(command.move_swivel_steps);
  if (command.move_shoulder_steps != 0) shoulder.debugMoveBySteps(command.move_swivel_steps);
  if (command.move_elbow_steps != 0) shoulder.debugMoveBySteps(command.move_elbow_steps); 

  // Precise control: Move by radians
  if (command.move_swivel_radians != 0) swivel.moveBy(command.move_swivel_radians);
  if (command.move_shoulder_radians != 0) shoulder.moveBy(command.move_swivel_radians);
  if (command.move_elbow_radians != 0) shoulder.moveBy(command.move_elbow_radians); 

  // IK control to move motors by coordinates
  Coordinates destination(gripperPosition);  // copies the current position
  if (command.has_x) destination.x = command.x;
  if (command.has_y) destination.y = command.y;
  if (command.has_z) destination.z = command.z;
  setCoordinates(destination);
}

BurtCan can(ARM_COMMAND_ID, handleCommand);
BurtSerial serial(handleCommand);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");

  Serial.print("Initializing CAN...");
	can.setup();
  Serial.println("Done!");

  Serial.print("Preparing motors... ");
	swivel.presetup();
	shoulder.presetup();
	elbow.presetup();
  Serial.println("Done!");

  Serial.println("Initializing motors...");
	swivel.setup();
	shoulder.setup();
	elbow.setup();

  Serial.println("Calibrating motors...");
	calibrateAllMotors();

	Serial.println("Arm subsystem ready");
}

void loop() {
  // Update communications
  can.update();
  if (USE_SERIAL_MONITOR) updateSerialMonitor();
  else serial.update();

  // Update motors
  swivel.update();
  shoulder.update();
  elbow.update();
}

void calibrateAllMotors() {
	swivel.calibrate();
	shoulder.calibrate();
	elbow.calibrate();
  gripperPosition = calibratedPosition;
}

void stopAllMotors() {
  swivel.stop();
  shoulder.stop();
  elbow.stop();
}

void setCoordinates(Coordinates destination) { 
  Serial.print("Going to ");
  printCoordinates(gripperPosition);

  Angles newAngles = ArmIK::calculateAngles(destination);
  if (newAngles.isFailure()) {
  	Serial.println("IK Failed"); 
  	return;
  } else {
	  Serial.print("IK finished: ");
  	newAngles.println();
  }

  swivel.moveTo(newAngles.theta);
  shoulder.moveTo(newAngles.B);
  elbow.moveTo(newAngles.C);
  gripperPosition = destination;
}

void updateSerialMonitor() {
	if (!Serial.available()) return;
	
  // Assuming max size of 4 arguments, feel free to change
  // String args[4] = {"", "", "", ""};
  // int index = 0;
  // while (Serial.available()) {
  // 	char character = Serial.read();
  // 	if (character == ' ') index++;
  // 	if (index == 4) {
  // 		Serial.println("Use 4 or less arguments");
  // 		Serial.readString();  // clear the rest of the buffer
  // 		return;
  // 	}
  // 	args[index].concat(character);
  // }

  String input = Serial.readString();
  if (input == "calibrate") return calibrateAllMotors();
  else if (input == "stop") return stopAllMotors();
  
	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	int delimiter2 = input.indexOf(" ", delimiter + 1);
	if (delimiter2 != -1) {  // was given an x, y, z command
		float x = input.substring(0, delimiter).toFloat();
		float y = input.substring(delimiter + 1, delimiter2).toFloat();
		float z = input.substring(delimiter2).toFloat();
		Serial.print("Moving to: (");
			Serial.print(x); Serial.print(", ");
			Serial.print(y); Serial.print(", ");
			Serial.print(z); 
		Serial.println(").");
		gripperPosition.x = x;
		gripperPosition.y = y;
		gripperPosition.z = z;
		setCoordinates(gripperPosition);
		return;
	}
	String command = input.substring(0, delimiter);
	float arg = input.substring(delimiter + 1).toFloat();

	if (command == "swivel") {
    swivel.moveTo(arg);
	} else if (command == "shoulder") {
    shoulder.moveTo(arg);
	} else if (command == "elbow") {
    elbow.moveTo(arg);
	} else if (command == "precise-swivel") swivel.debugMoveBySteps(arg);
	else if (command == "precise-elbow") elbow.debugMoveBySteps(arg);
	else if (command == "precise-shoulder") shoulder.debugMoveBySteps(arg);
  else Serial.println("Unrecognized command");
}
