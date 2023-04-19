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
  if (command.swivel.move_steps != 0) swivel.debugMoveBySteps(command.swivel.move_steps);
  if (command.shoulder.move_steps != 0) shoulder.debugMoveBySteps(command.shoulder.move_steps);
  if (command.elbow.move_steps != 0) elbow.debugMoveBySteps(command.elbow.move_steps);

  // Precise control: Move by radians
  if (command.swivel.move_radians != 0) swivel.moveBy(command.swivel.move_radians);
  if (command.shoulder.move_radians != 0) shoulder.moveBy(command.shoulder.move_radians);
  if (command.elbow.move_radians != 0) elbow.moveBy(command.elbow.move_radians);

  // IK control to move motors by coordinates
  // Coordinates newPosition = {gripperPosition.x, gripperPosition.y, gripperPosition.z}
  Coordinates newPosition;
  if (command.ik_x != 0) newPosition.x = command.ik_x; 
  if (command.ik_y != 0) newPosition.y = command.ik_y;
  if (command.ik_z != 0) newPosition.z = command.ik_z;
  setCoordinates(newPosition);

  //if (command.jab) jab();
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
  elbow.calibrate();
	shoulder.calibrate();
  gripperPosition = calibratedPosition;
}

void stopAllMotors() {
  swivel.stop();
  shoulder.stop();
  elbow.stop();
}

void setCoordinates(Coordinates destination) { 
  Serial.print("Going to ");
  printCoordinates(destination);

  Angles newAngles = ArmIK::calculateAngles(destination);
  if (newAngles.isFailure()) return;

  swivel.moveTo(newAngles.theta);
  shoulder.moveTo(newAngles.B);
  elbow.moveTo(newAngles.C);
  gripperPosition = destination;
}

void jab() {
  // TODO: Move the arm forward 
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

  if (input == "calibrate\n") return calibrateAllMotors();
  else if (input = "stop\n") return stopAllMotors();
	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	int delimiter2 = input.indexOf(" ", delimiter + 1);
	if (delimiter2 != -1) {  // was given an x, y, z command
		float x = input.substring(0, delimiter).toFloat();
		float y = input.substring(delimiter + 1, delimiter2).toFloat();
		float z = input.substring(delimiter2).toFloat();
		setCoordinates({x: x, y: y, z: z});
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
