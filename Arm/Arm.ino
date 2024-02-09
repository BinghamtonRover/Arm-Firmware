#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"
#include "src/ik/IK.h"

#include "src/arm.pb.h"
#include "pinouts.h"

#define ARM_COMMAND_ID 0x23
#define ARM_DATA_ID 0x15
#define DATA_SEND_INTERVAL 1000
#define USE_SERIAL_MONITOR false

/// When the joints are at their minimum angles (limit switches), these are the coordinates of the gripper.
// Coordinates calibratedPosition = {x: 0, y: 366.420, z: 1117.336};

/// The (x, y, z) coordinates of the gripper. 
///
/// These coordinates are passed to the IK algorithms which return angles for the 
/// joints of the arm: [swivel], [shoulder], and [elbow]. 
// Coordinates gripperPosition;

unsigned long nextSendTime;

void handleCommand(const uint8_t* buffer, int length);

BurtCan<Can3> can(ARM_COMMAND_ID, Device::Device_ARM, handleCommand);
BurtSerial serial(handleCommand, Device::Device_ARM);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");

  Serial.print("Initializing CAN...");
	can.setup();
  nextSendTime = millis() + DATA_SEND_INTERVAL;
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

  // Serial.println("Calibrating motors...");
	// calibrateAllMotors();

	Serial.println("Arm subsystem ready");  
}

void loop() {
  // Update communications
  can.update();
  if (USE_SERIAL_MONITOR) updateSerialMonitor();
  else serial.update();
  sendData();

  // Update motors
  swivel.update();
  shoulder.update();
  elbow.update();
}

// TODO: Validate this sends only 8 bytes
void sendMotorData(ArmData arm, StepperMotor& motor, MotorData* pointer) {
  MotorData data;

  data = MotorData_init_zero;
  data.is_moving = motor.isMoving();
  *pointer = data;
  can.send(ARM_DATA_ID, &arm, ArmData_fields);

  data = MotorData_init_zero;
  data.is_limit_switch_pressed = motor.isLimitSwitchPressed();
  *pointer = data;
  can.send(ARM_DATA_ID, &arm, ArmData_fields);

  data = MotorData_init_zero;
  data.current_step = motor.getPosition();
  *pointer = data;
  can.send(ARM_DATA_ID, &arm, ArmData_fields);

  data = MotorData_init_zero;
  data.target_step = motor.getPosition();
  *pointer = data;
  can.send(ARM_DATA_ID, &arm, ArmData_fields);

  data = MotorData_init_zero;
  data.angle = motor.getPosition();
  *pointer = data;
  can.send(ARM_DATA_ID, &arm, ArmData_fields);
}

/* TODO: Send IK data */
void sendData() {
  if (millis() < nextSendTime) return;

  ArmData arm = ArmData_init_zero;
  sendMotorData(arm, swivel, &arm.base);
  arm = ArmData_init_zero;
  sendMotorData(arm, shoulder, &arm.shoulder);
  arm = ArmData_init_zero;
  sendMotorData(arm, elbow, &arm.elbow);

  nextSendTime = millis() + DATA_SEND_INTERVAL;
}

void calibrateAllMotors() {
	swivel.calibrate();
  shoulder.calibrate();
  elbow.calibrate();
  // gripperPosition = calibratedPosition;
}

void stopAllMotors() {
  swivel.stop();
  shoulder.stop();
  elbow.stop();
}

// void setCoordinates(Coordinates destination) { 
//   Serial.print("Going to ");
//   printCoordinates(destination);

//   Angles newAngles = ArmIK::calculateAngles(destination);
//   if (newAngles.isFailure()) return;

//   swivel.moveTo(newAngles.theta);
//   shoulder.moveTo(newAngles.B);
//   elbow.moveTo(newAngles.C);
//   gripperPosition = destination;
// }

// void jab() {
//   // TODO: Move the arm forward 
// }

void updateSerialMonitor() {
	if (!Serial.available()) return;

  String input = Serial.readString();
	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	// int delimiter2 = input.indexOf(" ", delimiter + 1);
	// if (delimiter2 != -1) {  // was given an x, y, z command
	// 	float x = input.substring(0, delimiter).toFloat();
	// 	float y = input.substring(delimiter + 1, delimiter2).toFloat();
	// 	float z = input.substring(delimiter2).toFloat();
	// 	setCoordinates({x: x, y: y, z: z});
	// 	return;
	// }
	String command = input.substring(0, delimiter);
	float arg = input.substring(delimiter + 1).toFloat();

	if (command == "swivel") {
    Serial.println("Doing something");
    swivel.moveTo(arg);
	} else if (command == "shoulder") {
    shoulder.moveTo(arg);
	} else if (command == "elbow") {
    elbow.moveTo(arg);
	} else if (command == "precise-swivel") swivel.moveBySteps(arg);
	else if (command == "precise-elbow") elbow.moveBySteps(arg);
	else if (command == "precise-shoulder") shoulder.moveBySteps(arg);
  else Serial.println("Unrecognized command");
}

void handleCommand(const uint8_t* buffer, int length) {
  auto command = BurtProto::decode<ArmCommand>(buffer, length, ArmCommand_fields);
  Serial.print("Received command: Calibrate=");
  Serial.print(command.calibrate);
  Serial.print(", swivel=");
  Serial.println(command.swivel.move_steps);

  // General commands
  if (command.stop) stopAllMotors();
  if (command.calibrate) calibrateAllMotors();

  // Debug control: Move by individual steps
  if (command.swivel.move_steps != 0) swivel.moveBySteps(command.swivel.move_steps);
  if (command.shoulder.move_steps != 0) shoulder.moveBySteps(command.shoulder.move_steps);
  if (command.elbow.move_steps != 0) elbow.moveBySteps(command.elbow.move_steps);

  // Precise control: Move by radians
  if (command.swivel.move_radians != 0) swivel.moveBy(command.swivel.move_radians);
  if (command.shoulder.move_radians != 0) shoulder.moveBy(command.shoulder.move_radians);
  if (command.elbow.move_radians != 0) elbow.moveBy(command.elbow.move_radians);

  // IK control to move motors by coordinates
  // Coordinates newPosition = {gripperPosition.x, gripperPosition.y, gripperPosition.z}
  // if (command.ik_x != 0) newPosition.x = command.ik_x; 
  // if (command.ik_y != 0) newPosition.y = command.ik_y;
  // if (command.ik_z != 0) newPosition.z = command.ik_z;
  // setCoordinates(newPosition);
}
