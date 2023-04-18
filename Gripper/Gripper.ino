#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"

#include "src/arm.pb.h"
#include "pinouts.h"

#define GRIPPER_COMMAND_ID 0x33
#define USE_SERIAL_MONITOR true

void handleCommand(const uint8_t* data, int length) {
  auto command = BurtProto::decode<GripperCommand>(data, length, GripperCommand_fields);
  if (command.stop) stopAllMotors();
  if (command.calibrate) calibrateAllMotors();

  // Move by steps
  if (command.move_lift_steps != 0) lift.debugMoveBySteps(command.move_lift_steps);
  if (command.move_rotate_steps != 0) rotate.debugMoveBySteps(command.move_rotate_steps);
  if (command.move_pinch_steps != 0) pinch.debugMoveBySteps(command.move_pinch_steps);

  // Move by steps
  if (command.move_lift_radians != 0) lift.moveBy(command.move_lift_radians);
  if (command.move_rotate_radians != 0) rotate.moveBy(command.move_rotate_radians);
  if (command.move_pinch_radians != 0) pinch.moveBy(command.move_pinch_radians);
}

BurtCan can(GRIPPER_COMMAND_ID, handleCommand);
BurtSerial serial(handleCommand);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");	

  Serial.print("Initializing CAN...");
	can.setup();
  Serial.println("Done!");

  Serial.print("Preparing motors... ");
	lift.presetup();
	rotate.presetup();
	pinch.presetup();
  Serial.println("Done!");

  Serial.println("Initializing motors...");
	lift.setup();
	rotate.setup();
	pinch.setup();

  Serial.println("Calibrating motors...");
	calibrateAllMotors();

	Serial.println("Gripper subsystem ready");
}

void loop() {
	// Update communications
	can.update();
	if (USE_SERIAL_MONITOR) updateSerialMonitor();
	else serial.update();

	// Update motors
	lift.update();
	rotate.update();
	pinch.update();
}

void calibrateAllMotors() {
	lift.calibrate();
	rotate.calibrate();
	pinch.calibrate();
}

void stopAllMotors() {
  lift.stop();
  rotate.stop();
  pinch.stop();
}

void updateSerialMonitor() {
	String input = Serial.readString();
	if (input == "") return;

	// Parse the command
	int delimiter = input.indexOf(" ");
	if (delimiter == -1) return;
	String part1 = input.substring(0, delimiter);
	String part2 = input.substring(delimiter + 1);
	String motor = part1;
	int steps = part2.toInt();

	StepperMotor* m = &lift;
	if (motor == "lift") m = &lift;
	else if (motor == "rotate") m = &rotate;
	else if (motor == "pinch") m = &pinch;
	else {
	  Serial.print("Cannot move motor: [");
	  Serial.print(motor);
	  Serial.println("].");
	}

	m->debugMoveToStep(steps); 
}
