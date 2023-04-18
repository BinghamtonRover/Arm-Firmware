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
  if (command.move_lift != 0) wristLift.debugMoveBySteps(command.move_lift * LIFT_INCREMENT);
  if (command.move_rotate != 0) wristRotate.debugMoveBySteps(command.move_rotate * ROTATE_INCREMENT);
  if (command.move_gripper != 0) gripper.debugMoveBySteps(command.move_gripper * GRIPPER_INCREMENT);
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

	StepperMotor* m = &wristLift;
	if (motor == "lift") m = &wristLift;
	else if (motor == "rotate") m = &wristRotate;
	else if (motor == "pinch") m = &gripper;
	else {
	  Serial.print("Cannot move motor: [");
	  Serial.print(motor);
	  Serial.println("].");
	}

	m->debugMoveToStep(steps); 
}
