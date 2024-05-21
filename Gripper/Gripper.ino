#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"

#include "src/arm.pb.h"
#include "pinouts.h"

#define GRIPPER_COMMAND_ID 0x33
#define GRIPPER_DATA_ID 0x16
#define DATA_SEND_INTERVAL 250
#define USE_SERIAL_MONITOR false

void handleCommand(const uint8_t* data, int length);
void sendData();
void shutdown();

BurtCan<Can3> can(GRIPPER_COMMAND_ID, Device::Device_GRIPPER, handleCommand, shutdown);
BurtSerial serial(Device::Device_GRIPPER, handleCommand, shutdown);
BurtTimer dataTimer(DATA_SEND_INTERVAL, sendData);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");	

  Serial.print("Initializing communications...");
	can.setup();
  dataTimer.setup();
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

  // Serial.println("Calibrating motors...");
	// calibrateAllMotors();

	Serial.println("Gripper subsystem ready");
}

void loop() {
	// Update communications
	can.update();
	serial.update();
  dataTimer.update();

	// Update motors
	lift.update();
	rotate.update();
	pinch.update();
}

void sendMotorData(GripperData gripper, StepperMotor& motor, MotorData* pointer) {
  MotorData data;

  data = MotorData_init_zero;
  data.is_moving = motor.isMoving();
  *pointer = data;
  can.send(GRIPPER_DATA_ID, &gripper, GripperData_fields);

  data = MotorData_init_zero;
  data.is_limit_switch_pressed = motor.isLimitSwitchPressed();
  *pointer = data;
  can.send(GRIPPER_DATA_ID, &gripper, GripperData_fields);

  data = MotorData_init_zero;
  data.current_step = motor.getPosition();
  *pointer = data;
  can.send(GRIPPER_DATA_ID, &gripper, GripperData_fields);

  data = MotorData_init_zero;
  data.target_step = motor.getPosition();
  *pointer = data;
  can.send(GRIPPER_DATA_ID, &gripper, GripperData_fields);

  data = MotorData_init_zero;
  data.angle = motor.getPosition();
  *pointer = data;
  can.send(GRIPPER_DATA_ID, &gripper, GripperData_fields);
}

uint8_t version_buffer[6] = {0x22, 0x04, 0x08, 0x01, 0x10, 0x00};

/* TODO: Send IK data */
void sendData() {
  if (serial.isConnected) {
    Serial.write(version_buffer, 6);
    can.sendRaw(GRIPPER_DATA_ID, version_buffer, 6);
  }

  GripperData gripper = GripperData_init_zero;
  sendMotorData(gripper, lift, &gripper.lift);
  gripper = GripperData_init_zero;
  sendMotorData(gripper, rotate, &gripper.rotate);
  gripper = GripperData_init_zero;
  sendMotorData(gripper, pinch, &gripper.pinch);
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

void onDisconnect() { stopAllMotors(); }

void shutdown() { stopAllMotors(); }

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

	m->moveBySteps(steps); 
}

void handleCommand(const uint8_t* data, int length) {
  // heartbeats.registerHeartbeat();
  auto command = BurtProto::decode<GripperCommand>(data, length, GripperCommand_fields);
  if (command.stop) stopAllMotors();
  if (command.calibrate) calibrateAllMotors();

  // Debug control: Move by individual steps
  if (command.lift.move_steps != 0) {
    lift.moveBySteps(-command.lift.move_steps);
    rotate.moveBySteps(command.lift.move_steps);
  }
  if (command.rotate.move_steps != 0) {
    lift.moveBySteps(command.rotate.move_steps);
    rotate.moveBySteps(command.rotate.move_steps);
  }
  if (command.pinch.move_steps != 0) pinch.moveBySteps(command.pinch.move_steps);

  // Normal control: Move by radians
  if (command.lift.move_radians != 0) {
  	lift.moveBy(-command.lift.move_radians);
    rotate.moveBy(command.lift.move_radians);
  }
  if (command.rotate.move_radians != 0) {
  	lift.moveBy(command.rotate.move_radians);
  	rotate.moveBy(command.rotate.move_radians);
  }
  if (command.pinch.move_radians != 0) {
  	pinch.moveBy(command.pinch.move_radians);
  }
}
