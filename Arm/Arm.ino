#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"
#include "src/ik/IK.h"

#include "src/arm.pb.h"
#include "pinouts.h"

#define ARM_COMMAND_ID 0x23
#define ARM_DATA_ID 0x15
#define DATA_SEND_INTERVAL 100

/// When the joints are at their minimum angles (limit switches), these are the coordinates of the gripper.
// Coordinates calibratedPosition = {x: 0, y: 366.420, z: 1117.336};

/// The (x, y, z) coordinates of the gripper. 
///
/// These coordinates are passed to the IK algorithms which return angles for the 
/// joints of the arm: [swivel], [shoulder], and [elbow]. 
// Coordinates gripperPosition;

Version version = {major: 1, minor: 1};

void stopAllMotors() {
  swivel.stop();
  shoulder.stop();
  elbow.stop();
}

void handleCommand(const uint8_t* buffer, int length);
void sendData();

BurtCan<Can3> can(ARM_COMMAND_ID, handleCommand);
BurtSerial serial(Device::Device_ARM, handleCommand, ArmData_fields, ArmData_size);
BurtTimer dataTimer(DATA_SEND_INTERVAL, sendData);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");
  // Keep the laser on
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  Serial.print("Initializing communications...");
	can.setup();
  dataTimer.setup();

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
  serial.update();
  dataTimer.update();

  // Update motors
  swivel.update();
  shoulder.update();
  elbow.update();
}

MotorData getMotorData(StepperMotor& motor) {
  return {
    is_moving: motor.isMoving() ? BoolState::BoolState_YES : BoolState::BoolState_NO,
    is_limit_switch_pressed: motor.limitSwitch.isPressed() ? BoolState::BoolState_YES : BoolState::BoolState_NO,
    current_step: motor.currentSteps(),
    target_step: motor.targetSteps(),
    angle: (float) motor.currentPosition(),
  };
}

/* TODO: Send IK data */
void sendData() {
  ArmData data = ArmData_init_zero;
  data.version = version;
  data.has_version = true;
  data.base = getMotorData(swivel);
  data.has_base = true;
  data.shoulder = getMotorData(shoulder);
  data.has_shoulder = true;
  data.elbow = getMotorData(elbow);
  data.has_elbow = true;
  serial.send(&data);
}

void calibrateAllMotors() {
	swivel.calibrate();
  shoulder.calibrate();
  elbow.calibrate();
  // gripperPosition = calibratedPosition;
}

void handleCommand(const uint8_t* buffer, int length) {
  // heartbeats.registerHeartbeat();
  auto command = BurtProto::decode<ArmCommand>(buffer, length, ArmCommand_fields);

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

  // IK control: Move to radians: 
  if (command.swivel.angle != 0) swivel.moveTo(command.swivel.angle);
  if (command.shoulder.angle != 0) shoulder.moveTo(command.shoulder.angle);
  if (command.elbow.angle != 0) elbow.moveTo(command.elbow.angle);

  // IK control to move motors by coordinates
  // Coordinates newPosition = {gripperPosition.x, gripperPosition.y, gripperPosition.z}
  // if (command.ik_x != 0) newPosition.x = command.ik_x; 
  // if (command.ik_y != 0) newPosition.y = command.ik_y;
  // if (command.ik_z != 0) newPosition.z = command.ik_z;
  // setCoordinates(newPosition);
}
