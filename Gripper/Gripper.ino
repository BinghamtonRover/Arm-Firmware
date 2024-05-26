#include <Servo.h>
#include "src/utils/BURT_utils.h"
#include "src/tmc/BURT_TMC.h"

#include "src/arm.pb.h"
#include "src/laser/laser.h"
#include "pinouts.h"

#define GRIPPER_COMMAND_ID 0x33
#define GRIPPER_DATA_ID 0x16
#define DATA_SEND_INTERVAL 100

void handleCommand(const uint8_t* data, int length);
void sendData();

const Version version = {major: 1, minor: 1};
Laser laser(9);
Servo camera;
int cameraAngle = 90;
float liftAngle = 0;
float rotateAngle = 0;

BurtCan<Can3> can(GRIPPER_COMMAND_ID, handleCommand);
BurtSerial serial(Device::Device_GRIPPER, handleCommand, GripperData_fields, GripperData_size);
BurtTimer dataTimer(DATA_SEND_INTERVAL, sendData);

void setup() {
	Serial.begin(9600);
	Serial.println("Initializing...");	

  Serial.print("Initializing communications...");
	can.setup();
  dataTimer.setup();
  laser.setup();
  camera.attach(18);
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
	serial.update();
  dataTimer.update();

	// Update motors
	lift.update();
	rotate.update();
	pinch.update();
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
  GripperData data = GripperData_init_zero;
  data.version = version;
  data.has_version = true;
  data.lift = getMotorData(lift);
  data.lift.angle = liftAngle;  // override raw motor data
  data.has_lift = true;
  data.rotate = getMotorData(rotate);
  data.rotate.angle = rotateAngle;  // override raw motor data
  data.has_rotate = true;
  data.pinch = getMotorData(pinch);
  data.has_pinch = true;
  data.laserState = laser.isOn ? BoolState::BoolState_ON : BoolState::BoolState_OFF;
  data.servoAngle = cameraAngle;
  serial.send(&data);
}

void calibrateAllMotors() {
	lift.calibrate();
	rotate.calibrate();
	pinch.calibrate();
  liftAngle = 0;
  rotateAngle = 0;
}

void stopAllMotors() {
  lift.stop();
  rotate.stop();
  pinch.stop();
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
    liftAngle += command.lift.move_radians / 120;
  }
  if (command.rotate.move_radians != 0) {
  	lift.moveBy(command.rotate.move_radians);
  	rotate.moveBy(command.rotate.move_radians);
    rotateAngle += command.rotate.move_radians / 150;
  }
  if (command.pinch.move_radians != 0) {
  	pinch.moveBy(command.pinch.move_radians);
  }

  // IK control: Move to radians
  if (command.lift.angle != 0) {
    lift.moveTo(-command.lift.angle);
    rotate.moveTo(command.lift.angle);
    liftAngle = command.lift.angle;
  }
  if (command.rotate.angle != 0) {
    lift.moveTo(command.rotate.angle);
    rotate.moveTo(command.rotate.angle);
    rotateAngle = command.rotate.angle;
  }
  if (command.pinch.angle != 0) {
    pinch.moveTo(command.rotate.angle);
  }

  if (command.spin) {
    lift.moveBy(2 * PI);
    rotate.moveBy(2 * PI);
  }
  if (command.servoAngle != 0) camera.write(command.servoAngle);
  laser.handleCommand(command);
}
