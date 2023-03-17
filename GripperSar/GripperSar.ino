#include "src/utils/src/BURT_utils.h"
#include "src/tmc/src/BURT_TMC.h"

#include "src/arm.pb.h"

// HREI S1: Wrist lift
#define CS_PIN1 10
#define EN_PIN1 35
#define LIM_PIN1 0 // 7
#define CURRENT1 2000  // can go up to 2000
#define RATIO1 20

//HREI S2: Wrist rotate
#define CS_PIN2 37
#define EN_PIN2 34
#define LIM_PIN2 0
#define CURRENT2 500  // can go up to 1000
#define RATIO2 100

//HREI S3: Gripper pinch
#define CS_PIN3 36
#define EN_PIN3 40
#define LIM_PIN3 0  // 9
#define CURRENT3 500  // can go up to 1000
#define RATIO3 100

#define ACCEL 0x00030d4d
#define SPEED 0x000327e7
#define SPEED2 (SPEED * 2)

// 200 steps per rotation * 400:1 gear ratio * 256 microsteps per step
#define STEP_PER_360_WRIST_LIFT 200 * RATIO1 * 256
#define STEP_PER_180_WRIST_LIFT (STEP_PER_360_WRIST_LIFT / 2)

#define STEP_PER_360_WRIST_ROTATE 200 * 5 * 256
#define STEP_PER_180_WRIST_ROTATE (STEP_PER_360_WRIST_ROTATE / 2)

#define STEP_PER_360_GRIPPER 200 * 5 * 256
#define STEP_PER_180_GRIPPER (STEP_PER_360_GRIPPER / 2)

#define LIFT_INCREMENT    5000
#define ROTATE_INCREMENT  10000
#define GRIPPER_INCREMENT 10000

#define SHOULDER_RANGE 1000000
#define ELBOW_RANGE    1000000
#define SWIVEL_RANGE   100000

#define GRIPPER_COMMAND_ID 0x33

StepperMotor wristLift(CS_PIN1, EN_PIN1, LIM_PIN1, CURRENT1, -100000, 100000, STEP_PER_180_WRIST_LIFT, SPEED, ACCEL, "[wrist lift]");
StepperMotor wristRotate(CS_PIN2, EN_PIN2, LIM_PIN2, CURRENT2, -100000, 100000, STEP_PER_180_WRIST_ROTATE, SPEED, ACCEL, "[wrist rotate]");
StepperMotor gripper(CS_PIN3, EN_PIN3, LIM_PIN3, CURRENT3, -100000, 100000, STEP_PER_180_GRIPPER, SPEED2, ACCEL, "[gripper pinch]");

BurtSerial serial(handleArmCommand);
BurtCan can(GRIPPER_COMMAND_ID, handleArmCommand);

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing...");

  can.setup();
  Serial.println("CAN initialized");
  
  wristLift.presetup();
  wristRotate.presetup();
  gripper.presetup();

  wristLift.setup();
  wristRotate.setup();
  gripper.setup();
  delay(10);

  Serial.println("Calibrating...");
  // wristLift.calibrate();
  // wristRotate.calibrate();

  Serial.println("Gripper subsystem ready");
}

void loop() {
  can.update();
  wristLift.update();
  wristRotate.update();
  gripper.update();

  serial.update();
  // parseSerial();

  delay(10);
}

void handleArmCommand(const uint8_t* data, int length) {
  // Serial.println("Received gripper command");
  auto command = BurtProto::decode<GripperCommand>(data, length, GripperCommand_fields);
  if (command.stop) {
    wristRotate.stop();
    wristLift.stop();
    gripper.stop();
  }
  wristRotate.debugMoveBySteps(command.move_rotate * ROTATE_INCREMENT);
  gripper.debugMoveBySteps(command.move_gripper * GRIPPER_INCREMENT);
  wristLift.debugMoveBySteps(command.move_lift * LIFT_INCREMENT);
}

void parseSerial() {
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
  // while (!m->isFinished()) delay(100);
}
