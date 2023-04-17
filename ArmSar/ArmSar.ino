#include "src/utils/src/BURT_utils.h"
#include "src/tmc/src/BURT_TMC.h"

#include "src/arm.pb.h"

// HREI S1
#define CS_PIN1 10
#define EN_PIN1 35
#define LIM_PIN1 0
#define CURRENT1 2000
#define RATIO1 400

//HREI S2
#define CS_PIN2 37
#define EN_PIN2 34
#define LIM_PIN2 8
#define CURRENT2 2000
#define RATIO2 400

//HREI S3
#define CS_PIN3 36
#define EN_PIN3 40
#define LIM_PIN3 9
#define CURRENT3 2000
#define RATIO3 400

#define ACCEL 0x00030d4d
#define SPEED 0x000327e7

// 200 steps per rotation * 400:1 gear ratio * 256 microsteps per step
#define STEP_PER_360_ARM 200 * 400 * 256
#define STEP_PER_180_ARM (STEP_PER_360_ARM / 2)

#define STEP_PER_360_BASE 200 * 5 * 256
#define STEP_PER_180_BASE (STEP_PER_360_BASE / 2)

#define SHOULDER_INCREMENT 10000
#define ELBOW_INCREMENT    10000
#define SWIVEL_INCREMENT   10000

#define SHOULDER_RANGE 1000000
#define ELBOW_RANGE    1000000
#define SWIVEL_RANGE   100000

#define ARM_COMMAND_ID 0x23

#define SERIAL_MONITOR false

StepperMotor swivel(CS_PIN1, EN_PIN1, LIM_PIN1, CURRENT1, -100000, 100000, STEP_PER_180_BASE, SPEED, ACCEL, "[swivel]");
StepperMotor shoulder(CS_PIN2, EN_PIN2, LIM_PIN2, CURRENT2, -100000, 100000, STEP_PER_180_ARM, SPEED, ACCEL, "[shoulder]");
StepperMotor elbow(CS_PIN3, EN_PIN3, LIM_PIN3, CURRENT3, -100000, 100000, STEP_PER_180_ARM, SPEED, ACCEL, "[elbow]");

void handleArmCommand(const uint8_t* data, int length) {
  auto command = BurtProto::decode<ArmCommand>(data, length, ArmCommand_fields);
  if (command.stop) stop();
  shoulder.debugMoveBySteps(command.move_shoulder * SHOULDER_INCREMENT);
  swivel.debugMoveBySteps(command.move_swivel * SWIVEL_INCREMENT);
  elbow.debugMoveBySteps(command.move_elbow * ELBOW_INCREMENT);
}

BurtSerial serial(handleArmCommand);
BurtCan can(ARM_COMMAND_ID, handleArmCommand);

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing...");
  delay(1000);

  can.setup();

  swivel.presetup();
  shoulder.presetup();
  elbow.presetup();

  swivel.setup();
  shoulder.setup();
  elbow.setup();
  delay(10);

  Serial.println("Not calibrating (test script)");
  Serial.println("Arm subsystem ready");
}

void loop() {
  can.update();
  if (SERIAL_MONITOR) parseSerial();
  else serial.update();

  swivel.update();
  shoulder.update();
  elbow.update();

  delay(10);
}

void stop() {
  shoulder.stop();
  swivel.stop();
  elbow.stop();
}

void parseSerial() {
  String input = Serial.readString();
  if (input == "") return;
  if (input == "calibrate") {
    shoulder.calibrate();
    elbow.calibrate();
  }

  // Parse the command
  int delimiter = input.indexOf(" ");
  if (delimiter == -1) return;
  String part1 = input.substring(0, delimiter);
  String part2 = input.substring(delimiter + 1);
  String motor = part1;
  int steps = part2.toInt();

  // Determine which motor to move
  StepperMotor* m = &elbow;
  if (motor == "elbow") m = &elbow;
  else if (motor == "shoulder") m = &shoulder;
  else if (motor == "swivel") m = &swivel;
  else {
    Serial.print("Cannot move motor: [");
    Serial.print(motor);
    Serial.println("].");
  }

  // Move the correct motor
  m->debugMoveToStep(steps);
}
