#include "laser.h"
#include <Arduino.h>

Laser::Laser(int pin) : pin(pin) { }

void Laser::setup() {
  pinMode(pin, OUTPUT);
  turnOn();
}

void Laser::turnOn() {
  digitalWrite(pin, HIGH);
  isOn = true;
}

void Laser::turnOff() {
  digitalWrite(pin, LOW);
  isOn = false;
}

void Laser::handleCommand(GripperCommand command) {
  if (command.laserState == BoolState_ON) {
    turnOn();
  } else if (command.laserState == BoolState_OFF) {
    turnOff();
  }
}
