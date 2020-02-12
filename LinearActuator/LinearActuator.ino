#include <rocs.hpp>
int numSteps;
enum actuatorState {
  down,
  idle,
  up
};
actuatorState lowerState = idle;
actuatorState upperState = idle;

void setup() {
  pinMode(7,OUTPUT);
  pinMode(6,OUTPUT);
  
  pinMode(5,OUTPUT);
  pinMode(4,OUTPUT);

  pinMode(2,INPUT);
  digitalWrite(2,HIGH);
  attachInterrupt(digitalPinToInterrupt(2), magnet_detect, FALLING);

  Serial.begin(9600);

  rocs::init(0x11, "ptaArm", 6);
  rocs::set_write_handler(write_handler);
}

void loop() {
    uint8_t in;
    if (Serial.available() > 0) {
      in = Serial.read();
      write_handler(0,in);
      numSteps=0;
    }
    Serial.println(numSteps);
}

void setLowerState (actuatorState newState) {
    switch(newState) {
      case up:
        if(upperState == down) digitalWrite(7,LOW);
        delay(100);
        digitalWrite(6,HIGH);
        upperState = up;
      break;
      
      case down:
        if(upperState == up) digitalWrite(6,LOW);
        delay(100);
        digitalWrite(7,HIGH);
        upperState = down;
      break;
      
      case idle:
        digitalWrite(6,LOW);
        digitalWrite(7,LOW);
        upperState = idle;
      break;
    }
  }

  void setUpperState (actuatorState newState) { 
    switch(newState) {
      case up:
        if(lowerState == down) digitalWrite(5,LOW);
        delay(100);
        digitalWrite(4,HIGH);
        lowerState = up;
      break;
      
      case down:
        if(lowerState == up) digitalWrite(4,LOW);
        delay(100);
        digitalWrite(5,HIGH);
        lowerState = down;
      break;
      
      case idle:
        digitalWrite(4,LOW);
        digitalWrite(5,LOW);
        lowerState = idle;
      break;
    }
  }

actuatorState int2Enum (int i) {
  actuatorState ret;
  switch(i) {
    case 0: 
      ret = down;
    break;
    case 1:
      ret = idle;
    break;
    case 2:
      ret = up;
    break;
  }
  return ret;
}

void write_handler(uint8_t reg, uint8_t val) {
    actuatorState newState;
    switch (reg) {
        case 0:
          newState = int2Enum(val);
          setLowerState(newState);
        break;
        case 1:
          newState = int2Enum(val);
          setUpperState(newState);
        break;
    }
}

void magnet_detect() {
  numSteps++;
}
