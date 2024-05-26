#include "../arm.pb.h"

class Laser {
  private: 
    int pin;
  public: 
    bool isOn;
    Laser(int pin);
    void setup();
    void turnOn();
    void turnOff();
    void handleCommand(GripperCommand command);
};
