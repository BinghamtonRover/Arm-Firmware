#include <TMC_utils.h>

void setup() {
  // put your setup code here, to run once:
  TMC5160Stepper driver = newDriver(10, 23, 1700, 51200*60, 51200*8);
  driver.XTARGET(5120000);
  while(true) {
    delay(2000);
    auto xactual = driver.XACTUAL();
    auto xtarget = driver.XTARGET();
    if (xactual == xtarget) {
      driver.XTARGET(-xactual);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
