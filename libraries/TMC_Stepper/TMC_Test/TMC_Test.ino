#include "TMC_Stepper.h"
#include "TMC_Defines.h"

#define EN_PIN 23
#define CS_PIN 10
#define RUN_CURRENT 0x10
#define HOLD_CURRENT 0x10
#define HOLD_DELAY 0x15
#define VSTART  0x00005350
#define V_1     0x0000c350
#define VMAX    0x01030d40
#define VSTOP   0x00005380
#define A_1     0x000003e8
#define AMAX    0x000001e8
#define D_1     0x00000578 // NEVER ZERO
#define DMAX    0x000002bc

TMCConfig_t config = 
{
  EN_PIN,
  CS_PIN,

  Position,

  RUN_CURRENT,
  HOLD_CURRENT,
  HOLD_DELAY,

  VSTART,
  V_1,
  VMAX,
  VSTOP,

  A_1,
  AMAX,
  D_1,
  DMAX
};

TMCStepper Stepper(&config);

void setup()
{
  SPI.begin();
  Serial.begin(9600);

  Stepper.setup();
  Stepper.set_position(0xEFFFFFFF);
}
void loop()
{
  int32_t position;
  uint8_t status;
  
  Stepper.get_position_counter(position, status);
  Serial.print("Status: ");
  Serial.print(status, HEX);
  Serial.print (" Position: 0x");
  Serial.println(position, HEX);

  delay(100);
}
