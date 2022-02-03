#ifndef tmc_utils_h
#define tmc_utils_h

#include <TMCStepper.h>
// #include <Arduino.h>

#define SW_MOSI           11 // Software Master Out Slave In (MOSI)
#define SW_MISO           12 // Software Master In Slave Out (MISO)
#define SW_SCK            13 // Software Slave Clock (SCK)
#define R_SENSE           0.075f 

TMC5160Stepper newDriver(uint8_t cs_pin, uint8_t en_pin, uint32_t current, uint32_t maxVelocity, uint32_t accel);

#endif

