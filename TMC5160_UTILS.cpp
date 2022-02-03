#include "TMC5160_UTILS.h"

TMC5160Stepper newDriver(uint8_t cs_pin, uint8_t en_pin, uint32_t current, uint32_t maxVelocity, uint32_t accel) {
    
    TMC5160Stepper driver = TMC5160Stepper(SPI, cs_pin, R_SENSE);
    pinMode(en_pin, OUTPUT);
    pinMode(cs_pin, OUTPUT);
    pinMode(cs_pin, HIGH);
    
    digitalWrite(en_pin, LOW);
    Serial.println("Driver begin...");
    driver.begin();
    driver.reset();
    TMC5160Stepper::IOIN_t ioin{ driver.IOIN() };
    if (ioin.version == 0xFF || ioin.version == 0) {
        Serial.print("Driver communication error with CS pin: ");
        Serial.println(cs_pin);
        while(true);
    }

    Serial.print("Driver firmware version: ");
    Serial.println(ioin.version);

    if (ioin.sd_mode) {
        Serial.println("Driver is hardware configured for Step & Dir mode, CS pin is ");
        Serial.println(cs_pin);
        while(true);
    }
    if (ioin.drv_enn) {
        Serial.println("Driver is not hardware enabled");
        while(true);
    }

    digitalWrite(en_pin, HIGH);             //disable drive to clear any start up faults
    delay(1000);                            //give the drive some time to clear faults
    digitalWrite(en_pin, LOW);              //re-enable drive, to start loading in parameters
    driver.GSTAT(7);                        //clear gstat faults

    driver.rms_current(current);

    driver.tbl(2);                          //set blanking time to 24
    driver.toff(9);
    driver.pwm_freq(1);                     //pwm at 35.1kHz
    driver.microsteps(256);
 
    driver.a1(accel);
    driver.v1(maxVelocity);
    driver.AMAX(accel);
    driver.VMAX(maxVelocity);
    driver.DMAX(accel);
    driver.d1(accel);
    driver.vstop(100);
    driver.vstart(100);
    driver.RAMPMODE(0);

    Serial.print("DRV_STATUS=0b: ");
    Serial.println(driver.DRV_STATUS(), BIN);

    return driver;
}