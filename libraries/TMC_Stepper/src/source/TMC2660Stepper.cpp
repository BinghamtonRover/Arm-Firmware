#include "../TMCStepper.h"
#include "TMC_SPI.hpp"

using namespace TMCStepper_n;
using namespace TMC_HAL;

TMC2660Stepper::TMC2660Stepper(SPIClass &spi, PinDef cs, const float RS, const int8_t link_index) :
  TMC_SPI(spi, cs, link_index),
  Rsense(RS)
  {}

TMC2660Stepper::TMC2660Stepper(SW_SPIClass &spi, PinDef cs, const float RS, const int8_t link_index) :
  TMC_SPI(spi, cs, link_index),
  Rsense(RS)
  {}

using namespace TMC2660_n;

void TMC2660Stepper::begin() {
  TMC_SPI::begin();

  //TODO: Push shadow registers

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

bool TMC2660Stepper::isEnabled() const { return toff() > 0; }

uint8_t TMC2660Stepper::test_connection() {
  uint32_t drv_status = DRVSTATUS();
  switch (drv_status) {
      case 0xFFCFF: return 1;
      case 0: return 2;
      default: return 0;
  }
}

/*
  Requested current = mA = I_rms/1000
  Equation for current:
  I_rms = (CS+1)/32 * V_fs/R_sense * 1/sqrt(2)
  Solve for CS ->
  CS = 32*sqrt(2)*I_rms*R_sense/V_fs - 1

  Example:
  vsense = 0b0 -> V_fs = 0.310V //Typical
  mA = 1650mA = I_rms/1000 = 1.65A
  R_sense = 0.100 Ohm
  ->
  CS = 32*sqrt(2)*1.65*0.100/0.310 - 1 = 24,09
  CS = 24
*/

uint16_t TMC2660Stepper::cs2rms(const uint8_t CS) const {
  return (float)(CS+1)/32.0 * (vsense() ? 0.165 : 0.310)/(Rsense+0.02) / 1.41421 * 1000;
}

uint16_t TMC2660Stepper::rms_current() const {
  return cs2rms(cs());
}
void TMC2660Stepper::rms_current(const uint16_t mA) {
  uint8_t CS = 32.0*1.41421*mA/1000.0*Rsense/0.310 - 1;
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16) {
    vsense(true);
    CS = 32.0*1.41421*mA/1000.0*Rsense/0.165 - 1;
  } else { // If CS >= 16, turn off high_sense_r
    vsense(false);
  }

  if (CS > 31)
    CS = 31;

  cs(CS);
  //val_mA = mA;
}

void TMC2660Stepper::push() {
  DRVCTRL( sdoff() ? DRVCTRL_1_register.sr : DRVCTRL_0_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  SMARTEN(SMARTEN_register.sr);
  SGCSCONF(SGCSCONF_register.sr);
  DRVCONF(DRVCONF_register.sr);
}

void TMC2660Stepper::hysteresis_end(const int8_t value) { hend(value+3); }
int8_t TMC2660Stepper::hysteresis_end() const { return hend()-3; };

void TMC2660Stepper::hysteresis_start(const uint8_t value) { hstrt(value-1); }
uint8_t TMC2660Stepper::hysteresis_start() const { return hstrt()+1; }

void TMC2660Stepper::microsteps(const uint16_t ms) {
  switch(ms) {
    case 256: mres(0); break;
    case 128: mres(1); break;
    case  64: mres(2); break;
    case  32: mres(3); break;
    case  16: mres(4); break;
    case   8: mres(5); break;
    case   4: mres(6); break;
    case   2: mres(7); break;
    case   0: mres(8); break;
    default: break;
  }
}

uint16_t TMC2660Stepper::microsteps() {
  switch(mres()) {
    case 0: return 256;
    case 1: return 128;
    case 2: return  64;
    case 3: return  32;
    case 4: return  16;
    case 5: return   8;
    case 6: return   4;
    case 7: return   2;
    case 8: return   0;
  }
  return 0;
}

void TMC2660Stepper::blank_time(const uint8_t value) {
  switch (value) {
    case 16: tbl(0b00); break;
    case 24: tbl(0b01); break;
    case 36: tbl(0b10); break;
    case 54: tbl(0b11); break;
  }
}

uint8_t TMC2660Stepper::blank_time() const {
  switch (tbl()) {
    case 0b00: return 16;
    case 0b01: return 24;
    case 0b10: return 36;
    case 0b11: return 54;
  }
  return 0;
}

uint32_t TMC2660Stepper::CHOPCONF() const { return CHOPCONF_register.sr; }
void TMC2660Stepper::CHOPCONF(const uint32_t data) {
  CHOPCONF_register.sr = data;
  write(CHOPCONF_register.address, CHOPCONF_register.sr);
}

void TMC2660Stepper::toff(const uint8_t B)  {
  CHOPCONF_register.toff = B;
  write(CHOPCONF_register.address, CHOPCONF_register.sr);

  if (B>0)
    _savedToff = B;
}
void TMC2660Stepper::hstrt(const uint8_t B) { CHOPCONF_register.hstrt = B;  CHOPCONF(CHOPCONF_register.sr); }
void TMC2660Stepper::hend(const uint8_t B)  { CHOPCONF_register.hend = B;   CHOPCONF(CHOPCONF_register.sr); }
void TMC2660Stepper::hdec(const uint8_t B)  { CHOPCONF_register.hdec = B;   CHOPCONF(CHOPCONF_register.sr); }
void TMC2660Stepper::rndtf(const bool B)    { CHOPCONF_register.rndtf = B;  CHOPCONF(CHOPCONF_register.sr); }
void TMC2660Stepper::chm(const bool B)      { CHOPCONF_register.chm = B;    CHOPCONF(CHOPCONF_register.sr); }
void TMC2660Stepper::tbl(const uint8_t B)   { CHOPCONF_register.tbl = B;    CHOPCONF(CHOPCONF_register.sr); }

uint8_t TMC2660Stepper::toff()  const { return CHOPCONF_register.toff;  }
uint8_t TMC2660Stepper::hstrt() const { return CHOPCONF_register.hstrt; }
uint8_t TMC2660Stepper::hend()  const { return CHOPCONF_register.hend;  }
uint8_t TMC2660Stepper::hdec()  const { return CHOPCONF_register.hdec;  }
bool  TMC2660Stepper::rndtf()   const { return CHOPCONF_register.rndtf; }
bool  TMC2660Stepper::chm()     const { return CHOPCONF_register.chm;   }
uint8_t TMC2660Stepper::tbl()   const { return CHOPCONF_register.tbl;   }

uint32_t TMC2660Stepper::DRVCONF() const { return DRVCONF_register.sr; }
void TMC2660Stepper::DRVCONF(const uint32_t data) {
  DRVCONF_register.sr = data;
  write(DRVCONF_register.address, DRVCONF_register.sr);
}

void TMC2660Stepper::tst(const bool B)      { DRVCONF_register.tst = B;     DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::slph(const uint8_t B)  { DRVCONF_register.slph = B;    DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::slpl(const uint8_t B)  { DRVCONF_register.slpl = B;    DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::diss2g(const bool B)   { DRVCONF_register.diss2g = B;  DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::ts2g(const uint8_t B)  { DRVCONF_register.ts2g = B;    DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::sdoff(const bool B)    { DRVCONF_register.sdoff = B;   DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::vsense(const bool B)   { DRVCONF_register.vsense = B;  DRVCONF(DRVCONF_register.sr);}
void TMC2660Stepper::rdsel(const uint8_t B) { DRVCONF_register.rdsel = B;   DRVCONF(DRVCONF_register.sr);}

bool    TMC2660Stepper::tst()    const { return DRVCONF_register.tst;    }
uint8_t TMC2660Stepper::slph()   const { return DRVCONF_register.slph;   }
uint8_t TMC2660Stepper::slpl()   const { return DRVCONF_register.slpl;   }
bool    TMC2660Stepper::diss2g() const { return DRVCONF_register.diss2g; }
uint8_t TMC2660Stepper::ts2g()   const { return DRVCONF_register.ts2g;   }
bool    TMC2660Stepper::sdoff()  const { return DRVCONF_register.sdoff;  }
bool    TMC2660Stepper::vsense() const { return DRVCONF_register.vsense; }
uint8_t TMC2660Stepper::rdsel()  const { return DRVCONF_register.rdsel;  }

uint32_t TMC2660Stepper::DRVCTRL() const {
  if(sdoff() == 1) return DRVCTRL_1_register.sr;
  else return DRVCTRL_0_register.sr;
}
void TMC2660Stepper::DRVCTRL(const uint32_t data) {
  if(sdoff() == 1) {
    DRVCTRL_1_register.sr = data;
    write(DRVCTRL_1_register.address, DRVCTRL_1_register.sr);
  } else {
    DRVCTRL_0_register.sr = data;
    write(DRVCTRL_0_register.address, DRVCTRL_0_register.sr);
  }
}

// DRVCTRL (SPI)
void TMC2660Stepper::pha(const bool B)    { if(sdoff() == 0) return; DRVCTRL_1_register.pha = B;  DRVCTRL(DRVCTRL_1_register.sr); }
void TMC2660Stepper::ca(const uint8_t B)  { if(sdoff() == 0) return; DRVCTRL_1_register.ca = B;   DRVCTRL(DRVCTRL_1_register.sr); }
void TMC2660Stepper::phb(const bool B)    { if(sdoff() == 0) return; DRVCTRL_1_register.phb = B;  DRVCTRL(DRVCTRL_1_register.sr); }
void TMC2660Stepper::cb(const uint8_t B)  { if(sdoff() == 0) return; DRVCTRL_1_register.cb = B;   DRVCTRL(DRVCTRL_1_register.sr); }

bool TMC2660Stepper::pha()    { if(sdoff() == 0) sdoff(1); return DRVCTRL_1_register.pha; }
uint8_t TMC2660Stepper::ca()  { if(sdoff() == 0) sdoff(1); return DRVCTRL_1_register.ca;  }
bool TMC2660Stepper::phb()    { if(sdoff() == 0) sdoff(1); return DRVCTRL_1_register.phb; }
uint8_t TMC2660Stepper::cb()  { if(sdoff() == 0) sdoff(1); return DRVCTRL_1_register.cb;  }

// DRVCTRL (STEP/DIR)
void TMC2660Stepper::intpol(const bool B)  { if(sdoff()) return; DRVCTRL_0_register.intpol = B; DRVCTRL(DRVCTRL_0_register.sr); }
void TMC2660Stepper::dedge(const bool B)   { if(sdoff()) return; DRVCTRL_0_register.dedge = B;  DRVCTRL(DRVCTRL_0_register.sr); }
void TMC2660Stepper::mres(const uint8_t B) { if(sdoff()) return; DRVCTRL_0_register.mres = B;   DRVCTRL(DRVCTRL_0_register.sr); }

bool TMC2660Stepper::intpol() { if(sdoff()) sdoff(0); return DRVCTRL_0_register.intpol; }
bool TMC2660Stepper::dedge()  { if(sdoff()) sdoff(0); return DRVCTRL_0_register.dedge;  }
uint8_t TMC2660Stepper::mres(){ if(sdoff()) sdoff(0); return DRVCTRL_0_register.mres;   }

uint32_t TMC2660Stepper::DRVSTATUS() {
  uint32_t response = read()&0xFFCFF;
  READ_RDSEL00_register.sr = response & 0xFF;
  READ_RDSEL01_register.sr = response & 0xFF;
  READ_RDSEL10_register.sr = response & 0xFF;
  switch(rdsel()) {
    case 0b00: READ_RDSEL00_register.sr |= response & 0xFFC00; break;
    case 0b01: READ_RDSEL01_register.sr |= response & 0xFFC00; break;
    case 0b10: READ_RDSEL10_register.sr |= response & 0xFFC00; break;
    default: return 0;
  }
  return response;
}

uint16_t TMC2660Stepper::mstep() {
  if(rdsel() != 0b00)
    rdsel(0b00);

  DRVSTATUS();
  return READ_RDSEL00_register.mstep;
}
uint8_t TMC2660Stepper::se() {
  if(rdsel() != 0b10)
    rdsel(0b10);

  DRVSTATUS();
  return READ_RDSEL10_register.se;
}
bool TMC2660Stepper::stst() { DRVSTATUS(); return READ_RDSEL00_register.stst;  }
bool TMC2660Stepper::olb()  { DRVSTATUS(); return READ_RDSEL00_register.olb;   }
bool TMC2660Stepper::ola()  { DRVSTATUS(); return READ_RDSEL00_register.ola;   }
bool TMC2660Stepper::s2gb() { DRVSTATUS(); return READ_RDSEL00_register.s2gb;  }
bool TMC2660Stepper::s2ga() { DRVSTATUS(); return READ_RDSEL00_register.s2ga;  }
bool TMC2660Stepper::otpw() { DRVSTATUS(); return READ_RDSEL00_register.otpw;  }
bool TMC2660Stepper::ot()   { DRVSTATUS(); return READ_RDSEL00_register.ot;    }
bool TMC2660Stepper::sg()   { DRVSTATUS(); return READ_RDSEL00_register.sg_value;}

uint16_t TMC2660Stepper::sg_result(){
  uint16_t out = 0;
  if (rdsel() == 0b00) rdsel(0b01);
  DRVSTATUS();
  switch(rdsel()) {
    case 0b01: out = READ_RDSEL01_register.sg_result; break;
    case 0b10: out = READ_RDSEL10_register.sg_result; break;
    default: break;
  }
  return out;
}

uint32_t TMC2660Stepper::SGCSCONF() const { return SGCSCONF_register.sr; }
void TMC2660Stepper::SGCSCONF(const uint32_t data) {
  SGCSCONF_register.sr = data;
  write(SGCSCONF_register.address, SGCSCONF_register.sr);
}

void TMC2660Stepper::sfilt(const bool B)  { SGCSCONF_register.sfilt = B;  SGCSCONF(SGCSCONF_register.sr); }
void TMC2660Stepper::sgt(const uint8_t B) { SGCSCONF_register.sgt = B;    SGCSCONF(SGCSCONF_register.sr); }
void TMC2660Stepper::cs(const uint8_t B)  { SGCSCONF_register.cs = B;     SGCSCONF(SGCSCONF_register.sr); }

bool TMC2660Stepper::sfilt()  const { return SGCSCONF_register.sfilt; }
uint8_t TMC2660Stepper::sgt() const { return SGCSCONF_register.sgt; }
uint8_t TMC2660Stepper::cs()  const { return SGCSCONF_register.cs; }

uint32_t TMC2660Stepper::SMARTEN() const { return SMARTEN_register.sr; }
void TMC2660Stepper::SMARTEN(const uint32_t data) {
  SMARTEN_register.sr = data;
  write(SMARTEN_register.address, SMARTEN_register.sr);
}

void TMC2660Stepper::seimin(const bool B)   { SMARTEN_register.seimin = B;  SMARTEN(SMARTEN_register.sr); }
void TMC2660Stepper::sedn(const uint8_t B)  { SMARTEN_register.sedn = B;    SMARTEN(SMARTEN_register.sr); }
void TMC2660Stepper::semax(const uint8_t B) { SMARTEN_register.semax = B;   SMARTEN(SMARTEN_register.sr); }
void TMC2660Stepper::seup(const uint8_t B)  { SMARTEN_register.seup = B;    SMARTEN(SMARTEN_register.sr); }
void TMC2660Stepper::semin(const uint8_t B) { SMARTEN_register.semin = B;   SMARTEN(SMARTEN_register.sr); }

bool TMC2660Stepper::seimin()   const { return SMARTEN_register.seimin; }
uint8_t TMC2660Stepper::sedn()  const { return SMARTEN_register.sedn;   }
uint8_t TMC2660Stepper::semax() const { return SMARTEN_register.semax;  }
uint8_t TMC2660Stepper::seup()  const { return SMARTEN_register.seup;   }
uint8_t TMC2660Stepper::semin() const { return SMARTEN_register.semin;  }
