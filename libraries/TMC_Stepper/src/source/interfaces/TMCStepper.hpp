
#include "../../TMCStepper.h"

/*
  Requested current = mA = I_rms/1000
  Equation for current:
  I_rms = (CS+1)/32 * V_fs/(R_sense+0.02ohm) * 1/sqrt(2)
  Solve for CS ->
  CS = 32*sqrt(2)*I_rms*(R_sense+0.02)/V_fs - 1

  Example:
  vsense = 0b0 -> V_fs = 0.325V
  mA = 1640mA = I_rms/1000 = 1.64A
  R_sense = 0.10 Ohm
  ->
  CS = 32*sqrt(2)*1.64*(0.10+0.02)/0.325 - 1 = 26.4
  CS = 26
*/
template<class T>
uint16_t TMC2130_n::TMC_RMS<T>::cs2rms(const uint8_t CS) {
  const float rs = Rsense/255.0+0.02;
  return (float)(CS+1)/32.0 * (self().vsense() ? 0.180 : 0.325)/rs / 1.41421 * 1000;
}

template<class T>
void TMC2130_n::TMC_RMS<T>::rms_current(const uint16_t mA) {
  const float rs = Rsense/255.0+0.02;
  uint8_t CS = 32.0*1.41421*mA/1000.0*rs/0.325 - 1;
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16) {
    self().vsense(true);
    CS = 32.0*1.41421*mA/1000.0*rs/0.180 - 1;
  } else { // If CS >= 16, turn off high_sense_r
    self().vsense(false);
  }

  if (CS > 31)
    CS = 31;

  typename T::IHOLD_IRUN_t r{ self().IHOLD_IRUN() };
  r.irun = CS;
  r.ihold = CS*hold_multiplier();
  self().IHOLD_IRUN(r.sr);
  //val_mA = mA;
}

/**
  TMC2300:
  Requested current = mA = I_rms/1000
  Equation for current:
  I_rms = (CS+1)/32 * V_fs/(R_sense+0.03ohm) * 1/sqrt(2)
  Solve for CS ->
  CS = 32*sqrt(2)*I_rms*(R_sense+0.03)/V_fs - 1

  Example:
  V_fs = 0.325V
  mA = 1640mA = I_rms/1000 = 1.64A
  R_sense = 0.10 Ohm
  ->
  CS = 32*sqrt(2)*1.64*(0.10+0.03)/0.325 - 1 = 28.687
  CS = 28
*/
template<class T>
uint16_t TMC2300_n::TMC_RMS<T>::cs2rms(const uint8_t CS) {
  const float rs = Rsense/255.0+0.03;
  return (CS+1.0)/32.0 * 0.325/rs / 1.41421 * 1000;
}

template<class T>
void TMC2300_n::TMC_RMS<T>::rms_current(const uint16_t mA) {
  const float rs = Rsense/255.0+0.03;
  uint8_t CS = 32.0*1.41421*mA/1000.0*rs/0.325 - 1;

  if (CS > 31)
    CS = 31;

  typename T::IHOLD_IRUN_t r{ self().IHOLD_IRUN() };
  r.irun = CS;
  r.ihold = CS*hold_multiplier();
  self().IHOLD_IRUN(r.sr);
}

template<typename TYPE>
uint8_t TMCStepper<TYPE>::test_connection() {
  const uint32_t drv_status = self().DRV_STATUS();
  switch (drv_status) {
      case 0xFFFFFFFF: return 1;
      case 0: return 2;
      default: return 0;
  }
}

/*
  TMC2160 / 5160
  Requested current = mA = I_rms/1000
  Equation for current:
  I_rms = GLOBALSCALER/256 * (CS+1)/32 * V_fs/R_sense * 1/sqrt(2)
  Solve for GLOBALSCALER ->

                 32 * 256 * sqrt(2) * I_rms * R_sense    |
  GLOBALSCALER = ------------------------------------    |
                           (CS + 1) * V_fs               | V_fs = 0.325

*/

template<class T>
void TMC2160_n::TMC_RMS<T>::rms_current(const uint16_t mA) {
  const float rs = Rsense/255.0;
  constexpr uint32_t V_fs = 325; // 0.325 * 1000
  uint8_t CS = 31;
  uint32_t scaler = 0; // = 256

  const uint16_t RS_scaled = rs * 0xFFFF; // Scale to 16b
  uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
  numerator *= RS_scaled;
  numerator >>= 8;
  numerator *= mA;

  do {
    uint32_t denominator = V_fs * 0xFFFF >> 8;
    denominator *= CS+1;
    scaler = numerator / denominator;

    if (scaler > 255) scaler = 0; // Maximum
    else if (scaler < 128) CS--;  // Try again with smaller CS
  } while(0 < scaler && scaler < 128);


  if (CS > 31)
    CS = 31;

  typename T::IHOLD_IRUN_t r{ self().IHOLD_IRUN() };
  r.irun = CS;
  r.ihold = CS*hold_multiplier();
  self().IHOLD_IRUN(r.sr);
  self().GLOBAL_SCALER(scaler);
}

template<class T>
uint16_t TMC2160_n::TMC_RMS<T>::cs2rms(const uint8_t CS) {
    const float rs = Rsense/255.0;
    uint16_t scaler = self().GLOBAL_SCALER();
    if (!scaler) scaler = 256;
    uint32_t numerator = scaler * (CS+1);
    numerator *= 325;
    numerator >>= (8+5); // Divide by 256 and 32
    numerator *= 1000000;
    uint32_t denominator = rs*1000;
    denominator *= 1414;

    return numerator / denominator;
}

template<typename TYPE>
uint8_t TMCStepper<TYPE>::microsteps2mres(const uint16_t ms) {
  uint16_t mresValue{};
  switch(ms) {
    case 256: mresValue = 0; break;
    case 128: mresValue = 1; break;
    case  64: mresValue = 2; break;
    case  32: mresValue = 3; break;
    case  16: mresValue = 4; break;
    case   8: mresValue = 5; break;
    case   4: mresValue = 6; break;
    case   2: mresValue = 7; break;
    case   0: mresValue = 8; break;
    default: return mresValue;
  }
  return mresValue;
}

template<typename TYPE>
void TMCStepper<TYPE>::microsteps(const uint16_t ms) {
  self().mres(microsteps2mres(ms));
}

template<typename TYPE>
uint16_t TMCStepper<TYPE>::mres2microsteps(const uint8_t mres) {
  switch(mres) {
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

template<typename TYPE>
uint16_t TMCStepper<TYPE>::microsteps() {
  return mres2microsteps(self().mres());
}

template<typename TYPE>
void TMCStepper<TYPE>::blank_time(const uint8_t value) {
  switch (value) {
    case 16: self().tbl(0b00); break;
    case 24: self().tbl(0b01); break;
    case 36: self().tbl(0b10); break;
    case 54: self().tbl(0b11); break;
  }
}

template<typename TYPE>
uint8_t TMCStepper<TYPE>::blank_time() {
  switch (self().tbl()) {
    case 0b00: return 16;
    case 0b01: return 24;
    case 0b10: return 36;
    case 0b11: return 54;
  }
  return 0;
}
