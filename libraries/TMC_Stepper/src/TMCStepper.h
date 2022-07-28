#pragma once

//#define TMCDEBUG

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "source/TMC_HAL.h"

#pragma GCC diagnostic pop

#include "source/TMC2660_bitfields.h"

#include "source/interfaces/TMC2130.hpp"
#include "source/interfaces/TMC2160.hpp"
#include "source/interfaces/TMC5130.hpp"
#include "source/interfaces/TMC5160.hpp"
#include "source/interfaces/TMC2208.hpp"
#include "source/interfaces/TMC2209.hpp"
#include "source/interfaces/TMC2300.hpp"

#define TMCSTEPPER_VERSION 0x000703 // v0.7.3

struct TMC2130Stepper;
struct TMC2160Stepper;
struct TMC5130Stepper;
struct TMC5160Stepper;
struct TMC2208Stepper;
struct TMC2209Stepper;
struct TMC2300Stepper;

#include "source/TMC_SPI.hpp"
#include "source/TMC_UART.hpp"

template<typename TYPE>
class TMCStepper {
	public:
		uint8_t test_connection();

		// Helper functions
		uint8_t microsteps2mres(const uint16_t ms);
		uint16_t mres2microsteps(const uint8_t mres);
		void microsteps(const uint16_t ms);
		uint16_t microsteps();
		void blank_time(const uint8_t value);
		uint8_t blank_time();

		void hysteresis_end(const int8_t value) {
			self().hend(value+3);
		}

		int8_t hysteresis_end() {
			return self().hend()-3;
		}

		void hysteresis_start(const uint8_t value) {
			self().hstrt(value-1);
		}

		uint8_t hysteresis_start() {
			return self().hstrt()+1;
		}

	private:
		TYPE& self() { return *static_cast<TYPE*>(this); } 
};

namespace TMC2130_n {

template<class T>
struct TMC_RMS {
    uint16_t cs2rms(const uint8_t CS);
    void rms_current(const uint16_t mA);
    void rms_current(const uint16_t mA, const float mult) {
      hold_multiplier(mult);
      rms_current(mA);
    }
    uint16_t rms_current() {
      return cs2rms(self().irun());
    }
    void hold_multiplier(const float val) { holdMultiplier = val*255; }
    float hold_multiplier() const { return (holdMultiplier+0.5)/255.0; }
  protected:
    TMC_RMS(const float RS) : Rsense(RS*255) {};
    T& self() { return *static_cast<T*>(this); }

    const uint8_t Rsense;
    uint8_t holdMultiplier = 127;
};

}

namespace TMC2160_n {

template<class T>
struct TMC_RMS {
    uint16_t cs2rms(const uint8_t CS);
    void rms_current(const uint16_t mA);
    void rms_current(const uint16_t mA, float mult) {
      hold_multiplier(mult);
      rms_current(mA);
    }
    uint16_t rms_current() {
      return cs2rms(self().irun());
    }
    void hold_multiplier(const float val) { holdMultiplier = val*255; }
    float hold_multiplier() const { return (holdMultiplier+0.5)/255.0; }
  protected:
    TMC_RMS(const float RS) : Rsense(RS*255) {};
    T& self() { return *static_cast<T*>(this); }

    const uint8_t Rsense;
    uint8_t holdMultiplier = 127;
};

}

namespace TMC2300_n {

template<class T>
struct TMC_RMS {
    uint16_t cs2rms(const uint8_t CS);
    void rms_current(const uint16_t mA);
    void rms_current(const uint16_t mA, float mult) {
      hold_multiplier(mult);
      rms_current(mA);
    }
    uint16_t rms_current() {
      return cs2rms(self().irun());
    }
    void hold_multiplier(const float val) { holdMultiplier = val*255; }
    float hold_multiplier() const { return (holdMultiplier+0.5)/255.0; }
  protected:
    TMC_RMS(const float RS) : Rsense(RS*255) {};
    T& self() { return *static_cast<T*>(this); }

    const uint8_t Rsense;
    uint8_t holdMultiplier = 127;
};

}

namespace TMC5130_n { using TMC2130_n::TMC_RMS; }
namespace TMC5160_n { using TMC2160_n::TMC_RMS; }
namespace TMC2208_n { using TMC2130_n::TMC_RMS; }
namespace TMC2209_n { using TMC2130_n::TMC_RMS; }

#include "source/interfaces/TMCStepper.hpp"

class TMC2130Stepper :
	public TMCStepper_n::TMC_SPI,
	public TMCStepper<TMC2130Stepper>,
	public TMC2130_n::TMC_RMS<TMC2130Stepper>,
	public TMC2130_n::GCONF_i<TMC2130Stepper>,
	public TMC2130_n::GSTAT_i<TMC2130Stepper>,
	public TMC2130_n::IOIN_i<TMC2130Stepper>,
	public TMC2130_n::IHOLD_IRUN_i<TMC2130Stepper>,
	public TMC2130_n::TPOWERDOWN_i<TMC2130Stepper>,
	public TMC2130_n::TSTEP_i<TMC2130Stepper>,
	public TMC2130_n::TPWMTHRS_i<TMC2130Stepper>,
	public TMC2130_n::TCOOLTHRS_i<TMC2130Stepper>,
	public TMC2130_n::THIGH_i<TMC2130Stepper>,
	public TMC2130_n::XDIRECT_i<TMC2130Stepper>,
	public TMC2130_n::VDCMIN_i<TMC2130Stepper>,
	public TMC2130_n::MSLUTSEL_i<TMC2130Stepper>,
	public TMC2130_n::MSLUTSTART_i<TMC2130Stepper>,
	public TMC2130_n::MSCNT_i<TMC2130Stepper>,
	public TMC2130_n::MSCURACT_i<TMC2130Stepper>,
	public TMC2130_n::CHOPCONF_i<TMC2130Stepper>,
	public TMC2130_n::COOLCONF_i<TMC2130Stepper>,
	public TMC2130_n::DCCTRL_i<TMC2130Stepper>,
	public TMC2130_n::DRV_STATUS_i<TMC2130Stepper>,
	public TMC2130_n::PWMCONF_i<TMC2130Stepper>,
	public TMC2130_n::PWM_SCALE_i<TMC2130Stepper>,
	public TMC2130_n::ENCM_CTRL_i<TMC2130Stepper>,
	public TMC2130_n::LOST_STEPS_i<TMC2130Stepper>
	{
	public:
		TMC2130Stepper(SPIClass &spi, TMC_HAL::PinDef cs, const float RS, const int8_t link_index = -1);
		TMC2130Stepper(SW_SPIClass &spi, TMC_HAL::PinDef cs, const float RS, const int8_t link_index = -1);

		void begin();
		void defaults();
		void resetLibCache();
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::drv_enn_cfg6(); }
		void push();

		// Helper functions
		void sg_current_decrease(const uint8_t value);
		uint8_t sg_current_decrease();

		// Can be accessed for a common default value
		static constexpr float default_RS = 0.11;

		// Deleted functions
		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC2130Stepper(TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, const int8_t link_index = -1) = delete;

		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC2130Stepper(TMC_HAL::PinDef) = delete;

		using GCONF_t           = TMC2130_n::GCONF_t;
		using GSTAT_t           = TMC2130_n::GSTAT_t;
		using IOIN_t            = TMC2130_n::IOIN_t;
		using IHOLD_IRUN_t      = TMC2130_n::IHOLD_IRUN_t;
		using TPOWERDOWN_t      = TMC2130_n::TPOWERDOWN_t;
		using TSTEP_t           = TMC2130_n::TSTEP_t;
		using TPWMTHRS_t        = TMC2130_n::TPWMTHRS_t;
		using TCOOLTHRS_t       = TMC2130_n::TCOOLTHRS_t;
		using THIGH_t           = TMC2130_n::THIGH_t;
		using XDIRECT_t         = TMC2130_n::XDIRECT_t;
		using VDCMIN_t          = TMC2130_n::VDCMIN_t;
		using MSLUT0_t          = TMC2130_n::MSLUT0_t;
		using MSLUT1_t          = TMC2130_n::MSLUT1_t;
		using MSLUT2_t          = TMC2130_n::MSLUT2_t;
		using MSLUT3_t          = TMC2130_n::MSLUT3_t;
		using MSLUT4_t          = TMC2130_n::MSLUT4_t;
		using MSLUT5_t          = TMC2130_n::MSLUT5_t;
		using MSLUT6_t          = TMC2130_n::MSLUT6_t;
		using MSLUT7_t          = TMC2130_n::MSLUT7_t;
		using MSLUTSEL_t        = TMC2130_n::MSLUTSEL_t;
		using MSLUTSTART_t      = TMC2130_n::MSLUTSTART_t;
		using MSCNT_t           = TMC2130_n::MSCNT_t;
		using MSCURACT_t        = TMC2130_n::MSCURACT_t;
		using CHOPCONF_t        = TMC2130_n::CHOPCONF_t;
		using COOLCONF_t        = TMC2130_n::COOLCONF_t;
		using DCCTRL_t          = TMC2130_n::DCCTRL_t;
		using DRV_STATUS_t      = TMC2130_n::DRV_STATUS_t;
		using PWMCONF_t         = TMC2130_n::PWMCONF_t;
		using PWM_SCALE_t       = TMC2130_n::PWM_SCALE_t;
		using ENCM_CTRL_t       = TMC2130_n::ENCM_CTRL_t;
		using LOST_STEPS_t      = TMC2130_n::LOST_STEPS_t;
};

class TMC2160Stepper :
	public TMCStepper_n::TMC_SPI,
	public TMCStepper<TMC2160Stepper>,
	public TMC2160_n::TMC_RMS<TMC2160Stepper>,
	public TMC2160_n::GCONF_i<TMC2160Stepper>,
	public TMC2160_n::GSTAT_i<TMC2160Stepper>,
	public TMC2160_n::IOIN_i<TMC2160Stepper>,
	public TMC2160_n::IHOLD_IRUN_i<TMC2160Stepper>,
	public TMC2160_n::TPOWERDOWN_i<TMC2160Stepper>,
	public TMC2160_n::TSTEP_i<TMC2160Stepper>,
	public TMC2160_n::TPWMTHRS_i<TMC2160Stepper>,
	public TMC2160_n::TCOOLTHRS_i<TMC2160Stepper>,
	public TMC2160_n::THIGH_i<TMC2160Stepper>,
	public TMC2160_n::XDIRECT_i<TMC2160Stepper>,
	public TMC2160_n::VDCMIN_i<TMC2160Stepper>,
	public TMC2160_n::MSLUTSEL_i<TMC2160Stepper>,
	public TMC2160_n::MSLUTSTART_i<TMC2160Stepper>,
	public TMC2160_n::MSCNT_i<TMC2160Stepper>,
	public TMC2160_n::MSCURACT_i<TMC2160Stepper>,
	public TMC2160_n::CHOPCONF_i<TMC2160Stepper>,
	public TMC2160_n::COOLCONF_i<TMC2160Stepper>,
	public TMC2160_n::DCCTRL_i<TMC2160Stepper>,
	public TMC2160_n::DRV_STATUS_i<TMC2160Stepper>,
	public TMC2160_n::PWMCONF_i<TMC2160Stepper>,
	public TMC2160_n::ENCM_CTRL_i<TMC2160Stepper>,
	public TMC2160_n::LOST_STEPS_i<TMC2160Stepper>,
	public TMC2160_n::SHORT_CONF_i<TMC2160Stepper>,
	public TMC2160_n::DRV_CONF_i<TMC2160Stepper>,
	public TMC2160_n::GLOBAL_SCALER_i<TMC2160Stepper>,
	public TMC2160_n::OFFSET_READ_i<TMC2160Stepper>,
	public TMC2160_n::PWM_SCALE_i<TMC2160Stepper>
	{
	public:
		TMC2160Stepper(SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		TMC2160Stepper(SW_SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		void begin();
		void defaults();
		void resetLibCache();
		void push();
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::drv_enn(); }

		// Deleted functions
		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC2160Stepper(TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, const int8_t link_index = -1) = delete;

		static constexpr float default_RS = 0.075;

		using GCONF_t        	= TMC2160_n::GCONF_t;
		using GSTAT_t        	= TMC2160_n::GSTAT_t;
		using IOIN_t         	= TMC2160_n::IOIN_t;
		using IHOLD_IRUN_t   	= TMC2160_n::IHOLD_IRUN_t;
		using TPOWERDOWN_t   	= TMC2160_n::TPOWERDOWN_t;
		using TSTEP_t        	= TMC2160_n::TSTEP_t;
		using TPWMTHRS_t     	= TMC2160_n::TPWMTHRS_t;
		using TCOOLTHRS_t    	= TMC2160_n::TCOOLTHRS_t;
		using THIGH_t        	= TMC2160_n::THIGH_t;
		using XDIRECT_t      	= TMC2160_n::XDIRECT_t;
		using VDCMIN_t       	= TMC2160_n::VDCMIN_t;
		using MSLUTSEL_t     	= TMC2160_n::MSLUTSEL_t;
		using MSLUTSTART_t   	= TMC2160_n::MSLUTSTART_t;
		using MSCNT_t        	= TMC2160_n::MSCNT_t;
		using MSCURACT_t     	= TMC2160_n::MSCURACT_t;
		using CHOPCONF_t     	= TMC2160_n::CHOPCONF_t;
		using COOLCONF_t     	= TMC2160_n::COOLCONF_t;
		using DCCTRL_t       	= TMC2160_n::DCCTRL_t;
		using DRV_STATUS_t   	= TMC2160_n::DRV_STATUS_t;
		using PWMCONF_t      	= TMC2160_n::PWMCONF_t;
		using ENCM_CTRL_t    	= TMC2160_n::ENCM_CTRL_t;
		using LOST_STEPS_t   	= TMC2160_n::LOST_STEPS_t;
		using SHORT_CONF_t   	= TMC2160_n::SHORT_CONF_t;
		using DRV_CONF_t     	= TMC2160_n::DRV_CONF_t;
		using GLOBAL_SCALER_t	= TMC2160_n::GLOBAL_SCALER_t;
		using OFFSET_READ_t  	= TMC2160_n::OFFSET_READ_t;
		using PWM_SCALE_t    	= TMC2160_n::PWM_SCALE_t;
};

class TMC5130Stepper :
	public TMCStepper_n::TMC_SPI,
	public TMCStepper<TMC5130Stepper>,
	public TMC5130_n::TMC_RMS		  <TMC5130Stepper>,
	public TMC5130_n::GCONF_i         <TMC5130Stepper>,
	public TMC5130_n::GSTAT_i         <TMC5130Stepper>,
	public TMC5130_n::IFCNT_i         <TMC5130Stepper>,
	public TMC5130_n::SLAVECONF_i     <TMC5130Stepper>,
	public TMC5130_n::IOIN_i          <TMC5130Stepper>,
	public TMC5130_n::OUTPUT_i        <TMC5130Stepper>,
	public TMC5130_n::X_COMPARE_i     <TMC5130Stepper>,
	public TMC5130_n::IHOLD_IRUN_i    <TMC5130Stepper>,
	public TMC5130_n::TPOWERDOWN_i    <TMC5130Stepper>,
	public TMC5130_n::TSTEP_i         <TMC5130Stepper>,
	public TMC5130_n::TPWMTHRS_i      <TMC5130Stepper>,
	public TMC5130_n::TCOOLTHRS_i     <TMC5130Stepper>,
	public TMC5130_n::THIGH_i         <TMC5130Stepper>,
	public TMC5130_n::RAMPMODE_i      <TMC5130Stepper>,
	public TMC5130_n::XACTUAL_i       <TMC5130Stepper>,
	public TMC5130_n::VACTUAL_i       <TMC5130Stepper>,
	public TMC5130_n::VSTART_i        <TMC5130Stepper>,
	public TMC5130_n::A1_i            <TMC5130Stepper>,
	public TMC5130_n::V1_i            <TMC5130Stepper>,
	public TMC5130_n::AMAX_i          <TMC5130Stepper>,
	public TMC5130_n::VMAX_i          <TMC5130Stepper>,
	public TMC5130_n::DMAX_i          <TMC5130Stepper>,
	public TMC5130_n::D1_i            <TMC5130Stepper>,
	public TMC5130_n::VSTOP_i         <TMC5130Stepper>,
	public TMC5130_n::TZEROWAIT_i     <TMC5130Stepper>,
	public TMC5130_n::XTARGET_i       <TMC5130Stepper>,
	public TMC5130_n::VDCMIN_i        <TMC5130Stepper>,
	public TMC5130_n::SW_MODE_i       <TMC5130Stepper>,
	public TMC5130_n::RAMP_STAT_i     <TMC5130Stepper>,
	public TMC5130_n::XLATCH_i        <TMC5130Stepper>,
	public TMC5130_n::ENCMODE_i       <TMC5130Stepper>,
	public TMC5130_n::X_ENC_i         <TMC5130Stepper>,
	public TMC5130_n::ENC_CONST_i     <TMC5130Stepper>,
	public TMC5130_n::ENC_STATUS_i    <TMC5130Stepper>,
	public TMC5130_n::ENC_LATCH_i     <TMC5130Stepper>,
	public TMC5130_n::MSLUT0_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT1_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT2_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT3_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT4_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT5_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT6_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUT7_i        <TMC5130Stepper>,
	public TMC5130_n::MSLUTSEL_i      <TMC5130Stepper>,
	public TMC5130_n::MSLUTSTART_i    <TMC5130Stepper>,
	public TMC5130_n::MSCNT_i         <TMC5130Stepper>,
	public TMC5130_n::MSCURACT_i      <TMC5130Stepper>,
	public TMC5130_n::CHOPCONF_i      <TMC5130Stepper>,
	public TMC5130_n::COOLCONF_i      <TMC5130Stepper>,
	public TMC5130_n::DCCTRL_i        <TMC5130Stepper>,
	public TMC5130_n::DRV_STATUS_i    <TMC5130Stepper>,
	public TMC5130_n::PWMCONF_i       <TMC5130Stepper>,
	public TMC5130_n::PWM_SCALE_i     <TMC5130Stepper>,
	public TMC5130_n::ENCM_CTRL_i     <TMC5130Stepper>,
	public TMC5130_n::LOST_STEPS_i    <TMC5130Stepper>
	{
	public:
		TMC5130Stepper(SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		TMC5130Stepper(SW_SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);

		void begin();
		void defaults();
		void resetLibCache();
		void push();
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::drv_enn_cfg6(); }

		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC5130Stepper(TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, const int8_t link_index = -1) = delete;

		static constexpr float default_RS = 0.15;

		using GCONF_t     	= TMC5130_n::GCONF_t;
		using GSTAT_t     	= TMC5130_n::GSTAT_t;
		using IFCNT_t     	= TMC5130_n::IFCNT_t;
		using SLAVECONF_t 	= TMC5130_n::SLAVECONF_t;
		using IOIN_t      	= TMC5130_n::IOIN_t;
		using OUTPUT_t    	= TMC5130_n::OUTPUT_t;
		using X_COMPARE_t 	= TMC5130_n::X_COMPARE_t;
		using IHOLD_IRUN_t	= TMC5130_n::IHOLD_IRUN_t;
		using TPOWERDOWN_t	= TMC5130_n::TPOWERDOWN_t;
		using TSTEP_t     	= TMC5130_n::TSTEP_t;
		using TPWMTHRS_t  	= TMC5130_n::TPWMTHRS_t;
		using TCOOLTHRS_t 	= TMC5130_n::TCOOLTHRS_t;
		using THIGH_t     	= TMC5130_n::THIGH_t;
		using RAMPMODE_t  	= TMC5130_n::RAMPMODE_t;
		using XACTUAL_t   	= TMC5130_n::XACTUAL_t;
		using VACTUAL_t   	= TMC5130_n::VACTUAL_t;
		using VSTART_t    	= TMC5130_n::VSTART_t;
		using A1_t        	= TMC5130_n::A1_t;
		using V1_t        	= TMC5130_n::V1_t;
		using AMAX_t      	= TMC5130_n::AMAX_t;
		using VMAX_t      	= TMC5130_n::VMAX_t;
		using DMAX_t      	= TMC5130_n::DMAX_t;
		using D1_t        	= TMC5130_n::D1_t;
		using VSTOP_t     	= TMC5130_n::VSTOP_t;
		using TZEROWAIT_t 	= TMC5130_n::TZEROWAIT_t;
		using XTARGET_t   	= TMC5130_n::XTARGET_t;
		using VDCMIN_t    	= TMC5130_n::VDCMIN_t;
		using SW_MODE_t   	= TMC5130_n::SW_MODE_t;
		using RAMP_STAT_t 	= TMC5130_n::RAMP_STAT_t;
		using XLATCH_t    	= TMC5130_n::XLATCH_t;
		using ENCMODE_t   	= TMC5130_n::ENCMODE_t;
		using X_ENC_t     	= TMC5130_n::X_ENC_t;
		using ENC_CONST_t 	= TMC5130_n::ENC_CONST_t;
		using ENC_STATUS_t	= TMC5130_n::ENC_STATUS_t;
		using ENC_LATCH_t 	= TMC5130_n::ENC_LATCH_t;
		using MSLUT0_t    	= TMC5130_n::MSLUT0_t;
		using MSLUT1_t    	= TMC5130_n::MSLUT1_t;
		using MSLUT2_t    	= TMC5130_n::MSLUT2_t;
		using MSLUT3_t    	= TMC5130_n::MSLUT3_t;
		using MSLUT4_t    	= TMC5130_n::MSLUT4_t;
		using MSLUT5_t    	= TMC5130_n::MSLUT5_t;
		using MSLUT6_t    	= TMC5130_n::MSLUT6_t;
		using MSLUT7_t    	= TMC5130_n::MSLUT7_t;
		using MSLUTSEL_t  	= TMC5130_n::MSLUTSEL_t;
		using MSLUTSTART_t	= TMC5130_n::MSLUTSTART_t;
		using MSCNT_t     	= TMC5130_n::MSCNT_t;
		using MSCURACT_t  	= TMC5130_n::MSCURACT_t;
		using CHOPCONF_t  	= TMC5130_n::CHOPCONF_t;
		using COOLCONF_t  	= TMC5130_n::COOLCONF_t;
		using DCCTRL_t    	= TMC5130_n::DCCTRL_t;
		using DRV_STATUS_t	= TMC5130_n::DRV_STATUS_t;
		using PWMCONF_t   	= TMC5130_n::PWMCONF_t;
		using PWM_SCALE_t 	= TMC5130_n::PWM_SCALE_t;
		using ENCM_CTRL_t 	= TMC5130_n::ENCM_CTRL_t;
		using LOST_STEPS_t	= TMC5130_n::LOST_STEPS_t;
	};

class TMC5160Stepper :
	public TMCStepper_n::TMC_SPI,
	public TMCStepper<TMC5160Stepper>,
	public TMC5160_n::TMC_RMS			 <TMC5160Stepper>,
	public TMC5160_n::GCONF_i            <TMC5160Stepper>,
	public TMC5160_n::GSTAT_i            <TMC5160Stepper>,
	public TMC5160_n::IFCNT_i            <TMC5160Stepper>,
	public TMC5160_n::SLAVECONF_i        <TMC5160Stepper>,
	public TMC5160_n::IOIN_i             <TMC5160Stepper>,
	public TMC5160_n::OUTPUT_i           <TMC5160Stepper>,
	public TMC5160_n::X_COMPARE_i        <TMC5160Stepper>,
	public TMC5160_n::OTP_PROG_i         <TMC5160Stepper>,
	public TMC5160_n::OTP_READ_i         <TMC5160Stepper>,
	public TMC5160_n::FACTORY_CONF_i     <TMC5160Stepper>,
	public TMC5160_n::SHORT_CONF_i       <TMC5160Stepper>,
	public TMC5160_n::DRV_CONF_i         <TMC5160Stepper>,
	public TMC5160_n::GLOBAL_SCALER_i    <TMC5160Stepper>,
	public TMC5160_n::OFFSET_READ_i      <TMC5160Stepper>,
	public TMC5160_n::IHOLD_IRUN_i       <TMC5160Stepper>,
	public TMC5160_n::TPOWERDOWN_i       <TMC5160Stepper>,
	public TMC5160_n::TSTEP_i            <TMC5160Stepper>,
	public TMC5160_n::TPWMTHRS_i         <TMC5160Stepper>,
	public TMC5160_n::TCOOLTHRS_i        <TMC5160Stepper>,
	public TMC5160_n::THIGH_i            <TMC5160Stepper>,
	public TMC5160_n::RAMPMODE_i         <TMC5160Stepper>,
	public TMC5160_n::XACTUAL_i          <TMC5160Stepper>,
	public TMC5160_n::VACTUAL_i          <TMC5160Stepper>,
	public TMC5160_n::VSTART_i           <TMC5160Stepper>,
	public TMC5160_n::A1_i               <TMC5160Stepper>,
	public TMC5160_n::V1_i               <TMC5160Stepper>,
	public TMC5160_n::AMAX_i             <TMC5160Stepper>,
	public TMC5160_n::VMAX_i             <TMC5160Stepper>,
	public TMC5160_n::DMAX_i             <TMC5160Stepper>,
	public TMC5160_n::D1_i               <TMC5160Stepper>,
	public TMC5160_n::VSTOP_i            <TMC5160Stepper>,
	public TMC5160_n::TZEROWAIT_i        <TMC5160Stepper>,
	public TMC5160_n::XTARGET_i          <TMC5160Stepper>,
	public TMC5160_n::VDCMIN_i           <TMC5160Stepper>,
	public TMC5160_n::SW_MODE_i          <TMC5160Stepper>,
	public TMC5160_n::RAMP_STAT_i        <TMC5160Stepper>,
	public TMC5160_n::XLATCH_i           <TMC5160Stepper>,
	public TMC5160_n::ENCMODE_i          <TMC5160Stepper>,
	public TMC5160_n::X_ENC_i            <TMC5160Stepper>,
	public TMC5160_n::ENC_CONST_i        <TMC5160Stepper>,
	public TMC5160_n::ENC_STATUS_i       <TMC5160Stepper>,
	public TMC5160_n::ENC_LATCH_i        <TMC5160Stepper>,
	public TMC5160_n::ENC_DEVIATION_i    <TMC5160Stepper>,
	public TMC5160_n::MSLUT0_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT1_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT2_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT3_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT4_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT5_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT6_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUT7_i           <TMC5160Stepper>,
	public TMC5160_n::MSLUTSEL_i         <TMC5160Stepper>,
	public TMC5160_n::MSLUTSTART_i       <TMC5160Stepper>,
	public TMC5160_n::MSCNT_i            <TMC5160Stepper>,
	public TMC5160_n::MSCURACT_i         <TMC5160Stepper>,
	public TMC5160_n::CHOPCONF_i         <TMC5160Stepper>,
	public TMC5160_n::COOLCONF_i         <TMC5160Stepper>,
	public TMC5160_n::DCCTRL_i           <TMC5160Stepper>,
	public TMC5160_n::DRV_STATUS_i       <TMC5160Stepper>,
	public TMC5160_n::PWMCONF_i          <TMC5160Stepper>,
	public TMC5160_n::PWM_SCALE_i        <TMC5160Stepper>,
	public TMC5160_n::PWM_AUTO_i         <TMC5160Stepper>,
	public TMC5160_n::LOST_STEPS_i       <TMC5160Stepper>
	{
	public:
		TMC5160Stepper(SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		TMC5160Stepper(SW_SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);

		void begin();
		void defaults();
		void resetLibCache();
		void push();
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::drv_enn(); }

		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC5160Stepper(TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, TMC_HAL::PinDef, const int8_t link_index = -1) = delete;

		static constexpr float default_RS = 0.075;

        using GCONF_t        	= TMC5160_n::GCONF_t;
        using GSTAT_t        	= TMC5160_n::GSTAT_t;
        using IFCNT_t        	= TMC5160_n::IFCNT_t;
        using SLAVECONF_t    	= TMC5160_n::SLAVECONF_t;
        using IOIN_t         	= TMC5160_n::IOIN_t;
        using OUTPUT_t       	= TMC5160_n::OUTPUT_t;
        using X_COMPARE_t    	= TMC5160_n::X_COMPARE_t;
        using OTP_PROG_t     	= TMC5160_n::OTP_PROG_t;
        using OTP_READ_t     	= TMC5160_n::OTP_READ_t;
        using FACTORY_CONF_t 	= TMC5160_n::FACTORY_CONF_t;
        using SHORT_CONF_t   	= TMC5160_n::SHORT_CONF_t;
        using DRV_CONF_t     	= TMC5160_n::DRV_CONF_t;
        using GLOBAL_SCALER_t	= TMC5160_n::GLOBAL_SCALER_t;
        using OFFSET_READ_t  	= TMC5160_n::OFFSET_READ_t;
        using IHOLD_IRUN_t   	= TMC5160_n::IHOLD_IRUN_t;
        using TPOWERDOWN_t   	= TMC5160_n::TPOWERDOWN_t;
        using TSTEP_t        	= TMC5160_n::TSTEP_t;
        using TPWMTHRS_t     	= TMC5160_n::TPWMTHRS_t;
        using TCOOLTHRS_t    	= TMC5160_n::TCOOLTHRS_t;
        using THIGH_t        	= TMC5160_n::THIGH_t;
        using RAMPMODE_t     	= TMC5160_n::RAMPMODE_t;
        using XACTUAL_t      	= TMC5160_n::XACTUAL_t;
        using VACTUAL_t      	= TMC5160_n::VACTUAL_t;
        using VSTART_t       	= TMC5160_n::VSTART_t;
        using A1_t           	= TMC5160_n::A1_t;
        using V1_t           	= TMC5160_n::V1_t;
        using AMAX_t         	= TMC5160_n::AMAX_t;
        using VMAX_t         	= TMC5160_n::VMAX_t;
        using DMAX_t         	= TMC5160_n::DMAX_t;
        using D1_t           	= TMC5160_n::D1_t;
        using VSTOP_t        	= TMC5160_n::VSTOP_t;
        using TZEROWAIT_t    	= TMC5160_n::TZEROWAIT_t;
        using XTARGET_t      	= TMC5160_n::XTARGET_t;
        using VDCMIN_t       	= TMC5160_n::VDCMIN_t;
        using SW_MODE_t      	= TMC5160_n::SW_MODE_t;
        using RAMP_STAT_t    	= TMC5160_n::RAMP_STAT_t;
        using XLATCH_t       	= TMC5160_n::XLATCH_t;
        using ENCMODE_t      	= TMC5160_n::ENCMODE_t;
        using X_ENC_t        	= TMC5160_n::X_ENC_t;
        using ENC_CONST_t    	= TMC5160_n::ENC_CONST_t;
        using ENC_STATUS_t   	= TMC5160_n::ENC_STATUS_t;
        using ENC_LATCH_t    	= TMC5160_n::ENC_LATCH_t;
        using ENC_DEVIATION_t	= TMC5160_n::ENC_DEVIATION_t;
        using MSLUT0_t       	= TMC5160_n::MSLUT0_t;
        using MSLUT1_t       	= TMC5160_n::MSLUT1_t;
        using MSLUT2_t       	= TMC5160_n::MSLUT2_t;
        using MSLUT3_t       	= TMC5160_n::MSLUT3_t;
        using MSLUT4_t       	= TMC5160_n::MSLUT4_t;
        using MSLUT5_t       	= TMC5160_n::MSLUT5_t;
        using MSLUT6_t       	= TMC5160_n::MSLUT6_t;
        using MSLUT7_t       	= TMC5160_n::MSLUT7_t;
        using MSLUTSEL_t     	= TMC5160_n::MSLUTSEL_t;
        using MSLUTSTART_t   	= TMC5160_n::MSLUTSTART_t;
        using MSCNT_t        	= TMC5160_n::MSCNT_t;
        using MSCURACT_t     	= TMC5160_n::MSCURACT_t;
        using CHOPCONF_t     	= TMC5160_n::CHOPCONF_t;
        using COOLCONF_t     	= TMC5160_n::COOLCONF_t;
        using DCCTRL_t       	= TMC5160_n::DCCTRL_t;
        using DRV_STATUS_t   	= TMC5160_n::DRV_STATUS_t;
        using PWMCONF_t      	= TMC5160_n::PWMCONF_t;
        using PWM_SCALE_t    	= TMC5160_n::PWM_SCALE_t;
        using PWM_AUTO_t     	= TMC5160_n::PWM_AUTO_t;
        using LOST_STEPS_t   	= TMC5160_n::LOST_STEPS_t;
};

typedef TMC5160Stepper TMC5161Stepper;

class TMC2208Stepper :
	public TMC_HAL::TMC_UART,
	public TMCStepper<TMC2208Stepper>,
	public TMC2208_n::TMC_RMS<TMC2208Stepper>,
	public TMC2208_n::GCONF_i<TMC2208Stepper>,
	public TMC2208_n::GSTAT_i<TMC2208Stepper>,
	public TMC2208_n::IFCNT_i<TMC2208Stepper>,
	public TMC2208_n::SLAVECONF_i<TMC2208Stepper>,
	public TMC2208_n::OTP_PROG_i<TMC2208Stepper>,
	public TMC2208_n::OTP_READ_i<TMC2208Stepper>,
	public TMC2208_n::IOIN_i<TMC2208Stepper>,
	public TMC2208_n::FACTORY_CONF_i<TMC2208Stepper>,
	public TMC2208_n::IHOLD_IRUN_i<TMC2208Stepper>,
	public TMC2208_n::TPOWERDOWN_i<TMC2208Stepper>,
	public TMC2208_n::TSTEP_i<TMC2208Stepper>,
	public TMC2208_n::TPWMTHRS_i<TMC2208Stepper>,
	public TMC2208_n::VACTUAL_i<TMC2208Stepper>,
	public TMC2208_n::MSCNT_i<TMC2208Stepper>,
	public TMC2208_n::MSCURACT_i<TMC2208Stepper>,
	public TMC2208_n::CHOPCONF_i<TMC2208Stepper>,
	public TMC2208_n::DRV_STATUS_i<TMC2208Stepper>,
	public TMC2208_n::PWMCONF_i<TMC2208Stepper>,
	public TMC2208_n::PWM_SCALE_i<TMC2208Stepper>,
	public TMC2208_n::PWM_AUTO_i<TMC2208Stepper>
	{
	public:
	    TMC2208Stepper(HardwareSerial &SerialPort, const float RS, uint8_t addr, TMC_HAL::SSwitch &sswitch);
		TMC2208Stepper(HardwareSerial &SerialPort, const float RS);
		#if SW_CAPABLE_PLATFORM
			TMC2208Stepper(SoftwareSerial &ser, const float RS);
		#endif

		void defaults();
		void resetLibCache();
		void push();
		void begin(const uint32_t baud = 19200);
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::enn(); }

		using GCONF_t       	= TMC2208_n::GCONF_t;
		using GSTAT_t       	= TMC2208_n::GSTAT_t;
		using IFCNT_t       	= TMC2208_n::IFCNT_t;
		using SLAVECONF_t   	= TMC2208_n::SLAVECONF_t;
		using OTP_PROG_t    	= TMC2208_n::OTP_PROG_t;
		using OTP_READ_t    	= TMC2208_n::OTP_READ_t;
		using IOIN_t        	= TMC2208_n::IOIN_t;
		using FACTORY_CONF_t	= TMC2208_n::FACTORY_CONF_t;
		using IHOLD_IRUN_t  	= TMC2208_n::IHOLD_IRUN_t;
		using TPOWERDOWN_t  	= TMC2208_n::TPOWERDOWN_t;
		using TSTEP_t       	= TMC2208_n::TSTEP_t;
		using TPWMTHRS_t    	= TMC2208_n::TPWMTHRS_t;
		using VACTUAL_t     	= TMC2208_n::VACTUAL_t;
		using MSCNT_t       	= TMC2208_n::MSCNT_t;
		using MSCURACT_t    	= TMC2208_n::MSCURACT_t;
		using CHOPCONF_t    	= TMC2208_n::CHOPCONF_t;
		using DRV_STATUS_t  	= TMC2208_n::DRV_STATUS_t;
		using PWMCONF_t     	= TMC2208_n::PWMCONF_t;
		using PWM_SCALE_t   	= TMC2208_n::PWM_SCALE_t;
		using PWM_AUTO_t    	= TMC2208_n::PWM_AUTO_t;
};

class TMC2209Stepper :
	public TMC_HAL::TMC_UART,
	public TMCStepper					<TMC2209Stepper>,
	public TMC2209_n::TMC_RMS			<TMC2209Stepper>,
	public TMC2209_n::GCONF_i       	<TMC2209Stepper>,
	public TMC2209_n::GSTAT_i       	<TMC2209Stepper>,
	public TMC2209_n::IFCNT_i       	<TMC2209Stepper>,
	public TMC2209_n::SLAVECONF_i   	<TMC2209Stepper>,
	public TMC2209_n::OTP_PROG_i    	<TMC2209Stepper>,
	public TMC2209_n::OTP_READ_i    	<TMC2209Stepper>,
	public TMC2209_n::IOIN_i        	<TMC2209Stepper>,
	public TMC2209_n::FACTORY_CONF_i	<TMC2209Stepper>,
	public TMC2209_n::IHOLD_IRUN_i  	<TMC2209Stepper>,
	public TMC2209_n::TPOWERDOWN_i  	<TMC2209Stepper>,
	public TMC2209_n::TSTEP_i       	<TMC2209Stepper>,
	public TMC2209_n::TPWMTHRS_i    	<TMC2209Stepper>,
	public TMC2209_n::TCOOLTHRS_i   	<TMC2209Stepper>,
	public TMC2209_n::VACTUAL_i     	<TMC2209Stepper>,
	public TMC2209_n::SGTHRS_i      	<TMC2209Stepper>,
	public TMC2209_n::SG_RESULT_i   	<TMC2209Stepper>,
	public TMC2209_n::COOLCONF_i    	<TMC2209Stepper>,
	public TMC2209_n::MSCNT_i       	<TMC2209Stepper>,
	public TMC2209_n::MSCURACT_i    	<TMC2209Stepper>,
	public TMC2209_n::CHOPCONF_i    	<TMC2209Stepper>,
	public TMC2209_n::DRV_STATUS_i  	<TMC2209Stepper>,
	public TMC2209_n::PWMCONF_i     	<TMC2209Stepper>,
	public TMC2209_n::PWM_SCALE_i   	<TMC2209Stepper>,
	public TMC2209_n::PWM_AUTO_i    	<TMC2209Stepper>
	{
	public:
		TMC2209Stepper(HardwareSerial &SerialPort, const float RS, uint8_t addr);

		#if SW_CAPABLE_PLATFORM
			TMC2209Stepper(SoftwareSerial &SWSerial, const float RS, uint8_t addr);
		#endif

		void defaults();
		void resetLibCache();
		void push();
		void begin(const uint32_t baud = 19200);
		bool isEnabled() { return CHOPCONF_i::toff() && !IOIN_i::enn(); }

		using GCONF_t       	= TMC2209_n::GCONF_t;
		using GSTAT_t       	= TMC2209_n::GSTAT_t;
		using IFCNT_t       	= TMC2209_n::IFCNT_t;
		using SLAVECONF_t   	= TMC2209_n::SLAVECONF_t;
		using OTP_PROG_t    	= TMC2209_n::OTP_PROG_t;
		using OTP_READ_t    	= TMC2209_n::OTP_READ_t;
		using IOIN_t        	= TMC2209_n::IOIN_t;
		using FACTORY_CONF_t	= TMC2209_n::FACTORY_CONF_t;
		using IHOLD_IRUN_t  	= TMC2209_n::IHOLD_IRUN_t;
		using TPOWERDOWN_t  	= TMC2209_n::TPOWERDOWN_t;
		using TSTEP_t       	= TMC2209_n::TSTEP_t;
		using TPWMTHRS_t    	= TMC2209_n::TPWMTHRS_t;
		using TCOOLTHRS_t   	= TMC2209_n::TCOOLTHRS_t;
		using VACTUAL_t     	= TMC2209_n::VACTUAL_t;
		using SGTHRS_t      	= TMC2209_n::SGTHRS_t;
		using SG_RESULT_t   	= TMC2209_n::SG_RESULT_t;
		using COOLCONF_t    	= TMC2209_n::COOLCONF_t;
		using MSCNT_t       	= TMC2209_n::MSCNT_t;
		using MSCURACT_t    	= TMC2209_n::MSCURACT_t;
		using CHOPCONF_t    	= TMC2209_n::CHOPCONF_t;
		using DRV_STATUS_t  	= TMC2209_n::DRV_STATUS_t;
		using PWMCONF_t     	= TMC2209_n::PWMCONF_t;
		using PWM_SCALE_t   	= TMC2209_n::PWM_SCALE_t;
		using PWM_AUTO_t    	= TMC2209_n::PWM_AUTO_t;
};

using TMC2226Stepper = TMC2209Stepper;

class TMC2300Stepper :
	public TMC_HAL::TMC_UART,
	public TMCStepper<TMC2300Stepper>,
	public TMC2300_n::TMC_RMS<TMC2300Stepper>,
	public TMC2300_n::GCONF_i<TMC2300Stepper>,
	public TMC2300_n::GSTAT_i<TMC2300Stepper>,
	public TMC2300_n::IFCNT_i<TMC2300Stepper>,
	public TMC2300_n::SLAVECONF_i<TMC2300Stepper>,
	public TMC2300_n::IOIN_i<TMC2300Stepper>,
	public TMC2300_n::IHOLD_IRUN_i<TMC2300Stepper>,
	public TMC2300_n::TPOWERDOWN_i<TMC2300Stepper>,
	public TMC2300_n::TSTEP_i<TMC2300Stepper>,
	public TMC2300_n::VACTUAL_i<TMC2300Stepper>,
	public TMC2300_n::TCOOLTHRS_i<TMC2300Stepper>,
	public TMC2300_n::SGTHRS_i<TMC2300Stepper>,
	public TMC2300_n::SG_VALUE_i<TMC2300Stepper>,
	public TMC2300_n::COOLCONF_i<TMC2300Stepper>,
	public TMC2300_n::MSCNT_i<TMC2300Stepper>,
	public TMC2300_n::CHOPCONF_i<TMC2300Stepper>,
	public TMC2300_n::DRV_STATUS_i<TMC2300Stepper>,
	public TMC2300_n::PWMCONF_i<TMC2300Stepper>,
	public TMC2300_n::PWM_SCALE_i<TMC2300Stepper>,
	public TMC2300_n::PWM_AUTO_i<TMC2300Stepper>
	{
		public:
			TMC2300Stepper(HardwareSerial &SerialPort, const float RS, uint8_t addr);

			#if SW_CAPABLE_PLATFORM
				TMC2300Stepper(SoftwareSerial &SWSerial, const float RS, uint8_t addr);
			#endif
			void defaults();
			void resetLibCache();
			void push();
			void begin(const uint32_t baud = 19200);
			bool isEnabled() { return CHOPCONF_i::enable_drv() && IOIN_i::en(); }

			using GCONF_t       = TMC2300_n::GCONF_t;
			using GSTAT_t       = TMC2300_n::GSTAT_t;
			using IFCNT_t       = TMC2300_n::IFCNT_t;
			using SLAVECONF_t   = TMC2300_n::SLAVECONF_t;
			using IOIN_t        = TMC2300_n::IOIN_t;
			using IHOLD_IRUN_t  = TMC2300_n::IHOLD_IRUN_t;
			using TPOWERDOWN_t  = TMC2300_n::TPOWERDOWN_t;
			using TSTEP_t       = TMC2300_n::TSTEP_t;
			using VACTUAL_t     = TMC2300_n::VACTUAL_t;
			using TCOOLTHRS_t   = TMC2300_n::TCOOLTHRS_t;
			using SGTHRS_t      = TMC2300_n::SGTHRS_t;
			using SG_VALUE_t    = TMC2300_n::SG_VALUE_t;
			using COOLCONF_t    = TMC2300_n::COOLCONF_t;
			using MSCNT_t       = TMC2300_n::MSCNT_t;
			using CHOPCONF_t    = TMC2300_n::CHOPCONF_t;
			using DRV_STATUS_t  = TMC2300_n::DRV_STATUS_t;
			using PWMCONF_t     = TMC2300_n::PWMCONF_t;
			using PWM_SCALE_t   = TMC2300_n::PWM_SCALE_t;
			using PWM_AUTO_t    = TMC2300_n::PWM_AUTO_t;
};

class TMC2224Stepper : public TMC2208Stepper, public TMC2224_n::IOIN_i<TMC2224Stepper> {
	public:
	using TMC2208Stepper::TMC2208Stepper;

	using TMC2224_n::IOIN_i<TMC2224Stepper>::IOIN;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::enn;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::ms1;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::ms2;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::pdn_uart;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::spread;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::step;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::sel_a;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::dir;
	using TMC2224_n::IOIN_i<TMC2224Stepper>::version;
};

class TMC2660Stepper : TMC2660_n::TMC_SPI {
	public:
		TMC2660Stepper(SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		TMC2660Stepper(SW_SPIClass &spi, TMC_HAL::PinDef pinCS, const float RS, const int8_t link_index = -1);
		void begin();
		bool isEnabled() const;
		uint8_t test_connection();
		uint16_t cs2rms(const uint8_t CS) const;
		uint16_t rms_current() const;
		void rms_current(const uint16_t mA);
		//uint16_t getMilliamps() {return val_mA;}
		void push();
		uint8_t savedToff() const { return _savedToff; }

		// Helper functions
		void microsteps(const uint16_t ms);
		uint16_t microsteps();
		void blank_time(const uint8_t value);
		uint8_t blank_time() const;
		void hysteresis_end(const int8_t value);
		int8_t hysteresis_end() const;
		void hysteresis_start(const uint8_t value);
		uint8_t hysteresis_start() const;

		// W: DRVCONF
		void DRVCONF(const uint32_t);
		void tst(const bool);
		void slph(const uint8_t);
		void slpl(const uint8_t);
		void diss2g(const bool);
		void ts2g(const uint8_t);
		void sdoff(const bool);
		void vsense(const bool);
		void rdsel(const uint8_t);
		uint32_t DRVCONF() const;
		bool tst() const;
		uint8_t slph() const;
		uint8_t slpl() const;
		bool diss2g() const;
		uint8_t ts2g() const;
		bool sdoff() const;
		bool vsense() const;
		uint8_t rdsel() const;

		// W: DRVCTRL
		void DRVCTRL(const uint32_t);
		void pha(const bool B);
		void ca(const uint8_t B);
		void phb(const bool B);
		void cb(const uint8_t B);
		bool pha();
		uint8_t ca();
		bool phb();
		uint8_t cb();
		void intpol(const bool);
		void dedge(const bool);
		void mres(const uint8_t);
		uint32_t DRVCTRL() const;
		bool intpol();
		bool dedge();
		uint8_t mres();

		// W: CHOPCONF
		void CHOPCONF(const uint32_t);
		void tbl(const uint8_t);
		void chm(const bool);
		void rndtf(const bool);
		void hdec(const uint8_t);
		void hend(const uint8_t);
		void hstrt(const uint8_t);
		void toff(const uint8_t);
		uint32_t CHOPCONF() const;
		uint8_t tbl() const;
		bool chm() const;
		bool rndtf() const;
		uint8_t hdec() const;
		uint8_t hend() const;
		uint8_t hstrt() const;
		uint8_t toff() const;

		// R: DRVSTATUS
		uint32_t DRV_STATUS() { return DRVSTATUS(); }
		uint32_t DRVSTATUS();
		uint16_t mstep();
		uint8_t se();
		bool stst();
		bool olb();
		bool ola();
		bool s2gb();
		bool s2ga();
		bool otpw();
		bool ot();
		bool sg();
		uint16_t sg_result();

		// W: SGCSCONF
		uint32_t SGCSCONF() const;
		void sfilt(const bool);
		void sgt(const uint8_t);
		void cs(const uint8_t);
		void SGCSCONF(const uint32_t);
		bool sfilt() const;
		uint8_t sgt() const;
		uint8_t cs() const;

		// W: SMARTEN
		void SMARTEN(const uint32_t);
		void seimin(const bool B);
		void sedn(const uint8_t B);
		void semax(const uint8_t B);
		void seup(const uint8_t B);
		void semin(const uint8_t B);
		uint32_t SMARTEN() const;
		bool seimin() const;
		uint8_t sedn() const;
		uint8_t semax() const;
		uint8_t seup() const;
		uint8_t semin() const;
		/*
		// Alias
		SET_ALIAS(void, polarity_A, bool, pha);
		SET_ALIAS(void, current_A, uint8_t, ca);
		SET_ALIAS(void, polarity_B, bool, phb);
		SET_ALIAS(void, current_b, uint8_t, cb);
		SET_ALIAS(void, interpolate, bool, intpol);
		SET_ALIAS(void, double_edge_step, bool, dedge);
		SET_ALIAS(void, microsteps, uint8_t, mres);
		SET_ALIAS(void, blank_time, uint8_t, tbl);
		SET_ALIAS(void, chopper_mode, bool, chm);
		SET_ALIAS(void, random_off_time, bool, rndtf);
		SET_ALIAS(void, hysteresis_decrement, uint8_t, hdec);
		SET_ALIAS(void, hysteresis_low, uint8_t, hend);
		SET_ALIAS(void, hysteresis_start, uint8_t, hstrt);
		SET_ALIAS(void, off_time, uint8_t, toff);
		*/

		uint8_t status_response;

		// Deleted functions
		__attribute__((deprecated("Please provide a sense resistor value")))
		TMC2660Stepper(TMC_HAL::PinDef pinCS, TMC_HAL::PinDef pinMOSI, TMC_HAL::PinDef pinMISO, TMC_HAL::PinDef pinSCK, const int8_t link_index = -1) = delete;

	private:
		DRVCTRL_1_t DRVCTRL_1_register{{.sr=0}};
		DRVCTRL_0_t DRVCTRL_0_register{{.sr=0}};
		TMC2660_n::CHOPCONF_t CHOPCONF_register{{.sr=0}};
		SMARTEN_t SMARTEN_register{{.sr=0}};
		SGCSCONF_t SGCSCONF_register{{.sr=0}};
		DRVCONF_t DRVCONF_register{{.sr=0}};
		READ_RDSEL00_t READ_RDSEL00_register{{.sr=0}};
		READ_RDSEL01_t READ_RDSEL01_register{{.sr=0}};
		READ_RDSEL10_t READ_RDSEL10_register{{.sr=0}};

		const float Rsense;
		static constexpr float default_RS = 0.1;
		float holdMultiplier = 0.5;
		uint32_t spi_speed = 16000000/8; // Default 2MHz
		uint8_t _savedToff = 0;

		uint32_t read() {
			return TMC2660_n::TMC_SPI::read(DRVCONF_register.sr);
		}
};
