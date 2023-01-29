#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdbool.h>
#include <stdint.h>

#define GCONFI_REG 0x00
#define GSTAT_REG 0x01
#define IFCNT_REG 0x02
#define SLAVECONF_REG 0x03
#define IOIN_REG 0x04
#define OUTPUT_REG 0x04
#define X_COMPARE 0x05
#define OTP_PROG_REG 0x06
#define FACTORY_CONF_REG 0x08
#define SHORT_CONF_REG 0x09
#define DRV_CONF_REG 0x0A
#define GLOBAL_SCALER_REG 0x0B
#define OFFSET_READ 0x0C

#define IHOLD_IRUN_REG 0x10
#define TPOWER_DOWN_REG 0x11
#define TPWMTHRS_REG 0x13
#define TCOOLTHRS_REG 0x14
#define THIGH_REG 0x15
#define RAMPMODE_REG 0x20
#define XACTUAL_REG 0x21
#define VACTUAL_REG 0x22
#define VSTART_REG 0x23
#define A1_REG 0x24
#define V1_REG 0x25
#define AMAX_REG 0x26
#define VMAX_REG 0x27
#define DMAX_REG 0x28 
#define D1_REG 0x2A
#define VSTOP_REG 0x2B
#define TZEROWAIT_REG 0x2C
#define XTARGET_REG 0x2D
#define SW_MODE_REG

enum RampMode_t { Position, Velocity_Pos, Velocity_Neg, Hold };

struct TMCConfig_t
{
    uint8_t EN_Pin;

    RampMode_t RampMode;

    uint8_t IRun;
    uint8_t IHold;
    uint8_t IHoldDelay;

    uint32_t VStart;
    uint32_t V1;
    uint32_t VMax;
    uint32_t VStop;

    uint16_t A1;
    uint16_t AMax;
    uint16_t D1;
    uint16_t DMax;
};

class TMCStepper
{
    public:

    TMCStepper(TMCConfig_t* config);

    bool Setup();

    bool Reconfig(TMCConfig_t* config);

    bool Set_XActual(int32_t data);
    bool Get_XActual(int32_t& buffer);

    bool Set_XTarget(int32_t data);
    bool Get_XTarget(int32_t& buffer);

    bool Set_RampMode(RampMode_t mode);

    bool Is_SPIMode(bool& SPIMode);


    private:
    
    TMCConfig_t* TMC_Config;

    bool Set_VActual(int32_t data);

    bool Set_VStart(uint32_t data);

    bool Set_V1(uint32_t data);

    bool Set_VMax(uint32_t data);

    bool Set_VStop(uint32_t data);

    bool Set_A1(uint16_t data);

    bool Set_AMax(uint16_t data);

    bool Set_D1(uint16_t data);

    bool Set_DMax(uint16_t data);

    bool SPI_Write(uint8_t addr, void data, uint8_t size);
    bool SPI_Read(uint8_t addr, void& buffer, uint8_t size);
};