#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdbool.h>
#include <stdint.h>

enum RampMode_t { Position=0, Velocity_Pos, Velocity_Neg, Hold };

struct TMCConfig_t
{
    uint8_t EN_Pin;
    uint8_t CS_Pin;

    RampMode_t RampMode;

    uint8_t Run_Current;
    uint8_t Hold_Current;
    uint8_t Hold_Delay;

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

    uint8_t setup();

    bool reconfig(TMCConfig_t* config);

    bool set_position_counter(int32_t data);
    bool get_position_counter(int32_t& buffer, uint8_t& pstatus);

    bool set_position(int32_t data);
    bool get_position(int32_t& buffer, uint8_t& pstatus);

    bool set_rampmode(RampMode_t mode);

    bool is_SPImode(bool& SPImode);


    private:
    
    TMCConfig_t* config;

    bool get_velocity_counter(int32_t& buffer, uint8_t& pstatus);

    bool set_VStart(uint32_t data);

    bool set_V1(uint32_t data);

    bool set_VMax(uint32_t data);

    bool set_VStop(uint32_t data);

    bool set_A1(uint16_t data);

    bool set_AMax(uint16_t data);

    bool set_D1(uint16_t data);

    bool set_DMax(uint16_t data);

    bool set_current_data(uint8_t hold_current, uint8_t run_current, uint8_t hold_delay);

    bool SPI_Write(uint8_t addr, int32_t data, uint8_t size);
    bool SPI_Write(uint8_t addr, uint32_t data, uint8_t size);
    bool SPI_Read(uint8_t addr, uint32_t& pdata, uint8_t& pstatus, uint8_t size);
    bool SPI_Read(uint8_t addr, int32_t& pdata, uint8_t& pstatus, uint8_t size);
};
