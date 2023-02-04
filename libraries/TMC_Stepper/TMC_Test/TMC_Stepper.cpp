#include "TMC_Stepper.h"
#include "TMC_Defines.h"

TMCStepper::TMCStepper(TMCConfig_t* config) : config(config) {}

uint8_t TMCStepper::setup()
{
    pinMode(config->EN_Pin, OUTPUT);
    pinMode(config->CS_Pin, OUTPUT);
    
    digitalWrite(config->EN_Pin, LOW);
    
    uint8_t err_count = 0;
    bool ret;
    
    ret = SPI_Write(CHOPCONF_REG, (uint32_t)0x000100C3, 4);
    if(!ret)
      err_count++;
    ret = set_rampmode(config->RampMode);
    if(!ret)
        err_count++;
    ret = set_VStart(config->VStart);
    if(!ret)
        err_count++;
    ret = set_V1(config->V1);
    if(!ret)
        err_count++;
    ret = set_VMax(config->VMax);
    if(!ret)
        err_count++;
    ret = set_VStop(config->VStop);
    if(!ret)
        err_count++;
    ret = set_A1(config->A1);
    if(!ret)
        err_count++;
    ret = set_AMax(config->AMax);
    if(!ret)
        err_count++;
    ret = set_D1(config->D1);
    if(!ret)
        err_count++;
    ret = set_DMax(config->DMax);
    if(!ret)
        err_count++;
    ret = set_current_data(config->Run_Current, config->Hold_Current, config->Hold_Delay);
    if(!ret)
        err_count++;

    if(err_count)
        return 0;
    return err_count;
}

bool TMCStepper::reconfig(TMCConfig_t* config)
{
    this->config = config;
    bool ret = setup();
    return ret;
}

bool TMCStepper::set_position_counter(int32_t data)
{
    bool ret = SPI_Write(XACTUAL_REG, data, 4);
    return ret;
}

bool TMCStepper::get_position_counter(int32_t& pbuffer, uint8_t& pstatus)
{
    bool ret = SPI_Read(XACTUAL_REG, pbuffer, pstatus, 4);
    return ret;
}

bool TMCStepper::set_position(int32_t data)
{
    bool ret = SPI_Write(XTARGET_REG, data, 4);
    return ret;
}

bool TMCStepper::get_position(int32_t& pbuffer, uint8_t& pstatus)
{
    bool ret = SPI_Read(XTARGET_REG, pbuffer, pstatus, 4);
    return ret;
}

bool TMCStepper::get_velocity_counter(int32_t& pbuffer, uint8_t& pstatus)
{
    bool ret = SPI_Read(VACTUAL_REG, pbuffer, pstatus, 4);
    return ret;
}

bool TMCStepper::set_VStart(uint32_t data)
{
    bool ret = SPI_Write(VSTART_REG, data, 4);
    return ret;
}

bool TMCStepper::set_V1(uint32_t data)
{
    bool ret = SPI_Write(V1_REG, data, 4);
    return ret;
}

bool TMCStepper::set_VMax(uint32_t data)
{
    bool ret = SPI_Write(VMAX_REG, data, 4);
    return ret;
}

bool TMCStepper::set_VStop(uint32_t data)
{
    if(data == 0)
        return false;
    
    bool ret = SPI_Write(VSTOP_REG, data, 4);
    return ret;
}

bool TMCStepper::set_A1(uint16_t data)
{
    bool ret = SPI_Write(A1_REG, (uint32_t)data, 2);
    return ret;
}

bool TMCStepper::set_AMax(uint16_t data)
{
    bool ret = SPI_Write(AMAX_REG, (uint32_t)data, 2);
    return ret;
}

bool TMCStepper::set_D1(uint16_t data)
{
    if(data == 0)
        return false;
    
    bool ret = SPI_Write(D1_REG, (uint32_t)data, 2);
    return ret;
}

bool TMCStepper::set_DMax(uint16_t data)
{
    bool ret = SPI_Write(DMAX_REG, (uint32_t)data, 2);
    return ret;
}

bool TMCStepper::set_current_data(uint8_t hold_current, uint8_t run_current, uint8_t hold_delay)
{
    uint32_t data = ((hold_delay&0x0f)<<16) | ((run_current&0x1f)<<8) | ((hold_current&0x1f)<<0);

    bool ret = SPI_Write(IHOLD_IRUN_REG, data, 4);
    return ret;
}

bool TMCStepper::set_rampmode(RampMode_t mode)
{
    bool ret = SPI_Write(RAMPMODE_REG, (uint32_t)mode, 1);
    return ret;
}

bool TMCStepper::is_SPImode(bool& SPIMode)
{
    uint32_t pbuffer;
    uint8_t pstatus;
    
    bool ret = SPI_Read(IOIN_REG, pbuffer, pstatus, 2);
    if(pbuffer & (1<<7))
        SPIMode = false;
    else
        SPIMode = true;
    
    return ret;
}

bool TMCStepper::SPI_Write(uint8_t addr, int32_t data, uint8_t size)
{
    uint8_t bytes[size] = { 0 };
    for(uint8_t i = 0; i < size; i++)
        bytes[i] = data >> (8*(size-i-1));
    
    SPI.beginTransaction(TMC_SPI_SETTINGS);
    digitalWrite(config->CS_Pin, LOW);
    SPI.transfer(addr + WRITE_FLAG);

    for(uint8_t i = 0; i < size; i++)
        SPI.transfer(bytes[i]);
    
    digitalWrite(config->CS_Pin, HIGH);
    SPI.endTransaction();

    return true;
}

bool TMCStepper::SPI_Write(uint8_t addr, uint32_t data, uint8_t size)
{
    uint8_t bytes[size] = { 0 };
    for(uint8_t i = 0; i < size; i++)
        bytes[i] = data >> (8*(size-i-1));
    
    SPI.beginTransaction(TMC_SPI_SETTINGS);
    digitalWrite(config->CS_Pin, LOW);
    SPI.transfer(addr + WRITE_FLAG);

    for(uint8_t i = 0; i < size; i++)
        SPI.transfer(bytes[i]);
    
    digitalWrite(config->CS_Pin, HIGH);
    SPI.endTransaction();

    return true;
}

bool TMCStepper::SPI_Read(uint8_t addr, uint32_t& pdata, uint8_t& pstatus, uint8_t size)
{
    pdata = 0;
    
    SPI.beginTransaction(TMC_SPI_SETTINGS);
    digitalWrite(config->CS_Pin, LOW);
    pstatus = SPI.transfer(addr);
    
    for(uint8_t i = 0; i < size; i++)
        pdata |= (uint32_t)SPI.transfer(addr) << (8*(size-i-1));

    digitalWrite(config->CS_Pin, HIGH);
    SPI.endTransaction();

    return true;
}

bool TMCStepper::SPI_Read(uint8_t addr, int32_t& pdata, uint8_t& pstatus, uint8_t size)
{
    pdata = 0;
    
    SPI.beginTransaction(TMC_SPI_SETTINGS);
    digitalWrite(config->CS_Pin, LOW);
    pstatus = SPI.transfer(addr);
    
    for(uint8_t i = 0; i < size; i++)
        pdata |= (uint32_t)SPI.transfer(addr) << (8*(size-i-1));

    digitalWrite(config->CS_Pin, HIGH);
    SPI.endTransaction();

    return true;
}
