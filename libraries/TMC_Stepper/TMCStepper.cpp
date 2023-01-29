#include "TMCStepper.h"

TMCStepper::TMCStepper(TMCConfig_t* config) : TMCConfig(config) {}

bool TMCStepper::Setup()
{
    uint8_t err_count = 0;
    bool ret;
    ret = Set_RampMode(TMCConfig.RampMode);
    if(!ret)
        err_count++;
    ret = Set_VStart(TMCConfig.VStart);
    if(!ret)
        err_count++;
    ret = Set_V1(TMCConfig.V1);
    if(!ret)
        err_count++;
    ret = Set_VMax(TMCConfig.VMax);
    if(!ret)
        err_count++;
    ret = Set_VStop(TMCConfig.VStop);
    if(!ret)
        err_count++;
    ret = Set_A1(TMCConfig.A1);
    if(!ret)
        err_count++;
    ret = Set_AMax(TMCConfig.AMax);
    if(!ret)
        err_count++;
    ret = Set_D1(TMCConfig.D1);
    if(!ret)
        err_count++;
    ret = Set_DMax(TMCConfig.DMax);
    if(!ret)
        err_count++;

    if(err_count)
        return false;
    return true;
}

bool TMCStepper::Reconfig(TMCConfig_t* config)
{
    TMCConfig = config;
    Setup();
}

bool TMCStepper::Set_XActual(int32_t data)
{
    bool ret = SPI_Write(XACTUAL_REG, data, 4);
    return ret;
}

bool TMCStepper::Get_XActual(int32_t& buffer)
{
    bool ret = SPI_Read(XACTUAL_REG, buffer, 4);
    return ret;
}

bool TMCStepper::Set_XTarget(int32_t data)
{
    bool ret = SPI_Write(XTARGET_REG, data, 4);
    return ret;
}

bool TMCStepper::Get_XTarget(int32_t& buffer)
{
    bool ret = SPI_Read(XTARGET_REG, buffer, 4);
    return ret;
}

bool TMCStepper::Set_VActual(int32_t data)
{
    bool ret = SPI_Write(VACTUAL_REG, data, 4);
    return ret;
}

bool TMCStepper::Set_VStart(uint32_t data)
{
    bool ret = SPI_Write(VSTART_REG, data, 4);
    return ret;
}

bool TMCStepper::Set_V1(uint32_t data)
{
    bool ret = SPI_Write(V1_REG, data, 4);
    return ret;
}

bool TMCStepper::Set_VMax(uint32_t data)
{
    bool ret = SPI_Write(VMAX_REG, data, 4);
    return ret;
}

bool TMCStepper::Set_VStop(uint32_t data)
{
    if(data == 0)
        return false;
    
    bool ret = SPI_Write(VSTOP_REG, data, 4);
    return ret;
}

bool TMCStepper::Set_A1(uint16_t data)
{
    bool ret = SPI_Write(A1_REG, data, 2);
    return ret;
}

bool TMCStepper::Set_AMax(uint16_t data)
{
    bool ret = SPI_Write(AMAX_REG, data, 2);
    return ret;
}

bool TMCStepper::Set_D1(uint16_t data)
{
    if(data == 0)
        return false;
    
    bool ret = SPI_Write(D1_REG, data, 2);
    return ret;
}

bool TMCStepper::Set_DMax(uint16_t data)
{
    bool ret = SPI_Write(DMAX_REG, data, 2);
    return ret;
}

bool TMCStepper::Set_RampMode(RampMode_t mode)
{
    bool ret = SPI_Write(RAMPMODE_REG, mode, 1);
    return ret;
}

bool TMCStepper::Is_SPIMode(bool& SPIMode)
{
    uint16_t IOIN;
    
    bool ret = SPI_Read(IOIN_REG, &IOIN, 2);
    if(IOIN & (1<<7))
        SPIMode = false;
    else
        SPIMode = true;
    
    return ret;
}

bool TMCStepper::SPI_Write(uint8_t addr, void data, uint8_t size)
{
    uint8_t bytes[size] = { 0 };
    for(uint8_t i = 0; i < size; i++)
        uint8_t byte = data >> (8*(size-i-1));
    
    SPI.beginTransation();
    digitalWrite(EN_Pin, LOW);
    SPI.transfer(addr | WRITE_FLAG);

    for(uint8_t i = 0; i < size; i++)
        SPI.transfer(bytes[i]);
    
    digitalWrite(EN_Pin, HIGH);
    SPI.endTransaction();

    return true;
}

bool TMCStepper::SPIRead(uint8_t aadr, void* pbuffer, uint8_t size)
{
    *pbuffer = 0;
    
    SPI.beginTransation();
    digitalWrite(EN_Pin, LOW);
    SPI.transfer(addr);
    
    for(uint8_t i = 0; i < size; i++)
        *pbuffer |= (uint32_t)SPI.transfer(addr) << (8*(size-i-1));

    digitalWrite(EN_Pin, HIGH);
    SPI.endTransaction();

    return true;
}