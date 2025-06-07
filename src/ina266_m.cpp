/*****************************************************************
* This code is a modified library for the  INA226 Current and Power
* Sensor Module originally Written by Wolfgang (Wolle) Ewald
* https://wolles-elektronikkiste.de/en/ina226-current-and-power-sensor (English)
* https://wolles-elektronikkiste.de/ina226 (German)
* Modified by Martin Marsh for use with an external shunt and high current
* applications
*
* 1)  Units are standardised to Amps, Volts and Watts for functions
*     including setShuntValues.  However, getShuntVoltage_mV returns
*     uses mV as expected and shunt voltage alerts must be set in mV
*
* 2) Shunt parameters used to calibrate via setShuntValues - to calibrate
*    change the max_shunt_voltage (V) at measured max current
*
* 3) Functions to support Battery capacity calculation based on current
*    flow. 
*
*
******************************************************************/

#include "ina266_m.h"

bool INA226_M::init(){
    _wire->beginTransmission(i2cAddress);
    if(_wire->endTransmission()){
        return 0;
    }
    reset_INA226();
    calVal = 2048; // default
    writeRegister(INA226_CAL_REG, calVal);
    setAverage(INA226_AVERAGE_1);
    setConversionTime(INA226_CONV_TIME_1100);
    setMeasureMode(INA226_CONTINUOUS);
    current_LSB = 0.000025;
    pwrMultiplier_W = 625.0;
    convAlert = false;
    limitAlert = false;
    busVoltageCorrFactor = 1.0;
    i2cErrorCode = 0;
    return 1;
}

void INA226_M::reset_INA226(){
    writeRegister(INA226_CONF_REG, INA226_RST); 
}

void INA226_M::setConversionTime(INA226_CONV_TIME convTime){
    setConversionTime(convTime, convTime);
}

void INA226_M::setConversionTime(INA226_CONV_TIME shuntConvTime, INA226_CONV_TIME busConvTime){
    uint16_t currentConfReg = readRegister(INA226_CONF_REG);
    currentConfReg &= ~(0x01C0);  
    currentConfReg &= ~(0x0038);
    uint16_t convMask = (static_cast<uint16_t>(shuntConvTime))<<3;
    currentConfReg |= convMask;
    convMask = busConvTime<<6;
    currentConfReg |= convMask;
    writeRegister(INA226_CONF_REG, currentConfReg);
}


void INA226_M::setMeasureMode(INA226_MEASURE_MODE mode){
    deviceMeasureMode = mode;
    uint16_t currentConfReg = readRegister(INA226_CONF_REG);
    currentConfReg &= ~(0x0007);
    currentConfReg |= deviceMeasureMode;
    writeRegister(INA226_CONF_REG, currentConfReg);
}

float INA226_M::getBusVoltage(){
    uint16_t val;
    val = readRegister(INA226_BUS_REG);
    return (val * 0.00125 * busVoltageCorrFactor);
}

float INA226_M::getShuntVoltage_mV(){
    int16_t val;
    val = static_cast<int16_t>(readRegister(INA226_SHUNT_REG));
    return (val * 0.0025); 
}


void INA226_M::setAverage(INA226_AVERAGES averages){
    deviceAverages = averages;
    uint16_t currentConfReg = readRegister(INA226_CONF_REG);
    currentConfReg &= ~(0x0E00);  
    currentConfReg |= deviceAverages;
    writeRegister(INA226_CONF_REG, currentConfReg);
}

//set shunt values eg max_shunt_voltage, float max_shunt_current for .075mv 100A shunt
// ensure calVal used is rounded to an int
// r  = shunt_mv/max_current
// approxLsb = max_current/32768.0
// cal = round(0.00512/(approxlsb * r))
// lsb = 0.00512/(cal * r)
void INA226_M::setShuntValues(float max_shunt_voltage, float max_shunt_current){
    float resistance = max_shunt_voltage/max_shunt_current;
    calVal = round(0.00512/((max_shunt_current/32768.0) * resistance));
    current_LSB=0.00512/(float(calVal) * resistance);   
    pwrMultiplier_W = 25.0*current_LSB;
    writeRegister(INA226_CAL_REG, calVal);          
}

float INA226_M::getCurrent(){
    int16_t val;
    val = static_cast<int16_t>(readRegister(INA226_CURRENT_REG));
    return (val * current_LSB );
}


float INA226_M::getBusPower(){
    uint16_t val;
    val = readRegister(INA226_PWR_REG);
    return (val * pwrMultiplier_W);
}

void INA226_M::startSingleMeasurement(){
    uint16_t val = readRegister(INA226_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    val = readRegister(INA226_CONF_REG);
    writeRegister(INA226_CONF_REG, val);        // Starts conversion
    uint16_t convReady = 0x0000;
    unsigned long convStart = millis();
    while(!convReady && ((millis()-convStart) < 2000)){
        convReady = ((readRegister(INA226_MASK_EN_REG)) & 0x0008); // checks if sampling is completed
    }
}

// Don't wait for conversion to complete
void INA226_M::startSingleMeasurementNoWait(){
    uint16_t val = readRegister(INA226_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    val = readRegister(INA226_CONF_REG);
    writeRegister(INA226_CONF_REG, val);        // Starts conversion
}

void INA226_M::powerDown(){
    confRegCopy = readRegister(INA226_CONF_REG);
    setMeasureMode(INA226_POWER_DOWN);   
}

void INA226_M::powerUp(){
    writeRegister(INA226_CONF_REG, confRegCopy);
    delayMicroseconds(40);  
}

// Returns 1 if conversion is still ongoing
bool INA226_M::isBusy(){
    return (!(readRegister(INA226_MASK_EN_REG) &0x0008));
}
    
void INA226_M::waitUntilConversionCompleted(){
    readRegister(INA226_MASK_EN_REG); // clears CNVR (Conversion Ready) Flag
    uint16_t convReady = 0x0000;
    while(!convReady){
        convReady = ((readRegister(INA226_MASK_EN_REG)) & 0x0008); // checks if sampling is completed
    }
}

void INA226_M::setAlertPinActiveHigh(){
    uint16_t val = readRegister(INA226_MASK_EN_REG);
    val |= 0x0002;
    writeRegister(INA226_MASK_EN_REG, val);
}

void INA226_M::enableAlertLatch(){
    uint16_t val = readRegister(INA226_MASK_EN_REG);
    val |= 0x0001;
    writeRegister(INA226_MASK_EN_REG, val);
}

void INA226_M::enableConvReadyAlert(){
    uint16_t val = readRegister(INA226_MASK_EN_REG);
    val |= 0x0400;
    writeRegister(INA226_MASK_EN_REG, val);
}
    
void INA226_M::setAlertType(INA226_ALERT_TYPE type, float limit){
    deviceAlertType = type;
    uint16_t alertLimit = 0;
    
    switch(deviceAlertType){
        case INA226_SHUNT_OVER:
            alertLimit = limit * 400;           
            break;
        case INA226_SHUNT_UNDER:
            alertLimit = limit * 400; 
            break;
        case INA226_CURRENT_OVER:
            deviceAlertType = INA226_SHUNT_OVER;
            alertLimit = limit / current_LSB;
            break;
        case INA226_CURRENT_UNDER:
            deviceAlertType = INA226_SHUNT_UNDER;
            alertLimit = limit / current_LSB;
            break;
        case INA226_BUS_OVER:
            alertLimit = limit * 800;
            break;
        case INA226_BUS_UNDER:
            alertLimit = limit * 800;
            break;
        case INA226_POWER_OVER:
            alertLimit = limit / pwrMultiplier_W;
            break;
    }
    
    writeRegister(INA226_ALERT_LIMIT_REG, alertLimit);
    
    uint16_t value = readRegister(INA226_MASK_EN_REG);
    value &= ~(0xF800);
    value |= deviceAlertType;
    writeRegister(INA226_MASK_EN_REG, value);
    
}

void INA226_M::readAndClearFlags(){
    uint16_t value = readRegister(INA226_MASK_EN_REG);
    overflow = (value>>2) & 0x0001;
    convAlert = (value>>3) & 0x0001;
    limitAlert = (value>>4) & 0x0001;
}

uint8_t INA226_M::getI2cErrorCode(){
    return i2cErrorCode;
}  

/************************************************ 
    private functions
*************************************************/

void INA226_M::writeRegister(uint8_t reg, uint16_t val){
  _wire->beginTransmission(i2cAddress);
  uint8_t lVal = val & 255;
  uint8_t hVal = val >> 8;
  _wire->write(reg);
  _wire->write(hVal);
  _wire->write(lVal);
  _wire->endTransmission();
}
  
uint16_t INA226_M::readRegister(uint8_t reg){
  uint8_t MSByte = 0, LSByte = 0;
  uint16_t regValue = 0;
  _wire->beginTransmission(i2cAddress);
  _wire->write(reg);
  i2cErrorCode = _wire->endTransmission(false);
  _wire->requestFrom(static_cast<uint8_t>(i2cAddress),static_cast<uint8_t>(2));
  if(_wire->available()){
    MSByte = _wire->read();
    LSByte = _wire->read();
  }
  regValue = (MSByte<<8) + LSByte;
  return regValue;
}
    



