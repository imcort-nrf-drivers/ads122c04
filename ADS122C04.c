/*
  This is a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface

  It allows you to measure temperature very accurately using a
  Platinum Resistance Thermometer

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!

  Written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Based on the TI datasheet:
  https://www.ti.com/product/ADS122C04
  https://www.ti.com/lit/ds/symlink/ads122c04.pdf
  Using the example code from the "High Precision Temperature Measurement
  for Heat and Cold Meters Reference Design" (TIDA-01526) for reference:
  http://www.ti.com/tool/TIDA-01526
  http://www.ti.com/lit/zip/tidcee5

  The MIT License (MIT)
  Copyright (c) 2020 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ADS122C04.h"
#include "transfer_handler.h"
#include "math.h"

static uint8_t iic_sendbuf[20];
static uint8_t iic_recvbuf[20];

//Private Variables
uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

bool _printDebug = true; //Flag to print debugging variables

// Resistance of the reference resistor
const float REFERENCE_RESISTOR = 3906.0f;

// Amplifier gain setting
float AMPLIFIER_GAIN = 1.0f;

// Internal temperature sensor resolution
// One 14-bit LSB equals 0.03125
const float TEMPERATURE_SENSOR_RESOLUTION = 0.03125;

ADS122C04Reg_t ADS122C04_Reg; // Global to hold copies of all four configuration registers

bool ADS122C04_writeReg(uint8_t reg, uint8_t writeValue); // write a value to the selected register
bool ADS122C04_readReg(uint8_t reg, uint8_t *readValue); // read a value from the selected register (returned in readValue)

//bool ADS122C04_getConversionData(uint32_t *conversionData); // read the raw 24-bit conversion result
bool ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count); // read the raw conversion result and count (if enabled)

bool ADS122C04_sendCommand(uint8_t command); // write to the selected command register
bool ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value); // write a value to the selected command register


bool ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
    uint8_t command = 0;
    command = ADS122C04_WRITE_CMD(reg);
    return(ADS122C04_sendCommandWithValue(command, writeValue));
}

bool ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
    
    iic_sendbuf[0] = ADS122C04_READ_CMD(reg);

    iic_send(_deviceAddress, iic_sendbuf, 1, true);
    iic_read(_deviceAddress, iic_recvbuf, 1);
    
    *readValue = iic_recvbuf[0];
        
    return(true);
}

bool ADS122C04_sendCommand(uint8_t command)
{
    
	iic_sendbuf[0] = command;

	iic_send(_deviceAddress, iic_sendbuf, 1, true);
	
	return(true);
}

bool ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
    
	iic_sendbuf[0] = command;
	iic_sendbuf[1] = value;

	iic_send(_deviceAddress, iic_sendbuf, 2, true);
	
	return(true);
}

// Read the conversion result with count byte.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count)
{
    
    iic_sendbuf[0] = ADS122C04_RDATA_CMD;

    iic_send(_deviceAddress, iic_sendbuf, 1, true);
    iic_read(_deviceAddress, iic_recvbuf, 4);

    *count = iic_recvbuf[0];
    *conversionData = ((uint32_t)iic_recvbuf[3]) | ((uint32_t)iic_recvbuf[2]<<8) | ((uint32_t)iic_recvbuf[1]<<16);

    return(true);
}

// Read the conversion result.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool ADS122C04_getConversionData(uint32_t *conversionData)
{
    
	iic_sendbuf[0] = ADS122C04_RDATA_CMD;

	iic_send(_deviceAddress, iic_sendbuf, 1, true);
	iic_read(_deviceAddress, iic_recvbuf, 3);
    
    *conversionData = ((uint32_t)iic_recvbuf[2]) | ((uint32_t)iic_recvbuf[1]<<8) | ((uint32_t)iic_recvbuf[0]<<16);
    
    return(true);
}


//Attempt communication with the device and initialise it
//Return true if successful
bool ads_begin(uint8_t deviceAddress)
{
    iic_init();
    
    _deviceAddress = deviceAddress; //If provided, store the I2C address from user

    delay(1); // wait for power-on reset to complete (datasheet says we should do this)

    ads_reset(); // reset the ADS122C04 (datasheet says we should do this)

    ads_enablePGA(ADS122C04_PGA_ENABLED);
    ads_setDataRate(ADS122C04_DATA_RATE_330SPS);
    ads_setOperatingMode(ADS122C04_OP_MODE_NORMAL);
    ads_setConversionMode(ADS122C04_CONVERSION_MODE_SINGLE_SHOT);
    ads_setVoltageReference(ADS122C04_VREF_INTERNAL);
    ads_enableInternalTempSensor(ADS122C04_TEMP_SENSOR_OFF);
    ads_setDataCounter(ADS122C04_DCNT_DISABLE);
    ads_setDataIntegrityCheck(ADS122C04_CRC_DISABLED);
    ads_setBurnOutCurrent(ADS122C04_BURN_OUT_CURRENT_OFF);
    ads_setIDACcurrent(ADS122C04_IDAC_CURRENT_250_UA);

    ads_setInputMultiplexer(ADS122C04_MUX_AIN0_AIN1);
    ads_setGain(ADS122C04_GAIN_2);
    ads_setIDAC1mux(ADS122C04_IDAC1_AIN0);
    ads_setIDAC2mux(ADS122C04_IDAC2_DISABLED);

    ads_printADS122C04config();
    
    

    return true; // Default to using 'safe' settings (disable the IDAC current sources)
}

float ads_readResistance(float gain) // Read the temperature in Centigrade
{
	float resResult = ads_readRawVoltage();
	resResult *= REFERENCE_RESISTOR;
	resResult /= gain;

  return(resResult);
}

// Read the raw signed 24-bit ADC value as int32_t
// The result needs to be multiplied by VREF / GAIN to convert to Volts
float ads_readRawVoltage(void)
{
	float voltageResult;
  union raw_voltage_union raw_v; // union to convert uint32_t to int32_t
  bool drdy = false; // DRDY (1 == new data is ready)

  ads_start();

  // Wait for DRDY to go valid
  while(drdy == false)
  {
    nrf_delay_ms(5); // Don't pound the bus too hard
    drdy = ads_checkDataReady();
  }

  // Read the conversion result
  if(ADS122C04_getConversionData(&raw_v.UINT32) == false)
  {
    if (_printDebug == true)
    {
      NRF_LOG_INFO("readRawVoltage: ADS122C04_getConversionData failed");
    }
    return(0);
  }

  // The raw voltage is in the bottom 24 bits of raw_temp
  // If we just do a <<8 we will multiply the result by 256
  // Instead pad out the MSB with the MS bit of the 24 bits
  // to preserve the two's complement
  if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    raw_v.UINT32 |= 0xFF000000;
	
	voltageResult = ((float)raw_v.INT32) / 8192000.0f;
	
  return(voltageResult);
}

//// Read the internal temperature
//float ads_readInternalTemperature(void)
//{
//  union internal_temperature_union int_temp; // union to convert uint16_t to int16_t
//  uint32_t raw_temp; // The raw temperature from the ADC
//  bool drdy = false; // DRDY (1 == new data is ready)
//  float ret_val = 0.0; // The return value
//  uint8_t previousWireMode = _wireMode; // Record the previous wire mode so we can restore it

//  // Enable the internal temperature sensor
//  // Reading the ADC value will return the temperature
//  if ((ads_configureADCmode(ADS122C04_TEMPERATURE_MODE)) == false)
//  {
//    if (_printDebug == true)
//    {
//      NRF_LOG_INFO("readInternalTemperature: configureADCmode (1) failed");
//    }
//    return(ret_val);
//  }

//  // Start the conversion
//  ads_start();

//  // Wait for DRDY to go valid
//  while(drdy == false)
//  {
//    nrf_delay_ms(5); // Don't pound the bus too hard
//    drdy = ads_checkDataReady();
//  }

//  // Check if we timed out
//  if (drdy == false)
//  {
//    if (_printDebug == true)
//    {
//      NRF_LOG_INFO("readInternalTemperature: checkDataReady timed out");
//    }
//    ads_configureADCmode(previousWireMode); // Attempt to restore the previous wire mode
//    return(ret_val);
//  }

//  // Read the conversion result
//  if(ADS122C04_getConversionData(&raw_temp) == false)
//  {
//    if (_printDebug == true)
//    {
//      NRF_LOG_INFO("readInternalTemperature: ADS122C04_getConversionData failed");
//    }
//    ads_configureADCmode(previousWireMode); // Attempt to restore the previous wire mode
//    return(ret_val);
//  }

//  // Restore the previous wire mode
//  if ((ads_configureADCmode(previousWireMode)) == false)
//  {
//  if (_printDebug == true)
//    {
//      NRF_LOG_INFO("readInternalTemperature: configureADCmode (2) failed");
//    }
//    return(ret_val);
//  }

//  if (_printDebug == true)
//  {
//    NRF_LOG_INFO("readInternalTemperature: raw_temp (32-bit) = 0x%x", raw_temp);
//  }

//  // The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
//  int_temp.UINT16 = (uint16_t)(raw_temp >> 10); // Extract the 14-bit value

//  // The signed temperature is now in the bottom 14 bits of int_temp.UINT16
//  // If we just do a <<2 we will multiply the result by 4
//  // Instead we will pad out the two MS bits with the MS bit of the 14 bits
//  // to preserve the two's complement
//  if ((int_temp.UINT16 & 0x2000) == 0x2000) // Check if the MS bit is 1
//  {
//    int_temp.UINT16 |= 0xC000; // Value is negative so pad with 1's
//  }
//  else
//  {
//    int_temp.UINT16 &= 0x3FFF;  // Value is positive so make sure the two MS bits are 0
//  }

//  ret_val = ((float)int_temp.INT16) * TEMPERATURE_SENSOR_RESOLUTION; // Convert to float including the 2 bit shift
//  return(ret_val);
//}

// Configure the input multiplexer
bool ads_setInputMultiplexer(uint8_t mux_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.MUX = mux_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Configure the gain
bool ads_setGain(uint8_t gain_config)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.GAIN = gain_config;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Enable/disable the Programmable Gain Amplifier
bool ads_enablePGA(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
    return(false);
  ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Set the data rate (sample speed)
bool ads_setDataRate(uint8_t rate)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.DR = rate;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the operating mode (normal / turbo)
bool ads_setOperatingMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.MODE = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the conversion mode (single-shot / continuous)
bool ads_setConversionMode(uint8_t mode)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.CMBIT = mode;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the voltage reference
bool ads_setVoltageReference(uint8_t ref)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.VREF = ref;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the internal temperature sensor
bool ads_enableInternalTempSensor(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
    return(false);
  ADS122C04_Reg.reg1.bit.TS = enable;
  if (_printDebug == true)
  {
    NRF_LOG_INFO("enableInternalTempSensor: ADS122C04_Reg.reg1.bit.TS = 0x%x", ADS122C04_Reg.reg1.bit.TS);
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the conversion data counter
bool ads_setDataCounter(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.DCNT = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the data integrity check
bool ads_setDataIntegrityCheck(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.CRC = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Enable / disable the 10uA burn-out current source
bool ads_setBurnOutCurrent(uint8_t enable)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.BCS = enable;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the internal programmable current sources
bool ads_setIDACcurrent(uint8_t current)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
    return(false);
  ADS122C04_Reg.reg2.bit.IDAC = current;
  if (_printDebug == true)
  {
    NRF_LOG_INFO("setIDACcurrent: ADS122C04_Reg.reg2.bit.IDAC = 0x%x", ADS122C04_Reg.reg2.bit.IDAC);
  }
  return(ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the IDAC1 routing
bool ads_setIDAC1mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I1MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Configure the IDAC2 routing
bool ads_setIDAC2mux(uint8_t setting)
{
  if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
    return(false);
  ADS122C04_Reg.reg3.bit.I2MUX = setting;
  return(ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Read Config Reg 2 and check the DRDY bit
// Data is ready when DRDY is high
bool ads_checkDataReady(void)
{
  ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
  return(ADS122C04_Reg.reg2.bit.DRDY > 0);
}

// Debug print of the ADS122C04 configuration
void ads_printADS122C04config(void)
{
  if (_printDebug == true)
  {
    bool successful = true; // Flag to show if the four readRegs were successful
    // (If any one readReg returns false, success will be false)
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    successful &= ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);

    if (successful == false)
    {
      NRF_LOG_INFO("printADS122C04config: readReg failed");
      return;
    }
    else
    {
      NRF_LOG_INFO("ConfigReg0: MUX=%d", ADS122C04_Reg.reg0.bit.MUX);
      NRF_LOG_INFO(" GAIN=%d", ADS122C04_Reg.reg0.bit.GAIN);
      NRF_LOG_INFO(" PGA_BYPASS=%d", ADS122C04_Reg.reg0.bit.PGA_BYPASS);
      NRF_LOG_INFO("ConfigReg1: DR=%d", ADS122C04_Reg.reg1.bit.DR);
      NRF_LOG_INFO(" MODE=%d", ADS122C04_Reg.reg1.bit.MODE);
      NRF_LOG_INFO(" CMBIT=%d", ADS122C04_Reg.reg1.bit.CMBIT);
      NRF_LOG_INFO(" VREF=%d", ADS122C04_Reg.reg1.bit.VREF);
      NRF_LOG_INFO(" TS=%d", ADS122C04_Reg.reg1.bit.TS);
      NRF_LOG_INFO("ConfigReg2: DCNT=%d", ADS122C04_Reg.reg2.bit.DCNT);
      NRF_LOG_INFO(" CRC=%d", ADS122C04_Reg.reg2.bit.CRC);
      NRF_LOG_INFO(" BCS=%d", ADS122C04_Reg.reg2.bit.BCS);
      NRF_LOG_INFO(" IDAC=%d", ADS122C04_Reg.reg2.bit.IDAC);
      NRF_LOG_INFO("ConfigReg3: I1MUX=%d", ADS122C04_Reg.reg3.bit.I1MUX);
      NRF_LOG_INFO(" I2MUX=%d", ADS122C04_Reg.reg3.bit.I2MUX);
    }
  }
}

bool ads_reset(void)
{
  return(ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

bool ads_start(void)
{
  return(ADS122C04_sendCommand(ADS122C04_START_CMD));
}

bool ads_powerdown(void)
{
  return(ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}
