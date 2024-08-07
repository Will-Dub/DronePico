// -------------------------------------------------------------------------------------------------- //
// Robert's Smorgasbord 2022                                                                          //
// https://robertssmorgasbord.net                                                                     //
// https://www.youtube.com/channel/UCGtReyiNPrY4RhyjClLifBA                                           //
// QST QMC5883L 3-Axis Digital Compass and Arduino MCU – The Details (1) https://youtu.be/NTDS2Vmnr-4 //
// -------------------------------------------------------------------------------------------------- //

#include "QMC5883L.h"
#include <cmath>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

QMC5883L::QMC5883L(I2C* i2c)
: i2c(i2c)
{    
}

bool QMC5883L::begin()
{
   resetCalibration();

   return reset();
}

bool QMC5883L::reset()
{
   registers[control_register_2] = bit_soft_reset;
   registers[period_register]    = period_register_default;

   if (!writeRegister(control_register_2)) return false;
   if (!writeRegister(period_register))    return false;
   if (!readConfig())                      return false;

   return true;
}

bool QMC5883L::readConfig()
{
   return readRegisters(control_register_1, 2);
}

bool QMC5883L::writeConfig()
{
   return writeRegisters(control_register_1, 2);
}

bool QMC5883L::readData()
{
   if (!readRegisters(data_registers, 7))
   {
      return false; // Read data and status register
   }
   
   if (registers[status_register] & bit_data_overflow)  
   {
      last_error = Error::qmc_data_overflow;

      return false;
   }

   return true;
}

bool QMC5883L::readStatus()
{
   return readRegisters(status_register, 1);
}

bool QMC5883L::readTemperature()
{
   return readRegisters(temperature_registers, 2);
}

bool QMC5883L::readPeriod()
{
   return readRegisters(period_register, 1);
}

QMC5883L::Mode QMC5883L::getMode()
{
   return Mode(registers[control_register_1] & bits_mode);
}

QMC5883L::OutputDataRate QMC5883L::getOutputDataRate()
{
   return OutputDataRate(registers[control_register_1] & bits_output_data_rate);
}

QMC5883L::FullScaleRange QMC5883L::getFullScaleRange()
{
   return FullScaleRange(registers[control_register_1] & bits_full_scale_range);
}

QMC5883L::OverSampleRate QMC5883L::getOverSampleRate()
{
   return OverSampleRate(registers[control_register_1] & bits_over_sample_rate);
}

bool QMC5883L::getInterruptDisabled()
{
   return registers[control_register_2] & bit_interrupt_disabled;
}

QMC5883L::Config QMC5883L::getConfig()
{
   Config config;

   config.mode               = getMode();
   config.output_data_rate   = getOutputDataRate();
   config.full_scale_range   = getFullScaleRange();
   config.over_sample_rate   = getOverSampleRate();
   config.interrupt_disabled = getInterruptDisabled();

   return config;
}

void QMC5883L::setMode(Mode mode)
{
   registers[control_register_1] = (registers[control_register_1] & ~bits_mode) | mode;
}

void QMC5883L::setOutputDataRate(OutputDataRate output_data_rate)
{
   registers[control_register_1] = (registers[control_register_1] & ~bits_output_data_rate) | output_data_rate;
}

void QMC5883L::setFullScaleRange(FullScaleRange full_scale_range)
{
   registers[control_register_1] = (registers[control_register_1] & ~bits_full_scale_range) | full_scale_range;
}

void QMC5883L::setOverSampleRate(OverSampleRate over_sample_rate)
{
   registers[control_register_1] = (registers[control_register_1] & ~bits_over_sample_rate) | over_sample_rate;
}

void QMC5883L::setInterruptDisabled(bool interrupt_disabled)
{
   registers[control_register_2] = interrupt_disabled ? bit_interrupt_disabled : 0b0000000;
}

void QMC5883L::setConfig(const Config& config)
{
   setMode(config.mode); 
   setOutputDataRate(config.output_data_rate);
   setFullScaleRange(config.full_scale_range);
   setOverSampleRate(config.over_sample_rate);
   setInterruptDisabled(config.interrupt_disabled);
}

void QMC5883L::setCalibration(const Calibration& calib)
{
   calibration = calib;
}

void QMC5883L::resetCalibration()
{
   char axis;
   char column;
   
   for (axis = 0; axis < 3; axis++)
   {
      calibration.bias[axis] = 0;

      for (column = 0; column < 3; column++)
      {
         calibration.scale[axis][column] = (axis != column ? 0 : 1);
      }
   }
}

int QMC5883L::rawDataAxis(char axis)
{
   return *((int16_t*)(registers + data_registers + (2 * axis)));
}

int QMC5883L::rawDataX()
{
   return rawDataAxis(0);
}

int QMC5883L::rawDataY()
{
   return rawDataAxis(1);
}

int QMC5883L::rawDataZ()
{
   return rawDataAxis(2);
}

QMC5883L::RawDataAxes QMC5883L::rawDataAxes()
{
   RawDataAxes rawDataAxes;

   rawDataAxes.x = rawDataAxis(0);
   rawDataAxes.y = rawDataAxis(1);
   rawDataAxes.z = rawDataAxis(2);
   
   return rawDataAxes;
}

QMC5883L::RawDataArray QMC5883L::rawDataArray()
{
   RawDataArray rawDataArray;

   rawDataArray.axes[0] = rawDataAxis(0);
   rawDataArray.axes[1] = rawDataAxis(1);
   rawDataArray.axes[2] = rawDataAxis(2);
   
   return rawDataArray;
}

float QMC5883L::calibratedDataAxis(char axis)
{
   return   calibration.scale[axis][0] * (rawDataAxis(0) - calibration.bias[0])
          + calibration.scale[axis][1] * (rawDataAxis(1) - calibration.bias[1])
          + calibration.scale[axis][2] * (rawDataAxis(2) - calibration.bias[2]);
}

float QMC5883L::calibratedDataX()
{
   return calibratedDataAxis(0);
}

float QMC5883L::calibratedDataY()
{
   return calibratedDataAxis(1);
}

float QMC5883L::calibratedDataZ()
{
   return calibratedDataAxis(2);
}

QMC5883L::CalibratedDataAxes QMC5883L::calibratedDataAxes()
{
   CalibratedDataAxes calibratedDataAxes;

   calibratedDataAxes.x = calibratedDataAxis(0);
   calibratedDataAxes.y = calibratedDataAxis(1);
   calibratedDataAxes.z = calibratedDataAxis(2);
   
   return calibratedDataAxes;
}

QMC5883L::CalibratedDataArray QMC5883L::calibratedDataArray()
{
   CalibratedDataArray calibratedDataArray;

   calibratedDataArray.axes[0] = calibratedDataAxis(0);
   calibratedDataArray.axes[1] = calibratedDataAxis(1);
   calibratedDataArray.axes[2] = calibratedDataAxis(2);
   
   return calibratedDataArray;
}

float QMC5883L::azimuthZUp()
{
   float alpha;

   alpha = atan2(calibratedDataY(), calibratedDataX()); // atan2(y, x), not math. arctan2(x, y)! 

   return (alpha < 0 ? alpha + 2 * M_PI : alpha) * 180 / M_PI;
}

bool QMC5883L::dataReady()
{
   return registers[status_register] & bit_data_ready;
}

bool QMC5883L::dataOverflow()
{
   return registers[status_register] & bit_data_overflow;
}

bool QMC5883L::dataSkipped()
{
   return registers[status_register] & bit_data_skipped;
}

int QMC5883L::rawTemperature()
{
   return *((int16_t*)(registers + temperature_registers));
}

char QMC5883L::getPeriod()
{
   return registers[period_register];
}

QMC5883L::Error QMC5883L::lastError()
{
   return last_error;
}

bool QMC5883L::readRegisters(uint8_t register_first,  // Address of first register
                             size_t  register_count ) // Number of registers
{
   uint8_t status; 

   int outNb = i2c->reg_read(default_address, register_first, &registers[register_first], register_count);

   // Address the slave with read bit 1, read data into buffer and release the bus (STOP = true)
   if (outNb != register_count)
   {
      last_error = Error::i2c_request_partial;
      
      return false;
   }
   
   return true;
}

bool QMC5883L::writeRegisters(uint8_t register_first,  // Address of first register
                              size_t  register_count ) // Number of registers
{
   uint8_t register_address;

   for (register_address = register_first; 
        register_address < register_first + register_count;
        register_address++                                 )
   {
      if (!writeRegister(register_address)) return false;
   }
   
   return true;  
}

bool QMC5883L::writeRegister(uint8_t register_address) // Address of register
{
   uint8_t status;

   int outNb = i2c->reg_write(default_address, register_address, &registers[register_address], 1);

   if (outNb == 0) 
   {
      last_error = (Error)status;

      return false;
   }
   
   return true;  
}
