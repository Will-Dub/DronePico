// -------------------------------------------------------------------------------------------------- //
// Code based on Robert's Smorgasbord 2022                                                            //
// -------------------------------------------------------------------------------------------------- //

#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class I2C
{
   public:
    
      I2C(i2c_inst_t* i2c_port, const uint sda_pin, const uint scl_pin, int hz);

      uint64_t getLastReceiveTime();

      void setup();

      int reg_read(
      const uint addr,
                  const uint8_t reg,
                  uint8_t *buf,
                  const uint8_t nbytes);

      int reg_write(
         const uint addr, 
         const uint8_t reg, 
         const uint8_t *buf,
         const uint8_t nbytes);

   private:
      absolute_time_t lastReceiveTime;
      const int hz;
      const uint SDA_PIN;
      const uint SCL_PIN;
      i2c_inst_t *i2c_port;
   
};

#endif