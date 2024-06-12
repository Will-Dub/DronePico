// -------------------------------------------------------------------------------------------------- //
// Robert's Smorgasbord 2022                                                                          //
// https://robertssmorgasbord.net                                                                     //
// https://www.youtube.com/channel/UCGtReyiNPrY4RhyjClLifBA                                           //
// QST QMC5883L 3-Axis Digital Compass and Arduino MCU – The Details (1) https://youtu.be/NTDS2Vmnr-4 //
// -------------------------------------------------------------------------------------------------- //

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c.h"

#ifndef QMC5883L_H
#define QMC5883L_H

class QMC5883L
{
   public:

   enum Mode : uint8_t // QMC5883L operation mode
   {
      standby    = 0b00000000, // Device takes no measurements, low power mode
      continuous = 0b00000001  // Device takes continuous measurements
   };

   enum OutputDataRate : uint8_t // QMC5883L output data rate
   {
      odr_10hz   = 0b00000000, // Device takes  10 measurements each second
      odr_50hz   = 0b00000100, // Device takes  50 measurements each second
      odr_100hz  = 0b00001000, // Device takes 100 measurements each second
      odr_200hz  = 0b00001100  // Device takes 200 measurements each second
   };

   enum FullScaleRange : uint8_t // QMC5883L full scale range
   {
      rng_2g     = 0b00000000, // 2 Gauss
      rng_8g     = 0b00010000  // 8 Gauss
   };

   enum OverSampleRate : uint8_t // QMC5883L's ADC's over sample rate
   {
      osr_512    = 0b00000000, // 512
      osr_256    = 0b01000000, // 256
      osr_128    = 0b10000000, // 128
      osr_64     = 0b11000000  //  64
   };

   struct Config // Complete QMC5883L configuration
   {
      Mode           mode;
      OutputDataRate output_data_rate;
      FullScaleRange full_scale_range;
      OverSampleRate over_sample_rate;
      bool        interrupt_disabled;
   };

   struct Calibration // Combined bias values and scale factors 
   {
      float bias[3];     // Bias vector
      float scale[3][3]; // Scale matrix ([Row][Column])
   };
   
   struct RawDataAxes // Axes raw data, returned by rawDataAxes()
   {
      int x;
      int y;
      int z;
   };

   struct RawDataArray // Axes raw data, returned by rawDataArray()
   {
      int axes[3];
   };

   struct CalibratedDataAxes // Axes scaled/calibrated data, returned by scaledDataAxes()
   {
      float x;
      float y;
      float z;
   };

   struct CalibratedDataArray // Axes scaled/calibrated data, returned by rawDataArray()
   {
      float axes[3];
   };

   enum Error : uint8_t // Type of error, returned by lastError().
   {
      i2c_buffer_overflow = 01, // Data too long to fit in transmit buffer
      i2c_address_nack    = 02, // Received NACK on transmit of address
      i2c_data_nack       = 03, // Received NACK on transmit of data
      i2c_other           = 04, // Other error
      i2c_request_partial = 05, // Received less than expected bytes on request
      qmc_data_overflow   = 06  // Data of magnetic sensors is out of range
   };


   // Constructor
   // -----------
   // TwoWire* wire:   TwoWire object to be used for communication,
   //                  wire.begin() has to be called before using the QMC5883L object.
   // uint8_t address: Optional, address of the QMC5883L on the I2C bus (default 0x0D).     
   QMC5883L(I2C* i2c);

   // begin()
   // -------
   // Call before using any other method! Performs a reset().
   // Returns true if successful, otherwise false (use getError() to retrive fault condition).
   bool begin();

   // reset()
   // -------
   // Performs a soft reset of the QMC5883L via I2C and a readConfig().
   // Returns true if successful, otherwise false (use getError() to retrive fault condition).   
   bool reset();

   // Read Configuration
   // ------------------
   // Read the current configuration of the QMC5883L via I2C into a local buffer.
   // Returns true if successful, otherwise false (use getError() to retrive fault condition).
   bool readConfig();

   // readData()
   // ------------
   // Reads the QMC5883L's data output incl. the status via I2C.
   // Returns true if successful, otherwise false (use lastError() to retrive fault condition).
   bool readData();

   // readStatus()
   // ------------
   // Reads the QMC5883L's status via I2C.
   // Returns true if successful, otherwise false (use lastError() to retrive fault condition).
   bool readStatus();

   // readTemperature()
   // -----------------
   // Reads the QMC5883L's temperature via I2C.
   // Returns true if successful, otherwise false (use lastError() to retrive fault condition).
   bool readTemperature();

   // readPeriod()
   // ------------
   // Reads the QMC5883L's set/reset period FBR register via I2C.
   // Returns true if successful, otherwise false (use lastError() to retrive fault condition).
   bool readPeriod();

   // Write Configuration
   // -------------------
   // Write the configuration in the local buffer via I2C to the QMC5883L.
   // Return true if successful, otherwise false (use getError() to retrive fault condition).
   bool writeConfig();

   // Get Configuration Methods
   // -------------------------
   // Get configuration or single configuration options from the local buffer.
   // To transfer the current configuration of the QMC5883L to the buffer use readConfig().
   Mode           getMode();
   OutputDataRate getOutputDataRate();
   FullScaleRange getFullScaleRange();
   OverSampleRate getOverSampleRate();
   bool        getInterruptDisabled();
   Config         getConfig();

   // Set Configuration Methods
   // -------------------------
   // Set configuration or single configuration options in the local buffer.
   // To actually transfer the configuration from the buffer to the QMC5883L use writeConfig().
   void setMode(Mode mode);
   void setOutputDataRate(OutputDataRate output_data_rate);
   void setFullScaleRange(FullScaleRange full_scale_range);
   void setOverSampleRate(OverSampleRate over_sample_rate);
   void setInterruptDisabled(bool interrupt_disabled);
   void setConfig(const Config& config);

   // Calibration Methods
   // -------------------
   // setCalibration() sets the min/max values used by the calibration and calls recalcCalibration().
   void setCalibration(const Calibration& calib);
   // resetCalibration() resets the min/max values used by the calibration and calls recalcCalibration(). 
   void resetCalibration();
   // recalcCalibration() calulates the calibration biases and scales from the current min/max values(). 
   //void recalcCalibration();

   // Raw data retrival methods
   // -------------------------
   // Return raw data from the last readData() call, if it was successful (returned true).
   // If the last readData() call failed (returned false), this data might be corrupted. 
   int          rawDataAxis(char axis); // axis = 0 (X) or 1 (Y) or 2 (Z) 
   int          rawDataX();
   int          rawDataY();
   int          rawDataZ();
   RawDataAxes  rawDataAxes();
   RawDataArray rawDataArray();

   // Calibrated data retrival methods
   // --------------------------------
   // Return calibrated data from the last readData() call, if it was successful (returned true).
   // If the last readData() call failed (returned false), this data might be corrupted.
   // If setCalibration() has not been used beforehand, the data is not calibrated. 
   float               calibratedDataAxis(char axis); // axis = 0 (X) or 1 (Y) or 2 (Z) 
   float               calibratedDataX();
   float               calibratedDataY();
   float               calibratedDataZ();
   CalibratedDataAxes  calibratedDataAxes();
   CalibratedDataArray calibratedDataArray();

   // Azimuth retrival methods
   // ------------------------
   // Return azimuth calculated from calibrated data (see there).
   float azimuthZUp(); // 0° with X pointing north

   // Status retrival methods
   // -----------------------
   // Return status data from the last readData() call, if it was successful (returned true).
   // If the last readData() call failed (returned false), the data might be corrupted.
   bool dataReady();
   bool dataOverflow(); // This will also cause readData() to fail (return false);
   bool dataSkipped();

   // Temperature retrival methods
   // ----------------------------
   // Return temperature from the last readTemperature() call, if it was successful (returned true).
   // If the last readTemperature() call failed (returned false), the temperature might be corrupted.   
   int rawTemperature();

   // Period retrival methods
   // -----------------------
   // Returns the period read by the last readPeriod() call, if it was successful (returned true).
   // If the last readPeriod() call failed (returned false), the period might be corrupted.   
   char getPeriod();

   // lastError()
   // -----------
   // Returns the last encountered error.
   Error lastError();

   private:

   static const uint8_t default_address = 0x0D; // Default I2C address of QMC5883L
   
   static const uint8_t data_registers        = 0x00; // Address of first data output register
   static const uint8_t status_register       = 0x06; // Address of status register
   static const uint8_t temperature_registers = 0x07; // Address of first temperature register
   static const uint8_t control_register_1    = 0x09; // Address of control register 1
   static const uint8_t control_register_2    = 0x0A; // Address of control register 2
   static const uint8_t period_register       = 0x0B; // Address of period register

   static const uint8_t bit_data_ready    = 0b00000001; // Within status register
   static const uint8_t bit_data_overflow = 0b00000010; // Within status register
   static const uint8_t bit_data_skipped  = 0b00000100; // Within status register
   static const uint8_t bits_mode             = 0b00000001; // Within control register 1
   static const uint8_t bits_output_data_rate = 0b00001100; // Within control register 1
   static const uint8_t bits_full_scale_range = 0b00010000; // Within control register 1
   static const uint8_t bits_over_sample_rate = 0b11000000; // Within control register 1
   static const uint8_t bit_interrupt_disabled = 0b00000001; // Within control register 2
   static const uint8_t bit_roll_over_enabled  = 0b01000000; // Within control register 2
   static const uint8_t bit_soft_reset         = 0b10000000; // Within control register 2

   // Recommended value for period register. Without setting the register to this value
   // the QMC5883L will not work properly (especially the temperature sensor)!
   static const uint8_t period_register_default = 0x01;

   I2C* i2c;            // I2C object
   uint8_t  registers[period_register + 1]; // Copies of QMC5883L registers
   Calibration  calibration;                    // Min/max values for calibration
   //long         biases[3];                      // Calibration biases for the axes
   //float        scales[3];                      // Calibration scales for the axes

   Error    last_error;      // Holds the last error encountered

   bool readRegisters(uint8_t register_first,    // Address of first register
                      size_t  register_count );  // Number of registers
   bool writeRegisters(uint8_t register_first,   // Address of first register
                       size_t  register_count ); // Number of registers
   bool writeRegister(uint8_t register_address); // Address of register
};

#endif