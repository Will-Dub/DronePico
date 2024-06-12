
#include "qmc5883l.h"

QMC5883L qmc5883l(&Wire);

void setup()
{
   Serial.begin(1000000);
   while(!Serial);
   
   Wire.begin();
   Wire.setClock(400000); // 400kHz I2C clock

   if (!qmc5883l.begin())
   {
      printError(qmc5883l.lastError(), "begin()");
   }
   else
   {
      printConfig(qmc5883l.getConfig());

      qmc5883l.setConfig( { QMC5883L::Mode::continuous,
                            QMC5883L::OutputDataRate::odr_50hz,
                            QMC5883L::FullScaleRange::rng_8g,
                            QMC5883L::OverSampleRate::osr_256,
                            true                               } );

      if (!qmc5883l.writeConfig())
      {
         printError(qmc5883l.lastError(), "writeConfig()");
      }
      else
      {
         if (!qmc5883l.readConfig())
         {
            printError(qmc5883l.lastError(), "readConfig()"); 
         }
         else
         {
            printConfig(qmc5883l.getConfig());
         }
      }
   }
}

void loop()
{
}

void printError(QMC5883L::Error error, char *error_location)
{
   Serial.print("Error ");
   Serial.print(error);
   Serial.print(" in ");
   Serial.println(error_location);
}

void printConfig(const QMC5883L::Config& config)
{
   Serial.println("Config:");
   Serial.print(  "  Mode               = "); 
   switch (config.mode)
   {
      case QMC5883L::Mode::standby:    Serial.println("Standby"); break;
      case QMC5883L::Mode::continuous: Serial.println("Continuous"); break;
   }
   Serial.print(  "  Output Data Rate   = ");
   switch (config.output_data_rate)
   {
      case QMC5883L::OutputDataRate::odr_10hz:  Serial.println("10Hz");  break;
      case QMC5883L::OutputDataRate::odr_50hz:  Serial.println("50Hz");  break;
      case QMC5883L::OutputDataRate::odr_100hz: Serial.println("100Hz"); break;
      case QMC5883L::OutputDataRate::odr_200hz: Serial.println("200Hz"); break;
   }
   Serial.print(  "  Full Scale Range   = ");
   switch (config.full_scale_range)
   {
      case QMC5883L::FullScaleRange::rng_2g: Serial.println("2G"); break;
      case QMC5883L::FullScaleRange::rng_8g: Serial.println("5G"); break;
   }
   Serial.print(  "  Over Sample Rate   = ");
   switch (config.over_sample_rate)
   {
      case QMC5883L::OverSampleRate::osr_64:  Serial.println("64");  break;
      case QMC5883L::OverSampleRate::osr_128: Serial.println("128"); break;
      case QMC5883L::OverSampleRate::osr_256: Serial.println("256"); break;
      case QMC5883L::OverSampleRate::osr_512: Serial.println("512"); break;
   }
   Serial.print(  "  Interrupt Disabled = ");
   Serial.println(config.interrupt_disabled ? "True" : "False");  
}