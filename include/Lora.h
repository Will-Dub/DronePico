#ifndef LORA_H
#define LORA_H

#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include "LoRa-RP2040.h"
#include "DataPacket.h"

class Lora
{
   public:
    
      Lora(const uint pin);

      bool init();

      void sendDataPacket(DataPacket dataPacket);

      std::optional<DataPacket> getReceivedDataPacket();

   private:
      const uint PIN;
      const size_t MAX_BUFFER_SIZE = 256;
      string receivedData = "";
   
};

#endif