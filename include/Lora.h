#ifndef LORA_PICO_H
#define LORA_PICO_H

#include <stdio.h>
#include "pico/stdlib.h"
#include <string>
#include "LoRa-RP2040.h"
#include "DataPacket.h"
#include <optional>
#include <algorithm>

class Lora
{
   public:
    
      Lora(const long frequency);

      bool init();

      void writeDataPacket(DataPacket dataPacket);

      void readData();

      std::optional<DataPacket> getReceivedDataPacket();

      bool getIsNewDataReceived();

      uint64_t getLastReceiveTime();

   private:
      const long FREQUENCY;
      const size_t MAX_BUFFER_SIZE = 256;
      std::string receivedData = "";
      bool isLoraInitialized = false;
      bool isNewDataReceived;
      absolute_time_t lastReceiveTime;
};

#endif