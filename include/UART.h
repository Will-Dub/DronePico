#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <atomic>
#include <vector>
#include <optional>
#include <algorithm>
#include "Message.h"

//Interface pour uart
class UART
{
    public:
        /**
         * Initialise le uart avec l'instance, le rate et les pins
         */
        UART(uart_inst_t *uart, uint baudrate, uint rx_pin, uint tx_pin);

        void flush();

        std::string getReceivedData();

        std::vector<std::string> getReceivedLines();

        std::optional<Message> getReceiveMessage();

        uint64_t getLastReceiveTime();

        bool isNewDataReceived();

        void readData();

        /*
        * Envoie un message qui fini avec \n(automatique)
        */
        void writeLine(const std::string& str);

        void write(const std::string& str);

        void writeBlock(const uint8_t* data, uint size);

        void writeMessage(const Message &message);

    private:
        absolute_time_t lastReceiveTime;
        uart_inst_t *instance;
        uint baudrate;
        uint rxPin;
        uint txPin;
        bool newDataReceived;
        std::string receivedData;
        const size_t MAX_BUFFER_SIZE = 100;
};

#endif