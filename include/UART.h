#ifndef UART_H
#define UART_H

#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include <atomic>
#include <vector>
#include <optional>
#include "Message.h"

//Interface pour uart
class UART
{
    public:
    /**
     * Initialise le uart avec l'instance, le rate et les pins
     */
    UART(uart_inst_t *uart, uint baudrate, uint rx_pin, uint tx_pin);

    /*
    * Envoie un message qui fini avec \n(automatique)
    */
    void writeLine(const std::string& str);

    void write(const std::string& str);

    void writeBlock(const uint8_t* data, uint size);

    void readData();

    std::string getReceivedData();

    std::vector<std::string> getReceivedLines();

    void flush();

    bool isNewDataReceived();

    uint64_t get_last_receive_time();

    void writeMessage(const Message &message);

    std::optional<Message> getReceiveMessage();

    private:
    absolute_time_t last_receive_time;
    uart_inst_t *instance;
    uint baudrate;
    uint rx_pin;
    uint tx_pin;
    bool new_data_received;
    std::string received_data;
};

#endif