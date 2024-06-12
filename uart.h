#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include <atomic>
#include <vector>

#ifndef UART_H
#define UART_H

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

    void listenForData();

    std::string getReceivedData();

    std::vector<std::string> getReceivedLines();

    void flush();

    bool isNewDataReceived();

    private:
    uart_inst_t *instance;
    uint baudrate;
    uint rx_pin;
    uint tx_pin;
    std::atomic<bool> new_data_received;
    std::string received_data;
    critical_section_t critSec;
};

#endif