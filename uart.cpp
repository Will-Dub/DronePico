#include "uart.h"

UART::UART(uart_inst_t *uart_p, uint baudrate_p, uint rx_pin_p, uint tx_pin_p):
    instance(uart_p),
    baudrate(baudrate_p),
    rx_pin(rx_pin_p),
    tx_pin(tx_pin_p),
    new_data_received(false)
    {
        uart_init(instance, baudrate);
        gpio_set_function(tx_pin, GPIO_FUNC_UART);
        gpio_set_function(rx_pin, GPIO_FUNC_UART);

        uart_set_format(instance, 8, 1, UART_PARITY_NONE);

        uart_set_fifo_enabled(instance, true);
    }

void UART::writeLine(const std::string& str) {
    for (char c : str) {
        uart_putc(instance, c);
    }
    uart_putc(instance, '\n');
}

void UART::write(const std::string& str) {
    for (char c : str) {
        uart_putc(instance, c);
    }
}

std::string UART::getReceivedData() {
    std::string received_data_return = "";

    received_data_return = received_data;

    new_data_received = false;
    received_data.clear();

    return received_data_return;
}

std::vector<std::string> UART::getReceivedLines() {
    std::vector<std::string> lines;
    
    size_t pos = 0;
    while ((pos = received_data.find('\n')) != std::string::npos) {
        lines.push_back(received_data.substr(0, pos));
        received_data.erase(0, pos + 1);
    }
    
    new_data_received = !received_data.empty();
    return lines;
}

void UART::readData() {
    if(uart_is_readable(instance)){
        while(uart_is_readable(instance)) {
            char c = uart_getc(instance);
            received_data += c;
        }
        new_data_received = true;
    }
}

void UART::flush() {
    while (uart_is_writable(instance) == 0);
}

bool UART::isNewDataReceived() {
    return new_data_received;
}