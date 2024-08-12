#include "I2C.h"
#include <cstdint>

I2C::I2C(i2c_inst_t* i2c_port, const uint SDA_PIN, const uint SCL_PIN, int hz)
    : i2c_port(i2c_port),
    SDA_PIN(SDA_PIN),
    SCL_PIN(SCL_PIN),
    hz(hz){}

void I2C::setup(){
    i2c_init(i2c_port, hz);

    // Initialize I2C pins
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

// Write 1 byte to the specified register
int I2C::reg_write(
                const uint addr, 
                const uint8_t reg, 
                const uint8_t *buf,
                const uint8_t nbytes) {

    uint8_t *msg = new uint8_t[nbytes + 1];

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    int result = i2c_write_blocking(i2c_port, addr, msg, (nbytes + 1), false);

    delete[] msg;

    // Write data to register(s) over I2C
    if(result == PICO_ERROR_GENERIC){
        return 0;
    }

    return nbytes;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int I2C::reg_read(
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {
    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    int write_result = i2c_write_blocking(i2c_port, addr, &reg, 1, true);
    if(write_result == PICO_ERROR_GENERIC){
        return 0;
    }

    int result = i2c_read_blocking(i2c_port, addr, buf, nbytes, false);
    if(result == PICO_ERROR_GENERIC){
        return 0;
    }
    
    lastReceiveTime = get_absolute_time();
    return nbytes;
}

uint64_t I2C::getLastReceiveTime() {
    return to_us_since_boot(lastReceiveTime);
}
