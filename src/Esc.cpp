#include "Esc.h"

Esc::Esc(const uint pin)
    : PIN(pin),
    SLICE_NUM(pwm_gpio_to_slice_num(pin)) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_set_function(pin, GPIO_FUNC_PWM);
    }

void Esc::setSpeedUs(float pulse_width_us){
    if(killSwitchOn){
        return;
    }

    if(pulse_width_us > MAX_US){
        return;
    }

    uint32_t system_clk = clock_get_hz(clk_sys);

    float divisor = (float)system_clk / (FREQ * 65536);

    pwm_set_clkdiv(SLICE_NUM, divisor);

    uint16_t wrap = 65535;
    pwm_set_wrap(SLICE_NUM, wrap);

    float duty_cycle = (float)pulse_width_us / 20000.0;

    pwm_set_gpio_level(PIN, wrap * duty_cycle);

    pwm_set_enabled(SLICE_NUM, true);

    current_us = pulse_width_us;
    return;
}

void Esc::setSpeed(float pulse_width_p){
    float range = MAX_US - MIN_US;
    setSpeedUs(((range / 100) * pulse_width_p) + MIN_US);
    return;
}

void Esc::stop() {
    setSpeedUs(0);
    return;
}

void Esc::useKillSwitch() {
    stop();
    killSwitchOn = true;
    return;
}

void Esc::arm() {
    setSpeedUs(0);
    sleep_ms(1000);
    setSpeedUs(MAX_US);
    sleep_ms(1000);
    setSpeedUs(MIN_US);
    sleep_ms(1000);
    return;
}

void Esc::calibrate(){
    setSpeedUs(MAX_US);
    sleep_ms(34400);
    setSpeedUs(MIN_US);
    sleep_ms(10000);
    setSpeedUs(0);
    sleep_ms(2000);
    setSpeedUs(MIN_US);
    sleep_ms(1000);
    return;
}