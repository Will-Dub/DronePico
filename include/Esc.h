#ifndef ESC_H
#define ESC_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

class Esc
{
   public:
    
        Esc(const uint pin);
    
        /**
         * Set the speed to a value in microsecond
         */
        void setSpeedUs(float pulse_width_us);

        /**
         * Set the speed to a value in percent
         */
        void setSpeed(float pulse_width_p);

        void stop();

        const float MAX_US = 2000;
        const float MIN_US = 1000;

        void arm();

        void calibrate();

        bool isInit = false;
    private:
        uint current_us;
        const uint PIN;
        const uint32_t FREQ = 50;
        const uint SLICE_NUM;
   
};

#endif