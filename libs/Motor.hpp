#ifndef MOTOR_PICO_HPP
#define MOTOR_PICO_HPP

#include "hardware/gpio.h"

class PicoMotor {
    public:
        PicoMotor(uint n1_pin, uint n2_pin, uint pwm_pin); 
        void stop();
        void set_speed(float speed);
    private:
        uint n1_pin; 
        uint n2_pin; 
        uint pwm_pin; 
        uint num_slice;
        uint channel;
};


#endif
