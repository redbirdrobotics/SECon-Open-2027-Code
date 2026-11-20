#include "Motor.hpp"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

PicoMotor::PicoMotor(uint n1_pin, uint n2_pin, uint pwm_pin) { 
    // Initialize direction pins
    gpio_init(n1_pin);
    gpio_set_dir(n1_pin, GPIO_OUT);
    gpio_init(n2_pin);
    gpio_set_dir(n2_pin, GPIO_OUT);

    // Initialize PWM pin
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    num_slice = pwm_gpio_to_slice_num(pwm_pin);
    channel = pwm_gpio_to_channel(pwm_pin);

    // Configure PWM frequency (~1.9 kHz at 125 MHz sys_clk)
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, 65535);
    pwm_init(num_slice, &config, true);

    stop();
}


void PicoMotor::set_speed(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (speed > 0.0f) {
        gpio_put(n1_pin, 1);
        gpio_put(n2_pin, 0);
    } else if (speed < 0.0f) {
        gpio_put(n1_pin, 0);
        gpio_put(n2_pin, 1);
        speed = -speed; // Make positive for duty cycle calculation
    } else {
        stop();
        return;
    }

    uint16_t level = static_cast<uint16_t>(speed * 65535.0f);
    pwm_set_chan_level(num_slice, channel, level);
}

void PicoMotor::stop() {
    gpio_put(n1_pin, 0);
    gpio_put(n2_pin, 0);
    pwm_set_chan_level(num_slice, channel, 0);
}
