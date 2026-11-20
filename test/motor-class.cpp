#include "pico/stdlib.h"
#include "Motor.hpp"

#define MOTOR_IN1_PIN 2
#define MOTOR_IN2_PIN 3
#define MOTOR_PWM_PIN 0

int main() {
    
    PicoMotor motor(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_PWM_PIN);

    while(1){
        motor.set_speed(0.5f);
        sleep_ms(1000);
        motor.set_speed(-0.5f);
        sleep_ms(1000);
        motor.set_speed(0.3f);
        sleep_ms(1000);
        motor.set_speed(-0.3f);
        sleep_ms(1000);
    }

    return 0;
}
