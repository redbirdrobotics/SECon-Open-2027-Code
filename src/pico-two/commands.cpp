#include "packet.hpp"
#include "pico/stdlib.h"
#include "pico-two/commands.hpp"

void run_command(PicoMotor &motor_A, PicoMotor &motor_B, PicoMotor &motor_C, PicoMotor &motor_D, PicoCommand& command) {
    
    // Process all fields
    motor_A.set_speed(1.0f);
    motor_B.set_speed(1.0f);
    motor_C.set_speed(1.0f);
    motor_D.set_speed(1.0f);
    sleep_ms(100);
    motor_A.set_speed(-0.1286f);
    motor_B.set_speed(0.1286f);
    motor_C.set_speed(0.1286f);
    motor_D.set_speed(-0.1286f);
    sleep_ms(100);
    motor_A.set_speed(0.0f);
    motor_B.set_speed(-0.8f);
    motor_C.set_speed(0.0f);
    motor_D.set_speed(-0.8f);
    sleep_ms(100);
    motor_A.set_speed(-0.8321f);
    motor_B.set_speed(-0.7679f);
    motor_C.set_speed(-0.7679f);
    motor_D.set_speed(-0.8321f);
    sleep_ms(100);
    motor_A.set_speed(0.0f);
    motor_B.set_speed(0.0f);
    motor_C.set_speed(0.0f);
    motor_D.set_speed(0.0f);
}
