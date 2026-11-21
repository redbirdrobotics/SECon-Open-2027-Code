#ifndef PICO_TWO_COMMANDS_HPP
#define PICO_TWO_COMMANDS_HPP

#include "Motor.hpp"
#include "packet.hpp"

// this will contain functions for running the commands 
void run_command(PicoMotor &motor_A, PicoMotor &motor_B, PicoMotor &motor_C, PicoMotor &motor_D, PicoCommand& command);

#endif
