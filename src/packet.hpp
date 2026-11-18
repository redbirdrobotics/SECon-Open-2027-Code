#ifndef ROBOT_PACKET_HPP
#define ROBOT_PACKET_HPP

#include "pico/stdlib.h"
/*
This will eventually hold a struct that will contain motor, servo, and whatever other controls from pico one to pico two
*/

#define SLAVE_I2C_SDA_PIN 0 
#define SLAVE_I2C_SCL_PIN 0

#define MASTER_I2C_SDA_PIN 0
#define MASTER_I2C_SCL_PIN 0

// IPC meaning inter-pico-communication
#define IPC_I2C i2c0

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

typedef struct __attribute__((packed)){
    float r;
    float theta;
} PicoCommand;



#endif
