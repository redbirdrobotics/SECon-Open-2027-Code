#ifndef ROBOT_CONST_HPP
#define ROBOT_CONST_HPP

//All of the pins are defined here for the various devices on the car
//    These lines haven't been desided on yet and will be changed as the project progresses
//    (aka, we did not discuss these pins in the meeting)

//Taken pins: 4, 5, 26, 27, 39, 40 for pico1
//Taken pins: 1, 2, 4-7, 9-12, 19, 20, 39, 40 for pico2
#define SERVO_PIN 4     // Pin connected to the servo motor
#define DCFR_PIN 5      // Pin connected to the front right DC motor
#define DCFL_PIN 18     // Pin connected to the front left DC motor
#define DCRR_PIN 19     // Pin connected to the rear right DC motor
#define DCRL_PIN 21     // Pin connected to the rear left DC motor
#define APRILTAG_PIN 23 // Pin connected to the AprilTag reader

//NOTE: pico 1 is the slave and pico 2 is the master. 
//      The master will be the one that controls the motors and the slave will be the one that 
//      controls the LiDAR and the AprilTag reader
#define LiDAR_PIN 16, 17    // Pin connected to the LiDAR sensor on pico1
#define LiDAR_PIN2    // Pin connected to the LiDAR sensor on pico2 (no pin?)
#define IMU_PIN      // Pin connected to the IMU sensor on pico1
#define IMU_PIN2     // Pin connected to the IMU sensor on pico2

//ZSL matrix definitions for the various coordinate systems used in the project
ZSL_MATRIX_DEF(global_coordinates, 3, 1);
ZSL_MATRIX_DEF(local_coordinates, 3, 1);
ZSL_MATRIX_DEF(cartesian_coordinates, 2, 1);
ZSL_MATRIX_DEF(polar_coordinates, 2, 1);

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 460800 // The baud rate for what were doing

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

//I2C defines
#define I2C_ID i2c0
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7


#endif