#ifndef ROBOT_CONST_HPP
#define ROBOT_CONST_HPP


//Taken pins: 4, 5, 26, 27, 39, 40 for pico1
//Taken pins: 1, 2, 4-7, 9-12, 19, 20, 39, 40 for pico2
#define SERVO_PIN 4     // Pin connected to the servo motor
#define DCFR_PIN 5      // Pin connected to the front right DC motor
#define DCFL_PIN 18     // Pin connected to the front left DC motor
#define DCRR_PIN 19     // Pin connected to the rear right DC motor
#define DCRL_PIN 21     // Pin connected to the rear left DC motor
#define APRILTAG_PIN 23 // Pin connected to the AprilTag reader


#define LIDAR_PIN_RX 12    
#define LIDAR_PIN_TX 13
#define IMU_PIN_SCL      
#define IMU_PIN_SDA     

//ZSL matrix definitions for the various coordinate systems used in the project
ZSL_MATRIX_DEF(global_coordinates, 3, 1);
ZSL_MATRIX_DEF(local_coordinates, 3, 1);
ZSL_MATRIX_DEF(cartesian_coordinates, 2, 1);
ZSL_MATRIX_DEF(polar_coordinates, 2, 1);


#endif