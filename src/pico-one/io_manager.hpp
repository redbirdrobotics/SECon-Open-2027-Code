#ifndef IO_MANAGER_H__
#define IO_MANAGER_H__

#include "hardware/uart.h"
#include "pico/mutex.h"
#include "LiDAR.hpp"
#include "packet.hpp"

#define NUM_LIDAR_SWEEPS 5
#define LIDAR_ANGLE_RANGE 180

/*
    My idea is that each index theta will contain the n amount of magnitudes collected.
    This will make averaging the magnitudes much easier
*/

// using alias to make changing the type easier if nessesary
typedef float lidar_data_t;

typedef struct {
    lidar_data_t r_data[NUM_LIDAR_SWEEPS];
    mutex_t mtx;
} LidarData;



class IO_Manager {
    public:
        // initalizes all pins and values needed for IO
        bool init();
        // sets up core 1 and runs core 1
        void run();
        void signal_to_end();
        
        // sends a command to robin pico
        int send_child_command(PicoCommand command);
        // averages all the r's in theta_bufs, and puts the averaged data into average_theta_buf
        void average_lidar_data();
        RPLidarC1 lidar;
        LidarData theta_bufs[LIDAR_ANGLE_RANGE];
        lidar_data_t average_theta_buf[LIDAR_ANGLE_RANGE];
};


// global variable for io_manager to shared across threads
extern IO_Manager io_manager;    

#endif
