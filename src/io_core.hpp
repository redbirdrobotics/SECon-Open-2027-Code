#ifndef IO_CORE_H__
#define IO_CORE_H__

#include "hardware/uart.h"
#include "pico/mutex.h"
#include "LiDAR.hpp"

#define NUM_LIDAR_SWEEPS 5
#define LIDAR_ANGLE_RANGE 180

/*
    My idea is that each index theta will contain the n amount of magnitudes collected.
    This will make averaging the magnitudes much easier
*/

// using alias to make changing the type easier if nessesary
typedef uint8_t lidar_data_t;

typedef struct {
    lidar_data_t r_data[NUM_LIDAR_SWEEPS];
    mutex_t mtx;
} LidarData;


class IO_Manager {
    public:
        // sets up core 1
        bool init();
        void run();
        void signal_to_end();
         
    private:
        LidarData theta_bufs[LIDAR_ANGLE_RANGE];
        RPLidarC1 lidar;
};


// global variable for io_manager to shared across threads
extern IO_Manager io_manager;    

#endif