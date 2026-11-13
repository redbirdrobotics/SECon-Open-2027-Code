#ifndef IO_CORE_H__
#define IO_CORE_H__

#include "pico/mutex.h"

#define NUM_LIDAR_SWEEPS 5
#define LIDAR_ANGLE_RANGE 180

// using alias to make changing the type easier if nessesary
typedef uint8_t lidar_data_t;

typedef struct {
    lidar_data_t data_buf[LIDAR_ANGLE_RANGE];
    mutex_t mtx;
    
} LidarData;


class IO_Manager {
    public:
        // sets up core 1
        void run();
        void signal_to_end();
         
    private:
        LidarData data_bufs[NUM_LIDAR_SWEEPS];
};


// global variable for io_manager to shared across threads
extern IO_Manager io_manager;

#endif