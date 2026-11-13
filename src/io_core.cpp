#include "io_core.hpp"
#include "pico/multicore.h"

// actual definition of io_manager, this is required for extern IO_Manager io_manager to work
IO_Manager io_manager;

static int current_pass = 0;
static uint32_t last_theta = 0;

void poll_lidar() {
    RPLidarC1& lidar = io_manager.lidar;
    PolarPoint point;

    // this will poll the point when it is availible
    while (lidar.pollPoint(point)) {
        uint32_t theta_index = static_cast<uint32_t>(point.theta_deg);
        if (theta_index >= LIDAR_ANGLE_RANGE) {
            continue;
        }

        if (last_theta > 270 && theta_index < 90) {
            current_pass = (current_pass + 1) % NUM_LIDAR_SWEEPS;
        }
        last_theta = theta_index;

        LidarData& data = io_manager.theta_bufs[theta_index];
        mutex_enter_blocking(&data.mtx);
        data.r_data[current_pass] = point.r_mm;
        mutex_exit(&data.mtx);
    }
}


// this is the actual function that runs on core 1
void io_core_main() {
    while(true){
        if(io_manager.lidar.startScan()) break;
    } 

    // this will tell core 0 that core 1 is ready and is running
    multicore_fifo_push_blocking(1);

    while(true) {

        // main loop of io stuff 

        poll_lidar();

        if(multicore_fifo_rvalid()){ // this polling the stack between cores to see the end signal
            uint32_t signal = multicore_fifo_pop_blocking();   
            if(signal)
                break; 
        }
    }

    io_manager.lidar.stopScan();
}


void IO_Manager::run() {
    multicore_launch_core1(io_core_main);
    multicore_fifo_pop_blocking();  
}

void IO_Manager::signal_to_end() {
    multicore_fifo_push_blocking(1);
}

bool IO_Manager::init() {

    lidar.init(uart0, 1, 1);    
    
    return true;
}