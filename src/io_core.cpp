#include "io_core.hpp"
#include "pico/multicore.h"

// actual definition of io_manager, this is required for extern IO_Manager io_manager to work
IO_Manager io_manager;

// this is the actual function that runs on core 1
void io_core_main() {

    while(true) {

        // main loop of io stuff 

        if(multicore_fifo_rvalid()){ // this polling the stack between cores to see the end signal
            uint32_t signal = multicore_fifo_pop_blocking();   
            if(signal)
                break; 
        }
    }
}


void IO_Manager::run() {
    multicore_launch_core1(io_core_main);  
}

void IO_Manager::signal_to_end() {
    multicore_fifo_push_blocking(1);
}