#include "io_core.hpp"
#include "pico/multicore.h"

// actual definition of io_manager, this is required for extern IO_Manager io_manager to work
IO_Manager io_manager;

// this is the actual function that runs on core 1
void io_core_main() {

}


void IO_Manager::run() {
    multicore_launch_core1(io_core_main);  
}

