#include "pico/stdlib.h"
#include "pico-one/io_manager.hpp"

int main() {
    stdio_init_all();

    io_manager.init_lidar();
    io_manager.run();

    PicoCommand outgoing_command = {0};

    while (true) {

        io_manager.average_lidar_data();
        printf("lidar value: %f\n", io_manager.average_theta_buf[45]);
    }

    io_manager.signal_to_end();
}
