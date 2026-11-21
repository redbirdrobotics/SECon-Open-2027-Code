#include "io_manager.hpp"
#include "packet.hpp"
#include "pico/multicore.h"
#include "io_core.hpp"

// actual definition of io_manager, this is required for extern IO_Manager io_manager to work
IO_Manager io_manager;

int IO_Manager::send_child_command(PicoCommand command) {
   
    int written = i2c_write_blocking(
        MASTER_I2C,
        I2C_SLAVE_ADDRESS,
        reinterpret_cast<const uint8_t*>(&command),
        sizeof(PicoCommand),
        false
    );
    
    if (written != sizeof(PicoCommand)) {
        return 1;
    }

    return 0;
}


void IO_Manager::run() {
    multicore_launch_core1(io_core_main);
    multicore_fifo_pop_blocking();  
}

void IO_Manager::signal_to_end() {
    multicore_fifo_push_blocking(1);
}

bool IO_Manager::init() {
    
    i2c_init(MASTER_I2C, I2C_BAUDRATE);
    gpio_set_function(MASTER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MASTER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MASTER_I2C_SDA_PIN);
    gpio_pull_up(MASTER_I2C_SCL_PIN);

    lidar.init(uart0, 12, 13);    
    for(int i = 0; i < LIDAR_ANGLE_RANGE; i++){
        mutex_init(&io_manager.theta_bufs[i].mtx);
    }
    
    return true;
}
