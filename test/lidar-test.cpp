#include "pico/stdlib.h"
#include "pico-one/io_manager.hpp"

int main() {
    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    io_manager.init_lidar();
    io_manager.run();

    PicoCommand outgoing_command = {0};

    while (true) {

        sleep_ms(100);
        gpio_put(LED_PIN, 1);
        sleep_ms(100);
        gpio_put(LED_PIN, 0);
        io_manager.average_lidar_data();
        printf("forward: %f left: %f right: %f backwards: %f\n", io_manager.average_theta_buf[0], io_manager.average_theta_buf[270], io_manager.average_theta_buf[90],  io_manager.average_theta_buf[180]);
    }

    io_manager.signal_to_end();
}
