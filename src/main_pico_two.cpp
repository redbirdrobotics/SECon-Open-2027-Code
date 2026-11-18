#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "packet.hpp"


static uint8_t rx_buf[sizeof(PicoCommand)];
static volatile size_t rx_index = 0;
static volatile bool data_ready = false;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t byte = i2c_read_byte_raw(i2c);
            if (!data_ready && rx_index < sizeof(PicoCommand)) {
                rx_buf[rx_index++] = byte;
            }
            break;
        }

        case I2C_SLAVE_FINISH:
            if (rx_index == sizeof(PicoCommand)) {
                data_ready = true;
            } else {
                // Incomplete or oversized transfer; discard and reset
                rx_index = 0;
            }
            break;

        default:
            break;
    }
}

int main() {
    stdio_init_all();

    gpio_init(SLAVE_I2C_SDA_PIN);
    gpio_set_function(SLAVE_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SLAVE_I2C_SDA_PIN);

    gpio_init(SLAVE_I2C_SCL_PIN);
    gpio_set_function(SLAVE_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SLAVE_I2C_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    PicoCommand payload;


    while (true) {
        if (data_ready) {
            // Safely copy bytes into the aligned struct
            memcpy(&payload, rx_buf, sizeof(PicoCommand));

            // Release the buffer back to the ISR immediately
            rx_index = 0;
            data_ready = false;

            // Process all fields
        }
        tight_loop_contents();
    }
}
