/*
This cpp file is a custom library that is used to interface with the RP LiDAR C1. 
*/
#include "LiDAR.hpp"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"

// SLAMTEC Protocol Commands & Descriptors
static constexpr uint8_t CMD_SYNC_BYTE      = 0xA5;
static constexpr uint8_t CMD_STOP           = 0x25;
static constexpr uint8_t CMD_SCAN           = 0x20;
static constexpr uint8_t EXPECTED_DESC[7]   = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};


void RPLidarC1::init(uart_inst_t* uart_hw, uint tx_pin, uint rx_pin, uint32_t baudrate) {
    uart_ = uart_hw;

    // On RP2040/RP2350, UART TX is always an even GPIO and UART RX is always an odd GPIO.
    // Handle cases where identical pins (e.g. 1, 1) or swapped pins are passed.
    if (tx_pin == rx_pin) {
        tx_pin_ = tx_pin & ~1u;
        rx_pin_ = rx_pin | 1u;
    } else if ((tx_pin & 1u) && !(rx_pin & 1u)) {
        tx_pin_ = rx_pin;
        rx_pin_ = tx_pin;
    } else {
        tx_pin_ = tx_pin;
        rx_pin_ = rx_pin;
    }

    baudrate_ = baudrate;
    scanning_ = false;
    byte_idx_ = 0;

#ifdef uart_default
    // If stdio_uart was enabled by stdio_init_all() on the same UART, disable it
    // so printf() does not inject debug text onto the LiDAR UART line, and unmux
    // the default stdio UART pins (GP0/GP1) so they don't conflict with GP12/GP13.
    if (uart_ == uart_default) {
        stdio_set_driver_enabled(&stdio_uart, false);
#ifdef PICO_DEFAULT_UART_TX_PIN
        if (tx_pin_ != PICO_DEFAULT_UART_TX_PIN) {
            gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_NULL);
        }
#endif
#ifdef PICO_DEFAULT_UART_RX_PIN
        if (rx_pin_ != PICO_DEFAULT_UART_RX_PIN) {
            gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_NULL);
        }
#endif
    }
#endif

    uart_init(uart_, baudrate_);
    uart_set_hw_flow(uart_, false, false);
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_, true);
    uart_set_translate_crlf(uart_, false);

    gpio_set_function(tx_pin_, UART_FUNCSEL_NUM(uart_, tx_pin_));
    gpio_set_function(rx_pin_, UART_FUNCSEL_NUM(uart_, rx_pin_));
    gpio_pull_up(rx_pin_);

    while (uart_is_readable(uart_)) {
        (void)uart_getc(uart_);
    }
    uart_get_hw(uart_)->rsr = 0;
}



bool RPLidarC1::startScan(uint32_t timeout_ms) {
    // Matches prepare_serial_port() in test.py: send STOP (0xA5 0x25), wait 100ms, flush RX
    stopScan();

    const uint8_t cmd[2] = {CMD_SYNC_BYTE, CMD_SCAN};
    uart_write_blocking(uart_, cmd, 2);
    uart_tx_wait_blocking(uart_);

    // Wait for the 7-byte response descriptor: A5 5A 05 00 00 40 81
    // Also detect if the LiDAR is already streaming valid 5-byte scan packets
    uint8_t desc_idx = 0;
    uint8_t sync_buf[5];
    uint8_t sync_byte_idx = 0;
    uint8_t valid_packets = 0;
    uint32_t rx_count = 0;

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (!time_reached(deadline)) {
        if (uart_is_readable(uart_)) {
            uint8_t b = uart_getc(uart_);
            rx_count++;

            // 1. Check for standard 7-byte scan descriptor
            if (b == EXPECTED_DESC[desc_idx]) {
                desc_idx++;
                if (desc_idx == 7) {
                    scanning_ = true;
                    byte_idx_ = 0;
                    return true;
                }
            } else {
                desc_idx = (b == EXPECTED_DESC[0]) ? 1 : 0;
            }

            // 2. Fallback: if LiDAR is already streaming 5-byte scan packets,
            // verify 3 consecutive valid packets so we don't stall if descriptor was missed
            if (sync_byte_idx == 0) {
                if (isValidHeaderByte0(b)) {
                    sync_buf[0] = b;
                    sync_byte_idx = 1;
                } else {
                    valid_packets = 0;
                }
            } else if (sync_byte_idx == 1) {
                if (isValidHeaderByte1(b)) {
                    sync_buf[1] = b;
                    sync_byte_idx = 2;
                } else {
                    valid_packets = 0;
                    if (isValidHeaderByte0(b)) {
                        sync_buf[0] = b;
                        sync_byte_idx = 1;
                    } else {
                        sync_byte_idx = 0;
                    }
                }
            } else {
                sync_buf[sync_byte_idx++] = b;
                if (sync_byte_idx == 5) {
                    sync_byte_idx = 0;
                    uint16_t raw_angle_q6 = ((static_cast<uint16_t>(sync_buf[2]) << 7) |
                                             (static_cast<uint16_t>(sync_buf[1]) >> 1));
                    if (raw_angle_q6 < (360u * 64u)) {
                        valid_packets++;
                        if (valid_packets >= 3) {
                            scanning_ = true;
                            byte_idx_ = 0;
                            return true;
                        }
                    } else {
                        valid_packets = 0;
                    }
                }
            }
        } else {
            tight_loop_contents();
        }
    }

    printf("[LiDAR] startScan timeout (TX=GP%u, RX=GP%u, rx_bytes=%lu)\n",
           tx_pin_, rx_pin_, static_cast<unsigned long>(rx_count));
    return false;
}

void RPLidarC1::stopScan() {
    const uint8_t cmd[2] = {CMD_SYNC_BYTE, CMD_STOP};
    uart_write_blocking(uart_, cmd, 2);
    uart_tx_wait_blocking(uart_);
    scanning_ = false;
    byte_idx_ = 0;

    // RPLIDAR C1 needs ~100 ms after STOP to return to IDLE (same as time.sleep(0.1) in test.py).
    // Actively drain the RX FIFO during this window so the hardware FIFO does not overflow.
    absolute_time_t stop_wait = make_timeout_time_ms(100);
    while (!time_reached(stop_wait)) {
        while (uart_is_readable(uart_)) {
            (void)uart_getc(uart_);
        }
    }

    // Drain any leftover bytes currently in the hardware FIFO
    while (uart_is_readable(uart_)) {
        (void)uart_getc(uart_);
    }
    uart_get_hw(uart_)->rsr = 0;
}

bool RPLidarC1::isValidHeaderByte0(uint8_t b0) {
    // Bit 0 is Start Flag (S), Bit 1 is Inverse Start Flag (!S).
    // Valid packets strictly satisfy (S ^ !S) == 1.
    uint8_t s  = b0 & 0x01;
    uint8_t ns = (b0 >> 1) & 0x01;
    return (s ^ ns) == 1;
}

bool RPLidarC1::isValidHeaderByte1(uint8_t b1) {
    // Bit 0 of Byte 1 is the Check Bit (C), which is permanently 1
    return (b1 & 0x01) == 1;
}

bool RPLidarC1::pollPoint(PolarPoint& out_point) {
    if (!scanning_) {
        return false;
    }

    while (uart_is_readable(uart_)) {
        uint8_t b = uart_getc(uart_);

        if (byte_idx_ == 0) {
            if (isValidHeaderByte0(b)) {
                packet_buf_[0] = b;
                byte_idx_ = 1;
            }
        } else if (byte_idx_ == 1) {
            if (isValidHeaderByte1(b)) {
                packet_buf_[1] = b;
                byte_idx_ = 2;
            } else {
                // Frame alignment lost; check if current byte is a valid Byte 0
                if (isValidHeaderByte0(b)) {
                    packet_buf_[0] = b;
                    byte_idx_ = 1;
                } else {
                    byte_idx_ = 0;
                }
            }
        } else {
            packet_buf_[byte_idx_++] = b;

            if (byte_idx_ == 5) {
                byte_idx_ = 0;

                // Decode 5-byte Standard Scan Data Response Packet (Q6 angle, Q2 distance)
                uint16_t raw_angle_q6 = ((static_cast<uint16_t>(packet_buf_[2]) << 7) |
                                         (static_cast<uint16_t>(packet_buf_[1]) >> 1));
                uint16_t raw_dist_q2  = ((static_cast<uint16_t>(packet_buf_[4]) << 8) |
                                          static_cast<uint16_t>(packet_buf_[3]));

                if (raw_angle_q6 >= (360u * 64u)) {
                    // Angle >= 360 deg indicates false frame alignment; re-sync from packet_buf_[1..4]
                    for (uint8_t i = 1; i < 5; ++i) {
                        uint8_t nb = packet_buf_[i];
                        if (byte_idx_ == 0) {
                            if (isValidHeaderByte0(nb)) {
                                packet_buf_[0] = nb;
                                byte_idx_ = 1;
                            }
                        } else if (byte_idx_ == 1) {
                            if (isValidHeaderByte1(nb)) {
                                packet_buf_[1] = nb;
                                byte_idx_ = 2;
                            } else if (isValidHeaderByte0(nb)) {
                                packet_buf_[0] = nb;
                                byte_idx_ = 1;
                            } else {
                                byte_idx_ = 0;
                            }
                        } else {
                            packet_buf_[byte_idx_++] = nb;
                        }
                    }
                    continue;
                }

                out_point.start_of_scan = (packet_buf_[0] & 0x01) != 0;
                out_point.quality       = (packet_buf_[0] >> 2);
                out_point.theta_deg     = static_cast<float>(raw_angle_q6) / 64.0f;
                out_point.r_mm          = static_cast<float>(raw_dist_q2) / 4.0f;
                out_point.valid         = (raw_dist_q2 > 0);

                if (out_point.valid) {
                    return true;
                }
            }
        }
    }

    return false;
}

void RPLidarC1::printPoint(const PolarPoint& pt) {
    printf("R:%05.2f, Theta:%05.2f\n", pt.r_mm, pt.theta_deg);
}