/*
This cpp file is a custom library that is used to interface with the RP LiDAR C1. 
*/
#include "LiDAR.hpp"

// SLAMTEC Protocol Commands & Descriptors
static constexpr uint8_t CMD_SYNC_BYTE      = 0xA5;
static constexpr uint8_t CMD_STOP           = 0x25;
static constexpr uint8_t CMD_SCAN           = 0x20;
static constexpr uint8_t EXPECTED_DESC[7]   = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};


void RPLidarC1::init(uart_inst_t* uart_hw, uint tx_pin, uint rx_pin, uint32_t baudrate) {
    uart_ = uart_hw;
    tx_pin_ = tx_pin;
    rx_pin_ = rx_pin;
    baudrate_ = baudrate;
    uart_init(uart_, baudrate_);
    gpio_set_function(tx_pin_, GPIO_FUNC_UART);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);
    uart_set_hw_flow(uart_, false, false);
    uart_set_format(uart_, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart_, true);
}



bool RPLidarC1::startScan(uint32_t timeout_ms) {
    stopScan(); // Ensure clean state before starting

    const uint8_t cmd[2] = {CMD_SYNC_BYTE, CMD_SCAN};
    uart_write_blocking(uart_, cmd, 2);

    // Wait for and verify the 7-byte response descriptor: A5 5A 05 00 00 40 81
    uint8_t desc_idx = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (!time_reached(deadline)) {
        if (uart_is_readable(uart_)) {
            uint8_t b = uart_getc(uart_);
            if (b == EXPECTED_DESC[desc_idx]) {
                desc_idx++;
                if (desc_idx == 7) {
                    scanning_ = true;
                    byte_idx_ = 0;
                    return true;
                }
            } else {
                // Re-align if current byte matches start of descriptor
                desc_idx = (b == EXPECTED_DESC[0]) ? 1 : 0;
            }
        }
    }

    return false;
}

void RPLidarC1::stopScan() {
    const uint8_t cmd[2] = {CMD_SYNC_BYTE, CMD_STOP};
    uart_write_blocking(uart_, cmd, 2);
    scanning_ = false;
    byte_idx_ = 0;

    // Protocol requires >= 1 ms wait after STOP before issuing another command
    sleep_ms(2);

    // Flush remaining bytes in the RP2350 UART RX FIFO
    while (uart_is_readable(uart_)) {
        (void)uart_getc(uart_);
    }
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

                out_point.start_of_scan = (packet_buf_[0] & 0x01) != 0;
                out_point.quality       = (packet_buf_[0] >> 2);
                out_point.theta_deg     = static_cast<float>(raw_angle_q6) / 64.0f;
                out_point.r_mm          = static_cast<float>(raw_dist_q2) / 4.0f;
                out_point.valid         = (raw_dist_q2 > 0);

                return true;
            }
        }
    }

    return false;
}

void RPLidarC1::printPoint(const PolarPoint& pt) {
    printf("R:%05.2f, Theta:%05.2f\n", pt.r_mm, pt.theta_deg);
}