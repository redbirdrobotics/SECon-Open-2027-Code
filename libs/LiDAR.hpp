
/*
    This library is a custom library that is currently meant to be used for the RP LiDAR C1.
    If we decide to use a different LiDAR, this library will be updated to support the new LiDAR.
    In the library there will be many function that will interface and process the data from the given LiDAR.
    From formatting the incoming data to creating a histogram of the data to give a more accurate point cloud.
*/
#ifndef RPLIDAR_C1_HPP
#define RPLIDAR_C1_HPP

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <cstdint>
#include <cstdio>

struct PolarPoint {
    float r_mm;          // Radial distance in millimeters
    float theta_deg;     // Angle in degrees [0.00, 360.00)
    uint8_t quality;     // Signal reflectivity/quality (0-63)
    bool start_of_scan;  // True if this point begins a new 360-degree sweep
    bool valid;          // True if distance > 0 and framing bits are valid
};

class RPLidarC1 {
public:
    static constexpr uint32_t DEFAULT_BAUDRATE = 460800;

    RPLidarC1() = default;

    // Initializes UART hardware and GPIO pins
    void RPLidarC1::init(uart_inst_t* uart_hw, uint tx_pin, uint rx_pin, uint32_t baudrate);

    // Sends 0xA5 0x20 and verifies the 7-byte response descriptor
    bool startScan(uint32_t timeout_ms = 1000);

    // Sends 0xA5 0x25 and flushes the hardware RX FIFO
    void stopScan();

    // Non-blocking byte-stream parser. Returns true when a valid 5-byte point is assembled.
    bool pollPoint(PolarPoint& out_point);

    // Prints the polar point in the exact format "R:##.##, Theta:##.##\n"
    static void printPoint(const PolarPoint& pt);

private:
    uart_inst_t* uart_;
    uint tx_pin_;
    uint rx_pin_;
    uint32_t baudrate_;
    bool scanning_;

    uint8_t packet_buf_[5];
    uint8_t byte_idx_;

    // Validates RPLIDAR Standard Scan 5-byte framing rules
    static bool isValidHeaderByte0(uint8_t b0);
    static bool isValidHeaderByte1(uint8_t b1);
};

#endif // RPLIDAR_C1_HPP