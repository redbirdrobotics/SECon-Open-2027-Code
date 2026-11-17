
#include <Arduino.h>

// Use Hardware Serial 1 on Teensy 4.1 (Pins 0 and 1)
#define LidarSerial Serial1

// Slamtec standard request byte markers
const uint8_t SYNC_BYTE = 0xA5;
const uint8_t START_SCAN = 0x20;

// Parser structure for the 5-byte data packet
uint8_t packetBuffer[5];
int packetIndex = 0;

// Function declarations
void sendStartScanCommand();

void setup() {
  // Initialize USB Serial Monitor
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for Serial Monitor with timeout

  // C1 uses a strict 460800 baud rate over TTL UART
  LidarSerial.begin(460800);
  
  Serial.println("Teensy 4.1 Initializing RPLidar C1...");
  delay(1000); 

  // The C1 motor starts and scans automatically via UART commands
  sendStartScanCommand();
}

void loop() {
  while (LidarSerial.available() > 0) {
    uint8_t currentByte = LidarSerial.read();

    // Look for a valid 5-byte measurement packet frame
    if (packetIndex == 0) {
      // Byte 0 contains the inverse of the start bit in bit 1, and bit 0 is always 1
      // Valid data point synchronization checks
      bool bit0 = currentByte & 0x01;
      bool bit1 = (currentByte >> 1) & 0x01;
      
      if (bit0 != bit1) { // Check for proper framing sync
        packetBuffer[packetIndex++] = currentByte;
      }
    } 
    else if (packetIndex == 1) {
      // Byte 1 check: Bit 0 must always be 1 for a valid data point packet
      if (currentByte & 0x01) {
        packetBuffer[packetIndex++] = currentByte;
      } else {
        packetIndex = 0; // Invalid packet sequence, reset
      }
    } 
    else {
      packetBuffer[packetIndex++] = currentByte;

      // Full 5-byte structure successfully populated
      if (packetIndex >= 5) {
        // Parse Quality, Angle, and Distance
        uint8_t quality = packetBuffer[0] >> 2;
        
        // Extract 15-bit Angle data 
        uint16_t rawAngle = (packetBuffer[1] >> 1) | (packetBuffer[2] << 7);
        float angle = rawAngle / 64.0; // Scaled to degrees
        
        // Extract 16-bit Distance data
        uint16_t rawDistance = packetBuffer[3] | (packetBuffer[4] << 8);
        float distance = rawDistance / 4.0; // Scaled to millimeters

        // Output real-time coordinates if quality is reliable
        if (quality > 0 && distance > 0) {
          Serial.print("Angle: ");
          Serial.print(angle, 2);
          Serial.print("°\tDistance: ");
          Serial.print(distance, 1);
          Serial.print("mm\tQuality: ");
          Serial.println(quality);
        }

        packetIndex = 0; // Reset for next measurement frame
      }
    }
  }
}

// Forces the Lidar out of IDLE and into standard scanning mode
void sendStartScanCommand() {
  Serial.println("Sending SCAN request sequence to C1...");
  LidarSerial.write(SYNC_BYTE);
  LidarSerial.write(START_SCAN);
}
