#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial soft_serial(7, 8);  // For debug output (connect a serial adapter here)
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;         // Dynamixel Shield DIR pin

const uint8_t CURRENT_ID = 1;
const uint8_t NEW_ID = 4;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (dxl.ping(CURRENT_ID)) {
    DEBUG_SERIAL.println("Servo found!");
    dxl.torqueOff(CURRENT_ID);

    if (dxl.setID(CURRENT_ID, NEW_ID)) {
      DEBUG_SERIAL.println("ID changed to 4!");
    } else {
      DEBUG_SERIAL.println("Failed to change ID.");
    }
  } else {
    DEBUG_SERIAL.println("Ping failed. Check wiring.");
  }
}

void loop() {}