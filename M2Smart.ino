
#include <Arduino.h>
#include <Arduino_Due_SD_HSMCI.h>

#include "M2Smart.h"
#include "Serial.h"
#include "Logger.h"
#include "Comm.h"
#include "EEPROM.h"
#include "sys_io.h"
#include "settings.h"
#include "BUSDriver.h"

//file system on sdcard (HSMCI connected)
FileStore FS;

SettingsManager sm;
BUSDriver driver;
CommController comm(SerialCommon::USB, driver);

// We need to define this function to be able to pass a function pointer to the attachInterrupt function.. There is no good way around it
void SWCANCallback() { driver.SWCANIntHandler(); }

void frameCallback(CAN_FRAME &frame, BUSDriver::BUS whichBus) {
  // TODO: do something cool
  // if (isConnected) sendFrameToUSB(incoming, 1);
  // if (digToggleSettings.enabled && (digToggleSettings.mode & 1) && (digToggleSettings.mode & 4)) processDigToggleFrame(incoming);
  // if (SysSettings.logToFile) sendFrameToFile(incoming, 1);

  comm.sendFrameToUSB(frame, int(whichBus));
}

void setup() {
  setup_pins();
  initXBEE();

  SerialUSB.begin(115200);
  Serial.begin(115200);

  sm.readSettings();

  // setup_sys_io();

  // driver.setup();
  // driver.setFrameCallback(frameCallback);

  // enable interrupt for SWCAN
  // attachInterrupt(SWC_INT, SWCANCallback, FALLING); 
}

char rx_byte = 0;

void loop() {
  update_buttons();
  // driver.loop();

  comm.loop();

  if (digitalRead(Button2) == LOW) {
    // Enable forward mode to update firmware on XBEE
    SerialUSB.println("Entering infinite loop passthough mode");
    while (true) {
      if (SerialUSB.available() > 0) {
        rx_byte = SerialUSB.read();
        Serial.write(rx_byte);
      }
      if (Serial.available() > 0) {
        rx_byte = Serial.read();
        SerialUSB.write(rx_byte);
      }
      update_buttons();
    }
  }
  
  for (int loops = 0; Serial.available() && loops < 128; loops++) {
    rx_byte = Serial.read();
    SerialUSB.write(rx_byte);
  }

  for (int loops = 0; SerialUSB.available() && loops < 128; loops++) {
    rx_byte = SerialUSB.read();
    if (rx_byte == 'h') {
      update_uptime();
    }
  }
}