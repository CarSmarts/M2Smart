
#include <Arduino.h>
#include <Arduino_Due_SD_HSMCI.h>
#include "SPI.h"

#include "M2Smart.h"
#include "Logger.h"
#include "Comm.h"
#include "EEPROM.h"
#include "sys_io.h"
#include "settings.h"
#include "BUSDriver.h"

// #define TEST
// #include "M2Smart_unit.h"

//file system on sdcard (HSMCI connected)
FileStore FS;

SettingsManager sm;
BUSDriver *driver;
CommController *comm;

// We need to define this function to be able to pass a function pointer to the attachInterrupt function.. There is no good way around it
void SWCANCallback() { driver->SWCANIntHandler(); }

void frameCallback(CAN_FRAME &frame, BUSDriver::BUS whichBus) {
  // TODO: do something cool
  // if (isConnected) sendFrameToUSB(incoming, 1);
  // if (digToggleSettings.enabled && (digToggleSettings.mode & 1) && (digToggleSettings.mode & 4)) processDigToggleFrame(incoming);
  // if (SysSettings.logToFile) sendFrameToFile(incoming, 1);

  comm->sendFrameToUSB(frame, int(whichBus));
}

#ifndef TEST
void setup() {
  setup_pins();
  sys_early_setup();

  Serial.begin(115200);
  // Wire.begin(); // for now disable EEPROM Access
  SPI.begin();

  sm.readSettings();
  setup_sys_io();

  driver = new BUSDriver();
  comm = new CommController(driver);

  driver->setup();
  driver->setFrameCallback(frameCallback);

  // enable interrupt for SWCAN
  attachInterrupt(SWC_INT, SWCANCallback, FALLING);
}

char rx_byte = 0;
void loop() {
  update_buttons();

  driver->loop();
  comm->checkBuffer();

  for (int loops = 0; Serial.available() && loops < 128; loops++) {
    rx_byte = Serial.read();
    comm->read(&Serial, rx_byte);
  }

  for (int loops = 0; SerialUSB.available() && loops < 128; loops++) {
    rx_byte = SerialUSB.read();
    comm->read(&SerialUSB, rx_byte);
  }
}
#endif