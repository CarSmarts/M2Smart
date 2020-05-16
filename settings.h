/*
 * config.h
 *
 * allows the user to configure static parameters.
 *
 * Note: Make sure with all pin defintions of your hardware that each pin number is
 *       only defined once.

 Copyright (c) 2013-2016 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *      Author: Michael Neuweiler
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "due_can.h"
#include "EEPROM.h"

//buffer size for SDCard - Sending canbus data to the card. Still allocated even for GEVCU but unused in that case
//This is a large buffer but the sketch may as well use up a lot of RAM. It's there.
//This value is picked up by the SD card library and not directly used in the GVRET code.
#define BUF_SIZE    512

//size to use for buffering writes to the USB bulk endpoint
//This is, however, directly used.
#define SER_BUFF_SIZE       4096

//maximum number of microseconds between flushes to the USB port.
//The host should be polling every 1ms or so and so this time should be a small multiple of that
#define SER_BUFF_FLUSH_INTERVAL 2000

#define CFG_BUILD_NUM   345
#define CFG_VERSION "M2Smart Alpha April 15 2020"
#define EEPROM_ADDR     0x50
#define EEPROM_VER      0x10

//  (!!! else comment out !!!)
#define _M2IO   // define to enable the use of the M2_12VIO library & the safe use of the I/O pins as well as PWM in M2RET
#define MARK_LIMIT  6   //# of our analog input pins to use for marking. Defaults to all of them. Send voltage to pin to trigger it

#define NUM_ANALOG  NUM_ANALOG_INPUTS   // we get the number of analogue inputs from variant.h
#define NUM_DIGITAL 6   // Number of digital inputs on the M2 (sudo inputs/Analogue inputs)
#define NUM_OUTPUT  6   // Number of digital outputs on the M2


// Define the Digital Outputs for the M2
#define Dig_Out1 GPIO1
#define Dig_Out2 GPIO2
#define Dig_Out3 GPIO3
#define Dig_Out4 GPIO4
#define Dig_Out5 GPIO5
#define Dig_Out6 GPIO6

// Define the Analogue Inputs for the M2
// A0, A1, A2, A3, A7, A10, A11, A12, A15                     ADC CH#    position
#define Ana_In1 g_APinDescription[ANALOG_1].ulADCChannelNumber  //12        7
#define Ana_In2 g_APinDescription[ANALOG_2].ulADCChannelNumber  //11        6
#define Ana_In3 g_APinDescription[ANALOG_3].ulADCChannelNumber  //0         0
#define Ana_In4 g_APinDescription[ANALOG_4].ulADCChannelNumber  //2         2
#define Ana_In5 g_APinDescription[ANALOG_5].ulADCChannelNumber  //1         1
#define Ana_In6 g_APinDescription[ANALOG_6].ulADCChannelNumber  //7         4
#define Ana_In7 g_APinDescription[V_SENSE].ulADCChannelNumber   //3         3   // Vehicle Voltage
#define Ana_In8 g_APinDescription[I_SENSE].ulADCChannelNumber   //10        5   // M2 Supply AMPs
#define Ana_In9 g_APinDescription[CPU_TEMP].ulADCChannelNumber  //15        8   // CPU Temp


//Number of times a frame would have to be sent or received to actually toggle the LED
//This number thus slows down the blinking quite a bit - Useful to make it easier to see
//what is going on based on the LEDs.
//Applies just to RX and TX leds
#define BLINK_SLOWNESS     32  // default = 32

#define NUM_BUSES   5   //number of buses possible on this hardware - CAN0, CAN1, SWCAN, LIN1, LIN2 currently

struct FILTER {  //should be 10 bytes
  uint32_t id;
  uint32_t mask;
  boolean extended;
  boolean enabled;
};

enum FILEOUTPUTTYPE {
  NONE = 0,
  BINARYFILE = 1,
  GVRET = 2,
  CRTD = 3
};

void setLED(uint8_t which, boolean hi);

struct EEPROMSettings { //Must stay under 256
  uint8_t version;

  uint32_t CAN0Speed;
  uint32_t CAN1Speed;
  uint32_t SWCANSpeed;
  uint32_t LIN1Speed;
  uint32_t LIN2Speed;
  boolean CAN0_Enabled;
  boolean CAN1_Enabled;
  boolean SWCAN_Enabled;
  boolean LIN1_Enabled;
  boolean LIN2_Enabled;

  boolean useBinarySerialComm; //use a binary protocol on the serial link or human readable format?
  FILEOUTPUTTYPE fileOutputType; //what format should we use for file output?

  char fileNameBase[30]; //Base filename to use
  char fileNameExt[4]; //extension to use
  uint16_t fileNum; //incrementing value to append to filename if we create a new file each time
  boolean appendFile; //start a new file every power up or append to current?
  boolean autoStartLogging; //should logging start immediately on start up?

  uint8_t logLevel; //Level of logging to output on serial line
  uint8_t sysType; //Only M2 for now - Ignored until any hardware differences show up

  uint16_t valid; //stores a validity token to make sure EEPROM is not corrupt

  boolean SWCANListenOnly;
  boolean CAN0ListenOnly; //if true we don't allow any messing with the bus but rather just passively monitor.
  boolean CAN1ListenOnly;
};

class SettingsManager {
public:
  uint8_t LED_CANTX = RGB_GREEN;
  uint8_t LED_CANRX = RGB_BLUE;
  uint8_t LED_LOGGING = RGB_RED;

  // starts false, is set true on FS startup by Logger if autoStartLogging is on
  // can be enabled disabled manualy by serial commands
  boolean fileLoggingEnabled = false;

  boolean xbeeReset = false;
  boolean xbeePassthroughEnable = false;
  boolean xbeePassthroughWriteEnable = false;

  EEPROMSettings settings;

  void resetDefaults();

  void writeSettings() {
    EEPROM.write(EEPROM_ADDR, settings);
  }

  void readSettings() {
    EEPROM.read(EEPROM_ADDR, settings);
    if (settings.version != EEPROM_VER) { // if settings are not the current version then erase them and set defaults
      resetDefaults();
    } else {
    if (settings.CAN0ListenOnly > 1) settings.CAN0ListenOnly = 0;
    if (settings.CAN1ListenOnly > 1) settings.CAN1ListenOnly = 0;
    }
  }
};

extern SettingsManager sm;

#endif /* SETTINGS_H_ */
