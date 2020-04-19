#ifndef BUSDRIVER_H_
#define BUSDRIVER_H_

#include "settings.h"

#include <due_can.h>
#include <MCP2515_sw_can.h>

/// This class manages all of the bus related code
class BUSDriver
{
private:
public:
  enum BUS { BUS0 = 0, BUS1 = 1, SWBUS = 2 };
  enum SWTRANSCEIVERMODE {
    SLEEP = 0,
    HIGHSPEED = 1,
    HVWAKEUP = 2,
    NORMAL = 3
  };

private:
  SWcan SWCAN;
  // "MCP2515_sw_can" sets the tranciver to NORMAL on startup 
  SWTRANSCEIVERMODE swtranciverMode = NORMAL;

  // if message is sent with this id, will autmatically enable HVWAKEUP  
  static const uint32_t HVWAKEUPID = 0x100;

  boolean txToggle = true; // LED toggle values
  boolean rxToggle = true;

  static void noopFrameCallback(CAN_FRAME &, BUSDriver::BUS) { return; }

  void (*frameCallback)(CAN_FRAME&, BUSDriver::BUS) = noopFrameCallback;

  void toggleRXLED();
  void toggleTXLED();

public:
  BUSDriver() : SWCAN(SPI0_CS3, SWC_INT) { }

  void SWCANIntHandler() { SWCAN.intHandler(); };

  void setSWTranciverMode(SWTRANSCEIVERMODE newMode) {
    swtranciverMode = newMode;
    SWCAN.mode(newMode);
  }

  void setPromiscuousMode();
  void setup();

  void writeFrameToFile(CAN_FRAME &frame, int whichBus);

  void sendFrame(BUSDriver::BUS whichBus, CAN_FRAME &frame);
  void setFrameCallback(void (*frameCallback)(CAN_FRAME &, BUSDriver::BUS));

  void loop();
};

#endif /* BUSDRIVER_H_ */
