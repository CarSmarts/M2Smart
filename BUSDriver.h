#ifndef BUSDRIVER_H_
#define BUSDRIVER_H_

#include "settings.h"

#include <due_can.h>
#include <MCP2515_sw_can.h>

class DriverBase {
public:
  enum BUS { BUS0 = 0, BUS1 = 1, SWBUS = 2 };
  enum SWTRANSCEIVERMODE {
    SLEEP = 0,
    HIGHSPEED = 1,
    HVWAKEUP = 2,
    NORMAL = 3
  };

  typedef void (*frameCallback_t)(CAN_FRAME &, DriverBase::BUS);

protected:
  static void noopFrameCallback(CAN_FRAME &, DriverBase::BUS) { return; }

  frameCallback_t frameCallback = noopFrameCallback;

  uint8_t txCounter = 0;
  uint8_t rxCounter = 0;
  boolean txToggle = true; // LED toggle values
  boolean rxToggle = true;

  void toggleRXLED();
  void toggleTXLED();

public:
  void setFrameCallback(frameCallback_t newFrameCallback);

  virtual void writeFrameToFile(CAN_FRAME &frame, int whichBus);

  virtual void setup() { }
  virtual void loop() { }

  virtual void setPromiscuousMode() { }
  virtual void sendFrame(DriverBase::BUS whichBus, CAN_FRAME &frame) = 0;
};

/// This class manages all of the bus related code
class BUSDriver : public DriverBase
{
private:
  SWcan SWCAN;
  // "MCP2515_sw_can" sets the tranciver to NORMAL on startup 
  SWTRANSCEIVERMODE swtranciverMode = NORMAL;

  // if message is sent with this id, will autmatically enable HVWAKEUP  
  static const uint32_t HVWAKEUPID = 0x100;

public:
  BUSDriver() : SWCAN(SPI0_CS3, SWC_INT) { }

  void SWCANIntHandler() { SWCAN.intHandler(); };

  void setSWTranciverMode(SWTRANSCEIVERMODE newMode) {
    swtranciverMode = newMode;
    SWCAN.mode(newMode);
  }

  void setPromiscuousMode() override;
  void setup() override;
  void loop() override;
  void sendFrame(DriverBase::BUS whichBus, CAN_FRAME &frame) override;
};

#endif /* BUSDRIVER_H_ */
