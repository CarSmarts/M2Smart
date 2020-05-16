#ifndef BUSDRIVER_UNIT_H_
#define BUSDRIVER_UNIT_H_

#include "BUSDriver.h"

class BUSDriver_unit : public DriverBase
{
private:
  CAN_FRAME frameBuf[20];
  DriverBase::BUS busIdBuf[20];
  uint8_t frameCount = 0;

public:
  BUSDriver_unit() { }

  void sendFrame(DriverBase::BUS whichBus, CAN_FRAME &frame) override {
    if (frameCount >= 20)
      return;

    frameBuf[frameCount] = frame;
    busIdBuf[frameCount] = whichBus;
    frameCount += 1;
  }

  const CAN_FRAME &getFrame(uint8_t at) {
    return frameBuf[at];
  }

  DriverBase::BUS getBus(uint8_t at) {
    return busIdBuf[at];
  }
};

#endif /* BUSDRIVER_UNIT_H_ */
