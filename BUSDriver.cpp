
#include "BUSDriver.h"
#include "Logger.h"

void DriverBase::toggleRXLED() {
  rxCounter++;
  if (rxCounter >= BLINK_SLOWNESS) {
    rxCounter = 0;
    rxToggle = !rxToggle;
    setLED(sm.LED_CANRX, rxToggle);
  }
}

void DriverBase::toggleTXLED() {
  txCounter++;
  if (txCounter >= BLINK_SLOWNESS) {
    txCounter = 0;
    txToggle = !txToggle;
    setLED(sm.LED_CANTX, txToggle);
  }
}

void DriverBase::setFrameCallback(frameCallback_t newFrameCallback) {
  if (newFrameCallback == NULL) {
    frameCallback = noopFrameCallback;
  } else {
    frameCallback = newFrameCallback;
  }
}

void DriverBase::writeFrameToFile(CAN_FRAME &frame, int whichBus) {
  uint8_t buff[40];
  // uint8_t temp;
  uint32_t timestamp;

  switch (sm.settings.fileOutputType) {
  case BINARYFILE:
    if (frame.extended) {
      frame.id |= 1 << 31;
    }
    timestamp = micros();
    buff[0] = (uint8_t)(timestamp & 0xFF);
    buff[1] = (uint8_t)(timestamp >> 8);
    buff[2] = (uint8_t)(timestamp >> 16);
    buff[3] = (uint8_t)(timestamp >> 24);
    buff[4] = (uint8_t)(frame.id & 0xFF);
    buff[5] = (uint8_t)(frame.id >> 8);
    buff[6] = (uint8_t)(frame.id >> 16);
    buff[7] = (uint8_t)(frame.id >> 24);
    buff[8] = frame.length + (uint8_t)(whichBus << 4);
    for (int c = 0; c < frame.length; c++) {
      buff[9 + c] = frame.data.bytes[c];
    }
    Logger::fileRaw(buff, 9 + frame.length);
    break;

  case GVRET:
    sprintf((char *)buff, "%i,%x,%i,%i,%i", millis(), frame.id, frame.extended,
            whichBus, frame.length);
    Logger::fileRaw(buff, strlen((char *)buff));

    for (int c = 0; c < frame.length; c++) {
      sprintf((char *)buff, ",%x", frame.data.bytes[c]);
      Logger::fileRaw(buff, strlen((char *)buff));
    }
    buff[0] = '\r';
    buff[1] = '\n';
    Logger::fileRaw(buff, 2);
    break;

  case CRTD:
    int idBits = 11;
    if (frame.extended) {
      idBits = 29;
    }
    sprintf((char *)buff, "%f R%i %x", millis() / 1000.0f, idBits, frame.id);
    Logger::fileRaw(buff, strlen((char *)buff));

    for (int c = 0; c < frame.length; c++) {
      sprintf((char *)buff, " %x", frame.data.bytes[c]);
      Logger::fileRaw(buff, strlen((char *)buff));
    }
    buff[0] = '\r';
    buff[1] = '\n';
    Logger::fileRaw(buff, 2);
    break;
  }
}

void BUSDriver::setPromiscuousMode() {
  // By default there are 7 mailboxes for each device that are RX boxes
  // This sets each mailbox to have an open filter that will accept extended
  // or standard frames
  int filter;
  // extended
  for (filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
    Can1.setRXFilter(filter, 0, 0, true);
  }
  // standard
  for (filter = 3; filter < 7; filter++) {
    Can0.setRXFilter(filter, 0, 0, false);
    Can1.setRXFilter(filter, 0, 0, false);
  }
}

void BUSDriver::setup() {
  if (sm.settings.CAN0_Enabled)
  {
    if (sm.settings.CAN0ListenOnly) {
      Can0.setListenOnlyMode(true);
    } else {
      Can0.setListenOnlyMode(false);
    }
    Can0.enable();
    Can0.begin(sm.settings.CAN0Speed, 255);
  }
  else
  {
    Can0.disable();
  }

  if (sm.settings.CAN1_Enabled) {
    if (sm.settings.CAN1ListenOnly) {
      Can1.setListenOnlyMode(true);
    } else {
      Can1.setListenOnlyMode(false);
    }
    Can1.enable();
    Can1.begin(sm.settings.CAN1Speed, 255);
  } else {
    Can1.disable();
  }

  if (sm.settings.SWCAN_Enabled) {
    SWCAN.setupSW(sm.settings.SWCANSpeed);
    setSWTranciverMode(SWTRANSCEIVERMODE::NORMAL);
    delay(20);

    if (sm.settings.SWCANListenOnly) {
      SWCAN.setListenOnlyMode(true);
    } else {
      SWCAN.setListenOnlyMode(false);
    }
  } else {
    SWCAN.Reset();
    setSWTranciverMode(SWTRANSCEIVERMODE::SLEEP);
  }
}

void BUSDriver::loop() {
  CAN_FRAME recived_frame;

  if (Can0.available() > 0) {
    Can0.read(recived_frame);
    // addBits(0, recived_frame); // TODO: Implement BUSload
    toggleRXLED();

    frameCallback(recived_frame, BUS::BUS0);
  }

  if (Can1.available() > 0) {
    Can1.read(recived_frame);
    // addBits(1, recived_frame);
    toggleRXLED();

    frameCallback(recived_frame, BUS::BUS1);
  }

  // if (SWCAN.GetRXFrame(recived_frame)) {
  //   toggleRXLED();
  //   frameCallback(recived_frame, BUS::SWBUS);
  // }

  toggleRXLED(); // show that we are still alive
}

void BUSDriver::sendFrame(BUSDriver::BUS whichBus, CAN_FRAME &frame) {
  switch (whichBus) {
  case BUS::BUS0:
    Can0.sendFrame(frame);
    break;
  
  case BUS::BUS1:
    Can1.sendFrame(frame);
    break;

  case BUS::SWBUS:
    // Special case to enable HVWAKUP for gmlan?
    if (frame.id == HVWAKEUPID) {
      SWCAN.mode(2);
      delay(1);
    }

    SWCAN.sendFrame(frame);

    // revert to last tranciver mode
    if (frame.id == HVWAKEUPID) {
      delay(1);
      SWCAN.mode(swtranciverMode);
    }
    break;
  }

  writeFrameToFile(frame, whichBus); // copy sent frames to file as well.
  // addBits(whichBus, frame);
  toggleTXLED();
}
