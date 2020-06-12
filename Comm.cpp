#include "Comm.h"
#include "settings.h"
#include "sys_io.h"

void CommController::sendFrameToUSB(CAN_FRAME &frame, int whichBus) {
  uint8_t chksum;
  uint32_t now = micros();

  if (sm.settings.useBinarySerialComm) {
    if (frame.extended)
      frame.id |= 1 << 31;
    frameOutputBuffer[frameOutputBufferLength++] = 0xF1;
    frameOutputBuffer[frameOutputBufferLength++] =
        0; // 0 = canbus frame sending
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(now & 0xFF);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(now >> 8);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(now >> 16);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(now >> 24);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(frame.id & 0xFF);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(frame.id >> 8);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(frame.id >> 16);
    frameOutputBuffer[frameOutputBufferLength++] = (uint8_t)(frame.id >> 24);
    frameOutputBuffer[frameOutputBufferLength++] =
        frame.length + (uint8_t)(whichBus << 4);
    for (int c = 0; c < frame.length; c++) {
      frameOutputBuffer[frameOutputBufferLength++] = frame.data.bytes[c];
    }
    // chksum = checksumCalc(responseBuff, 11 + frame.length);
    chksum = 0;
    frameOutputBuffer[frameOutputBufferLength++] = chksum;
  } else {
    _out.print(micros());
    _out.print(" - ");
    _out.print(frame.id, HEX);
    if (frame.extended)
      _out.print(" X ");
    else
      _out.print(" S ");
    _out.print(whichBus);
    _out.print(" ");
    _out.print(frame.length);
    for (int c = 0; c < frame.length; c++) {
      _out.print(" ");
      _out.print(frame.data.bytes[c], HEX);
    }
    _out.println();
  }
}

//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t CommController::checksumCalc(uint8_t *buffer, int length)
{
  uint8_t valu = 0;
  for (int c = 0; c < length; c++) {
    valu ^= buffer[c];
  }
  return valu;
}

void CommController::checkBuffer() {
  if (micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL) {
    if (frameOutputBufferLength > 0) {
      _out.write(frameOutputBuffer, frameOutputBufferLength);
      frameOutputBufferLength = 0;
      lastFlushMicros = micros();
    }
  }
}

void CommController::getCommandLoop(STATE_t &currentState, int in_byte) {
  uint32_t now = micros();
  uint8_t temp8;
  uint16_t temp16;

  if (in_byte == PROTO_BUILD_CAN_FRAME) {
    responseBuff[0] = 0xF1;
    currentState = BUILD_CAN_FRAME;

  } else if (in_byte == PROTO_TIME_SYNC) { 
    responseBuff[0] = 0xF1;
    responseBuff[1] = 1; // time sync
    responseBuff[2] = (uint8_t)(now & 0xFF);
    responseBuff[3] = (uint8_t)(now >> 8);
    responseBuff[4] = (uint8_t)(now >> 16);
    responseBuff[5] = (uint8_t)(now >> 24);
    _out.write(responseBuff, 6);
    currentState = IDLE;

  } else if (in_byte == PROTO_DIG_INPUTS) { 
    // immediately return the data for digital inputs
    temp8 = getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) +
            (getDigital(3) << 3) + (getDigital(4) << 4) + (getDigital(5) << 5);
    responseBuff[0] = 0xF1;
    responseBuff[1] = 6; // digital inputs
    responseBuff[2] = temp8;
    temp8 = checksumCalc(responseBuff, 2);
    responseBuff[3] = temp8;
    _out.write(responseBuff, 4);
    currentState = IDLE;

  } else if (in_byte == PROTO_ANA_INPUTS) { 
    // immediately return data on analog inputs
    temp16 = getAnalog(0); // Analogue input 1
    responseBuff[0] = 0xF1;
    responseBuff[1] = 3;
    responseBuff[2] = temp16 & 0xFF;
    responseBuff[3] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(1); // Analogue input 2
    responseBuff[4] = temp16 & 0xFF;
    responseBuff[5] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(2); // Analogue input 3
    responseBuff[6] = temp16 & 0xFF;
    responseBuff[7] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(3); // Analogue input 4
    responseBuff[8] = temp16 & 0xFF;
    responseBuff[9] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(4); // Analogue input 5
    responseBuff[10] = temp16 & 0xFF;
    responseBuff[11] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(5); // Analogue input 6
    responseBuff[12] = temp16 & 0xFF;
    responseBuff[13] = uint8_t(temp16 >> 8);
    temp16 = getAnalog(6); // Vehicle Volts
    responseBuff[14] = temp16 & 0xFF;
    responseBuff[15] = uint8_t(temp16 >> 8);
    temp8 = checksumCalc(responseBuff, 9);
    responseBuff[16] = temp8;
    _out.write(responseBuff, 17);
    currentState = IDLE;    

  } else if (in_byte == PROTO_SET_DIG_OUT) { 
    responseBuff[0] = 0xF1;
    currentState = SET_DIG_OUTPUTS;

  } else if (in_byte == PROTO_SETUP_CANBUS) { 
    responseBuff[0] = 0xF1;
    currentState = SETUP_CANBUS;

  } else if (in_byte == PROTO_GET_CANBUS_PARAMS) { 
    // immediately return data on canbus params
    responseBuff[0] = 0xF1;
    responseBuff[1] = 6;
    responseBuff[2] = sm.settings.CAN0_Enabled +
                      ((unsigned char)sm.settings.CAN0ListenOnly << 4);
    responseBuff[3] = sm.settings.CAN0Speed;
    responseBuff[4] = sm.settings.CAN0Speed >> 8;
    responseBuff[5] = sm.settings.CAN0Speed >> 16;
    responseBuff[6] = sm.settings.CAN0Speed >> 24;
    responseBuff[7] =
        sm.settings.CAN1_Enabled +
        ((unsigned char)sm.settings.CAN1ListenOnly
         << 4); //+ (unsigned char)sm.settings.singleWireMode << 6;
    responseBuff[8] = sm.settings.CAN1Speed;
    responseBuff[9] = sm.settings.CAN1Speed >> 8;
    responseBuff[10] = sm.settings.CAN1Speed >> 16;
    responseBuff[11] = sm.settings.CAN1Speed >> 24;
    _out.write(responseBuff, 12);
    currentState = IDLE;

  } else if (in_byte == PROTO_GET_DEV_INFO) { 
    // immediately return device information
    responseBuff[0] = 0xF1;
    responseBuff[1] = 7;
    responseBuff[2] = CFG_BUILD_NUM & 0xFF;
    responseBuff[3] = (CFG_BUILD_NUM >> 8);
    responseBuff[4] = EEPROM_VER;
    responseBuff[5] = (unsigned char)sm.settings.fileOutputType;
    responseBuff[6] = (unsigned char)sm.settings.autoStartLogging;
    responseBuff[7] = 0; // was single wire mode. Should be rethought for this board.
    _out.write(responseBuff, 8);
    currentState = IDLE;

  } else if (in_byte == PROTO_SET_SW_MODE) { 
    responseBuff[0] = 0xF1;
    currentState = SET_SINGLEWIRE_MODE;

  } else if (in_byte == PROTO_KEEPALIVE) { 
    responseBuff[0] = 0xF1;
    responseBuff[1] = 0x09;
    responseBuff[2] = 0xDE;
    responseBuff[3] = 0xAD;
    _out.write(responseBuff, 4);
    currentState = IDLE;

  } else if (in_byte == PROTO_SET_SYSTYPE) { 
    responseBuff[0] = 0xF1;
    currentState = SET_SYSTYPE;

  } else if (in_byte == PROTO_ECHO_CAN_FRAME) { 
    responseBuff[0] = 0xF1;
    currentState = ECHO_CAN_FRAME;

  } else if (in_byte == PROTO_GET_NUMBUSES) { 
    responseBuff[0] = 0xF1;
    responseBuff[1] = 12;
    responseBuff[2] = 3; // number of buses actually supported by this hardware
                         // (TODO: will be 5 eventually)
    _out.write(responseBuff, 3);
    currentState = IDLE;

  } else if (in_byte == PROTO_GET_EXT_BUSES) { 
    responseBuff[0] = 0xF1;
    responseBuff[1] = 13;
    responseBuff[2] = sm.settings.SWCAN_Enabled +
                      ((unsigned char)sm.settings.SWCANListenOnly << 4);
    responseBuff[3] = sm.settings.SWCANSpeed;
    responseBuff[4] = sm.settings.SWCANSpeed >> 8;
    responseBuff[5] = sm.settings.SWCANSpeed >> 16;
    responseBuff[6] = sm.settings.SWCANSpeed >> 24;
    responseBuff[7] = sm.settings.LIN1_Enabled;
    responseBuff[8] = sm.settings.LIN1Speed;
    responseBuff[9] = sm.settings.LIN1Speed >> 8;
    responseBuff[10] = sm.settings.LIN1Speed >> 16;
    responseBuff[11] = sm.settings.LIN1Speed >> 24;
    responseBuff[12] = sm.settings.LIN2_Enabled;
    responseBuff[13] = sm.settings.LIN2Speed;
    responseBuff[14] = sm.settings.LIN2Speed >> 8;
    responseBuff[15] = sm.settings.LIN2Speed >> 16;
    responseBuff[16] = sm.settings.LIN2Speed >> 24;
    _out.write(responseBuff, 17);
    currentState = IDLE;

  } else if (in_byte == PROTO_SET_EXT_BUSES) { 
    responseBuff[0] = 0xF1;
    currentState = SETUP_EXT_BUSES;

  } else {
    currentState = IDLE;
  }
}

void CommController::echoCanFrame(int in_byte) {
  Logger::console("Processing echo byte %X", in_byte);
  static CAN_FRAME build_out_frame;
  static int out_bus;
  static int step = 0;
  int temp8;

  Logger::console("Step: %i", step);

  responseBuff[1 + step] = in_byte;
  switch (step) {
  case 0: {
    build_out_frame.id = in_byte;
    break;
  }
  case 1: {
    build_out_frame.id |= in_byte << 8;
    break;
  }
  case 2: {
    build_out_frame.id |= in_byte << 16;
    break;
  }
  case 3: {
    build_out_frame.id |= in_byte << 24;
    if (build_out_frame.id & 1 << 31) {
      build_out_frame.id &= 0x7FFFFFFF;
      build_out_frame.extended = true;
    } else
      build_out_frame.extended = false;
    break;
  }
  case 4: {
    out_bus = in_byte & 1;
    break;
  }
  case 5: {
    build_out_frame.length = in_byte & 0xF;
    if (build_out_frame.length > 8)
      build_out_frame.length = 8;
    break;
  }
  default: {
    if (step < build_out_frame.length + 6) {
      build_out_frame.data.bytes[step - 6] = in_byte;
    } else {
      // this would be the checksum byte. Compute and compare.
      temp8 = checksumCalc(responseBuff, step);
      // if (temp8 == in_byte)
      //{
      // toggleRXLED();
      Logger::console("Sending Frame");
      sendFrameToUSB(build_out_frame, 0);
      //}

      currentState = IDLE;
      step = 0;
      return;
    }
    break;
  }
  }
  step++;
}

void CommController::buildCanFrame(int in_byte) {
  static CAN_FRAME build_out_frame;
  static int out_bus;
  static int step = 0;
  uint8_t temp8;

  responseBuff[1 + step] = in_byte;
  switch (step) {
  case 0: {
    build_out_frame.id = in_byte;
    break;
  }
  case 1: {
    build_out_frame.id |= in_byte << 8;
    break;
  }
  case 2: {
    build_out_frame.id |= in_byte << 16;
    break;
  }
  case 3: {
    build_out_frame.id |= in_byte << 24;
    if (build_out_frame.id & 1 << 31) {
      build_out_frame.id &= 0x7FFFFFFF;
      build_out_frame.extended = true;
    } else
      build_out_frame.extended = false;
    break;
  }
  case 4: {
    out_bus = in_byte & 3;
    break;
  }
  case 5: {
    build_out_frame.length = in_byte & 0xF;
    if (build_out_frame.length > 8)
      build_out_frame.length = 8;
    break;
  }
  default: {
    if (step < build_out_frame.length + 6) {
      build_out_frame.data.bytes[step - 6] = in_byte;
    } else {
      // this would be the checksum byte. Compute and compare.
      temp8 = checksumCalc(responseBuff, step);
      build_out_frame.rtr = 0;

      _driver->sendFrame(BUSDriver::BUS(out_bus), build_out_frame);

      currentState = IDLE;
      step = 0;
      return;
    }
    break;
  }
  }
  step++;
}

void CommController::setupCanBus(int in_byte) {
  static uint32_t build_int;
  static int step = 0;
  // todo: validate checksum

  switch (step) {
  case 0: {
    build_int = in_byte;
    break;
  }
  case 1: {
    build_int |= in_byte << 8;
    break;
  }
  case 2: {
    build_int |= in_byte << 16;
    break;
  }
  case 3: {
    build_int |= in_byte << 24;
    if (build_int > 0) {
      if (build_int & 0x80000000) { // signals that enabled and listen only
                                    // status are also being passed
        if (build_int & 0x40000000) {
          sm.settings.CAN0_Enabled = true;
        } else {
          sm.settings.CAN0_Enabled = false;
        }
        if (build_int & 0x20000000) {
          sm.settings.CAN0ListenOnly = true;
        } else {
          sm.settings.CAN0ListenOnly = false;
        }
      } else {
        sm.settings.CAN0_Enabled = true;
      }
      build_int = build_int & 0xFFFFF;
      if (build_int > 1000000)
        build_int = 1000000;
      sm.settings.CAN0Speed = build_int;
    } else { // disable first canbus
      sm.settings.CAN0_Enabled = false;
    }

    break;
  }
  case 4: {
    build_int = in_byte;
    break;
  }
  case 5: {
    build_int |= in_byte << 8;
    break;
  }
  case 6: {
    build_int |= in_byte << 16;
    break;
  }
  case 7: {
    build_int |= in_byte << 24;
    if (build_int > 0) {
      if (build_int & 0x80000000) { // signals that enabled and listen only
                                    // status are also being passed
        if (build_int & 0x40000000) {
          sm.settings.CAN1_Enabled = true;
        } else {
          sm.settings.CAN1_Enabled = false;
        }
        if (build_int & 0x20000000) {
          sm.settings.CAN1ListenOnly = true;
        } else {
          sm.settings.CAN1ListenOnly = false;
        }
      } else {
        sm.settings.CAN1_Enabled = true;
      }
      build_int = build_int & 0xFFFFF;
      if (build_int > 1000000)
        build_int = 1000000;
      sm.settings.CAN1Speed = build_int;
    } else { // disable second canbus
      sm.settings.CAN1_Enabled = false;
    }
    // now, write out the new canbus settings to EEPROM
    sm.writeSettings();
    // force BUSDriver to reload settings from sm
    _driver->setup();

    currentState = IDLE;
    step = 0;
    return;
  }
  }
  step++;
}

// setup enable/listenonly/speed for SWCAN,
// Enable/Speed for LIN1, LIN2
void CommController::setupExtBuses(int in_byte) {
  static uint32_t build_int;
  static int step = 0;

  switch (step) {
  case 0: {
    build_int = in_byte;
    break;
  }
  case 1: {
    build_int |= in_byte << 8;
    break;
  }
  case 2: {
    build_int |= in_byte << 16;
    break;
  }
  case 3: {
    build_int |= in_byte << 24;
    if (build_int > 0) {
      if (build_int & 0x80000000) { // signals that enabled and listen only
                                    // status are also being passed
        if (build_int & 0x40000000) {
          sm.settings.SWCAN_Enabled = true;
        } else {
          sm.settings.SWCAN_Enabled = false;
        }
        if (build_int & 0x20000000) {
          sm.settings.SWCANListenOnly = true;
        } else {
          sm.settings.SWCANListenOnly = false;
        }
      } else {
        sm.settings.SWCAN_Enabled = true;
      }
      build_int = build_int & 0xFFFFF;
      if (build_int > 100000)
        build_int = 100000;
      sm.settings.SWCANSpeed = build_int;
    } else {
      sm.settings.SWCAN_Enabled = false;
    }
    break;
  }
  case 4: {
    build_int = in_byte;
    break;
  }
  case 5: {
    build_int |= in_byte << 8;
    break;
  }
  case 6: {
    build_int |= in_byte << 16;
    break;
  }
  case 7: {
    build_int |= in_byte << 24;
    /* FIX THIS UP TO INITIALIZE LIN1
      if (build_int > 0) {
        if (build_int & 0x80000000) { //signals that enabled and listen only
      status are also being passed if (build_int & 0x40000000) {
                sm.settings.CAN1_Enabled = true;
                Can1.enable();
            } else {
                sm.settings.CAN1_Enabled = false;
                Can1.disable();
            }
        } else {
            Can1.enable(); //if not using extended status mode then just
      default to enabling - this was old behavior sm.settings.CAN1_Enabled =
      true;
        }
        build_int = build_int & 0xFFFFF;
        if (build_int > 1000000) build_int = 1000000;
        Can1.begin(build_int, 255);
        //Can1.set_baudrate(build_int);
        sm.settings.CAN1Speed = build_int;
      } else { //disable second canbus
        Can1.disable();
        sm.settings.CAN1_Enabled = false;
      }*/
    break;
  }
  case 8: {
    build_int = in_byte;
    break;
  }
  case 9: {
    build_int |= in_byte << 8;
    break;
  }
  case 10: {
    build_int |= in_byte << 16;
    break;
  }
  case 11: {
    build_int |= in_byte << 24;
    /* FIX THIS UP TO INITIALIZE LIN2
      if (build_int > 0) {
        if (build_int & 0x80000000) { //signals that enabled and listen only
      status are also being passed if (build_int & 0x40000000) {
                sm.settings.CAN1_Enabled = true;
                Can1.enable();
            } else {
                sm.settings.CAN1_Enabled = false;
                Can1.disable();
            }
        } else {
            Can1.enable(); //if not using extended status mode then just
      default to enabling - this was old behavior sm.settings.CAN1_Enabled =
      true;
        }
        build_int = build_int & 0xFFFFF;
        if (build_int > 1000000) build_int = 1000000;
        Can1.begin(build_int, 255);
        //Can1.set_baudrate(build_int);
        sm.settings.CAN1Speed = build_int;
      } else { //disable second canbus
        Can1.disable();
        sm.settings.CAN1_Enabled = false;
      } */
    // now, write out the new canbus settings to EEPROM
    sm.writeSettings();
    // force BUSDriver to reload settings from sm
    _driver->setup();

    currentState = IDLE;
    step = 0;
    return;
  }
  }
  step++;
}

void CommController::read(Print &out, int in_byte) {
  static uint32_t build_int;

  // _out = out; // switch to whatever output we are using currently

  if (currentState == IDLE) {
    if (in_byte == 0xF1) {
      currentState = GET_COMMAND;
    } else if (in_byte == 0xE7) {
      sm.settings.useBinarySerialComm = true;
      sm.writeSettings();

    } else {
      _console.rcvCharacter(_out, (uint8_t)in_byte);
    }
  } else if (currentState == GET_COMMAND) { 
    getCommandLoop(currentState, in_byte);
  } else if (currentState == BUILD_CAN_FRAME) {
    buildCanFrame(in_byte);
  } else if (currentState == SETUP_CANBUS) {
    setupCanBus(in_byte);
  } else if (currentState == ECHO_CAN_FRAME) {
    echoCanFrame(in_byte);
  } else if (currentState == SETUP_EXT_BUSES) {
    setupExtBuses(in_byte);
  } else {
    currentState = IDLE;
  }
}