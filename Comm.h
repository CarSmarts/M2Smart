#ifndef COMM_H_
#define COMM_H_

#include <Arduino.h>
#include "Serial.h"
#include "BUSDriver.h"

class CommController
{
private:
  enum STATE {
    IDLE,
    GET_COMMAND,
    BUILD_CAN_FRAME,
    TIME_SYNC,
    GET_DIG_INPUTS,
    GET_ANALOG_INPUTS,
    SET_DIG_OUTPUTS,
    SETUP_CANBUS,
    GET_CANBUS_PARAMS,
    GET_DEVICE_INFO,
    SET_SINGLEWIRE_MODE,
    SET_SYSTYPE,
    ECHO_CAN_FRAME,
    SETUP_EXT_BUSES
  };

  enum GVRET_PROTOCOL {
    PROTO_BUILD_CAN_FRAME = 0,
    PROTO_TIME_SYNC = 1,
    PROTO_DIG_INPUTS = 2,
    PROTO_ANA_INPUTS = 3,
    PROTO_SET_DIG_OUT = 4,
    PROTO_SETUP_CANBUS = 5,
    PROTO_GET_CANBUS_PARAMS = 6,
    PROTO_GET_DEV_INFO = 7,
    PROTO_SET_SW_MODE = 8,
    PROTO_KEEPALIVE = 9,
    PROTO_SET_SYSTYPE = 10,
    PROTO_ECHO_CAN_FRAME = 11,
    PROTO_GET_NUMBUSES = 12,
    PROTO_GET_EXT_BUSES = 13,
    PROTO_SET_EXT_BUSES = 14
  };

  SerialCommon _serial;
  BUSDriver &_driver;

  // statemachine variables for reciving instructions
  STATE currentState = IDLE;
  byte responseBuff[20];

  uint8_t checksumCalc(uint8_t *buffer, int length);

  void getCommandLoop(int in_byte);
  void echoCanFrame(int in_byte);
  void buildCanFrame(int in_byte);
  void setupCanBus(int in_byte);
  void setupExtBuses(int in_byte);

  // buffer for outputing frames
  byte frameOutputBuffer[SER_BUFF_SIZE];
  int frameOutputBufferLength = 0; //not creating a ring buffer. The buffer should be large enough to never overflow
  uint32_t lastFlushMicros = 0;
  // check to see if buffer is ready to be sent
  void checkBuffer();

public:
  CommController(SerialCommon::MODE commMode, BUSDriver &driver) : _serial(commMode), _driver(driver) { };
  ~CommController(){};

  void setup();
  void loop();

  void sendFrameToUSB(CAN_FRAME &frame, int whichBus);
};

#endif /* COMM_H_ */
