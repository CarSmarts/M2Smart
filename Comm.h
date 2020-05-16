#ifndef COMM_H_
#define COMM_H_

#include <Arduino.h>
#include "BUSDriver.h"
#include "SerialConsole.h"

class CommController
{
private:
  enum STATE_t: uint8_t {
    IDLE = 0,
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

  enum GVRET_PROTOCOL: uint8_t {
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

  Print &_out;
  DriverBase &_driver;
  SerialConsole _console;

  // statemachine variables for reciving instructions
  STATE_t currentState = IDLE;
  byte responseBuff[20];

  uint8_t checksumCalc(uint8_t *buffer, int length);

  void getCommandLoop(STATE_t &currentState, int in_byte);
  void echoCanFrame(int in_byte);
  void buildCanFrame(int in_byte);
  void setupCanBus(int in_byte);
  void setupExtBuses(int in_byte);

  // buffer for outputing frames
  byte frameOutputBuffer[SER_BUFF_SIZE];
  int frameOutputBufferLength = 0; //not creating a ring buffer. The buffer should be large enough to never overflow
  uint32_t lastFlushMicros = 0;

public:
  CommController(Print &out, DriverBase &driver) : _out(out), _driver(driver), _console(driver) { };

  /// call whenever there is a new input to process
  void read(Print &out, int in_byte);

  /// check to see if buffer is ready to be sent
  /// needs to be called peroidically
  void checkBuffer();

  void sendFrameToUSB(CAN_FRAME &frame, int whichBus);
};

#endif /* COMM_H_ */
