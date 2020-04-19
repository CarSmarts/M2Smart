#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>

class SerialCommon : public Stream {
private:

public:
  enum MODE { USB, XBEE };
  MODE mode = USB;

  SerialCommon(MODE mode = USB) : mode(mode){};

  // Serial wrapper functions
  void begin(unsigned long);
  void end();
  int available(void);
  int peek(void);
  int read(void);
  void flush(void);
  size_t write(uint8_t);
  using Print::write; // pull in write(str) and write(buf, size) from Print
  operator bool();
};

#endif /* SERIAL_H_ */
