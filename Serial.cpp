#include "Serial.h"

void SerialCommon::begin(unsigned long baud) {
  if (mode == USB) {
    SerialUSB.begin(baud);
  } else {
    Serial.begin(baud);
  }
}

void SerialCommon::end() {
  if (mode == USB) {
    SerialUSB.end();
  } else {
    Serial.end();
  }
}

int SerialCommon::available(void) {
  if (mode == USB) {
    SerialUSB.available();
  } else {
    Serial.available();
  }
}

int SerialCommon::peek(void) {
  if (mode == USB) {
    SerialUSB.peek();
  } else {
    Serial.peek();
  }
}

int SerialCommon::read(void) {
  if (mode == USB) {
    SerialUSB.read();
  } else {
    Serial.read();
  }
}

void SerialCommon::flush(void) {
  if (mode == USB) {
    SerialUSB.flush();
  } else {
    Serial.flush();
  }
}

size_t SerialCommon::write(uint8_t value) {
  if (mode == USB) {
    return SerialUSB.write(value);
  } else {
    return Serial.write(value);
  }
}

SerialCommon::operator bool() {
  if (mode == USB) {
    return bool(SerialUSB);
  } else {
    return bool(Serial);
  }
}
