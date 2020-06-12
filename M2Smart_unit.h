#ifdef TEST
#line 2 "M2Smart.ino"
#include <ArduinoUnit.h>
#include "Comm.h"
#include "BUSDriver_unit.h"


BUSDriver_unit mockDriver;
MockStream ms;
CommController mockComm(ms, mockDriver);

void readStream(Stream &stream) {
  while (stream.available())
  {
    mockComm.read(stream.read());
  }
}

test(dead)
{
  ms.input.print(0xF109);
  readStream(ms);
  
  assertEqual(ms.output,String(0xF109DEAD));
}

void setup() {
  setup_pins();
  xbeeReset();

  Serial.begin(115200);

  sm.readSettings();

  Test::out = &SerialUSB;
}
void loop() {
  Test::run();
}

#endif
