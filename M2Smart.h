#ifndef M2SMART_H_
#define M2SMART_H_

#include "sys_io.h"

void setup_pins() {
  sys_early_setup();

  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);

  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_BLUE, OUTPUT);
  pinMode(DS2, OUTPUT);
  pinMode(DS3, OUTPUT);
  pinMode(DS4, OUTPUT);
  pinMode(DS5, OUTPUT);
  pinMode(DS6, OUTPUT);

  pinMode(XBEE_RST, OUTPUT);
  pinMode(XBEE_MULT4, OUTPUT);

  digitalWrite(SWC_M0, LOW); // Mode 0 for SWCAN
  digitalWrite(SWC_M1, LOW); // mode 1

  // Set RGB LED to completely off.
  digitalWrite(RGB_GREEN, HIGH);
  digitalWrite(RGB_BLUE, HIGH);
  digitalWrite(RGB_RED, HIGH);
  digitalWrite(DS2, HIGH);
  digitalWrite(DS3, HIGH);
  digitalWrite(DS4, HIGH);
  digitalWrite(DS5, HIGH);
  digitalWrite(DS6, HIGH);
}

void xbeeReset() {
  digitalWrite(DS2, LOW);
  digitalWrite(XBEE_RST, LOW);
  delay(1000);
  digitalWrite(DS2, HIGH);
  digitalWrite(XBEE_RST, HIGH);
}

// Enable forward mode to update firmware on XBEE
void xbeePassthroughLoop() {
  char rx_byte = 0;
  SerialUSB.println("Entering infinite loop passthough mode");

  if (sm.xbeePassthroughWriteEnable) {
    digitalWrite(DS3, LOW);
    digitalWrite(XBEE_MULT4, LOW);
    digitalWrite(DS2, LOW);
    digitalWrite(XBEE_RST, LOW);
    delay(500);

    digitalWrite(DS2, HIGH);
    digitalWrite(XBEE_RST, HIGH);
    delay(200);

    digitalWrite(DS3, HIGH);
    digitalWrite(XBEE_MULT4, HIGH);
    digitalWrite(DS2, HIGH);
    digitalWrite(XBEE_RST, HIGH);
  }

  while (true) {
    if (SerialUSB.available() > 0) {
      rx_byte = SerialUSB.read();
      Serial.write(rx_byte);
    }
    if (Serial.available() > 0) {
      rx_byte = Serial.read();
      SerialUSB.write(rx_byte);
    }
  }
}

void update_buttons() {
  static uint32_t last_led_update = 0;
  static int is_on = 0;
  const uint32_t update_freq = 2000;

  int xbee_rst_button_state = digitalRead(Button1);
  int xbee_multi4_button_state = digitalRead(Button2);

  if (xbee_rst_button_state == LOW) {
    digitalWrite(DS2, LOW);
    digitalWrite(XBEE_RST, LOW);
  } else {
    digitalWrite(DS2, HIGH);
    digitalWrite(XBEE_RST, HIGH);
  }

  if (xbee_multi4_button_state == LOW) {
    digitalWrite(DS3, LOW);
    digitalWrite(XBEE_MULT4, LOW);
  } else {
    digitalWrite(DS3, HIGH);
    digitalWrite(XBEE_MULT4, HIGH);
  }

  if (sm.xbeeReset) {
    xbeeReset();
  }

  if (sm.xbeePassthroughEnable || digitalRead(Button2) == LOW) {
    xbeePassthroughLoop();
  }
}

#endif /* M2SMART_H_ */
