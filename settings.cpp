
#include "settings.h"

void SettingsManager::resetDefaults() {
  settings.version = EEPROM_VER;
  settings.appendFile = false;
  settings.CAN0Speed = 500000;
  settings.CAN0_Enabled = true;
  settings.CAN1Speed = 500000;
  settings.CAN1_Enabled = false;
  settings.CAN0ListenOnly = false;
  settings.CAN1ListenOnly = false;
  settings.SWCAN_Enabled = false;
  settings.SWCANListenOnly = false; //TODO: Not currently respected or implemented.
  settings.SWCANSpeed = 33333;
  settings.LIN1_Enabled = false;
  settings.LIN2_Enabled = false;
  settings.LIN1Speed = 19200;
  settings.LIN2Speed = 19200;
  sprintf((char *)settings.fileNameBase, "CANBUS");
  sprintf((char *)settings.fileNameExt, "TXT");
  settings.fileNum = 1;
  settings.fileOutputType = CRTD;
  settings.useBinarySerialComm = false;
  settings.autoStartLogging = false;
  settings.logLevel = 1; //info
  settings.sysType = 0; //CANDUE as default
  settings.valid = 0; //not used right now
}
