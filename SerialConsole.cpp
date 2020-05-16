/*
 * SerialConsole.cpp
 *
 Copyright (c) 2014 Collin Kidder

 Shamelessly copied from the version in GEVCU

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include "SerialConsole.h"
#include <due_wire.h>
#include <due_can.h>
#include <MCP2515_sw_can.h>
#include <lin_stack.h>
#include "EEPROM.h"
#include "settings.h"
#include "sys_io.h"

SerialConsole::SerialConsole(DriverBase &driver) : _driver(driver)
{
    //State variables for serial console
    ptrBuffer = 0;
}

void SerialConsole::printMenu(Print &out)
{
    char buff[80];
    //Show build # here as well in case people are using the native port and don't get to see the start up messages
    out.print("Build number: ");
    out.println(CFG_BUILD_NUM);
    out.println("System Menu:");
    out.println();
    out.println("Enable line endings of some sort (LF, CR, CRLF)");
    out.println();
    out.println("Short Commands:");
    out.println("h = help (displays this message)");
    out.println("R = reset to factory defaults");
    out.println("s = Start logging to file");
    out.println("S = Stop logging to file");
    out.println();
    out.println("Config Commands (enter command=newvalue). Current values shown in parenthesis:");
    out.println();

    Logger::console(out, "LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", sm.settings.logLevel);
    // Logger::console(out, "SYSTYPE=%i - set board type (0=Macchina M2)", sm.settings.sysType);
    out.println();

    Logger::console(out, "CAN0EN=%i - Enable/Disable CAN0 (0 = Disable, 1 = Enable)", sm.settings.CAN0_Enabled);
    Logger::console(out, "CAN0SPEED=%i - Set speed of CAN0 in baud (125000, 250000, etc)", sm.settings.CAN0Speed);
    Logger::console(out, "CAN0LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", sm.settings.CAN0ListenOnly);
    /*for (int i = 0; i < 8; i++) {
        sprintf(buff, "CAN0FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
        Logger::console(out, buff, sm.settings.CAN0Filters[i].id, sm.settings.CAN0Filters[i].mask,
                        sm.settings.CAN0Filters[i].extended, sm.settings.CAN0Filters[i].enabled);
    }*/
    out.println();

    Logger::console(out, "CAN1EN=%i - Enable/Disable CAN1 (0 = Disable, 1 = Enable)", sm.settings.CAN1_Enabled);
    Logger::console(out, "CAN1SPEED=%i - Set speed of CAN1 in baud (125000, 250000, etc)", sm.settings.CAN1Speed);
    Logger::console(out, "CAN1LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", sm.settings.CAN1ListenOnly);
    /*
    for (int i = 0; i < 8; i++) {
        sprintf(buff, "CAN1FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
        Logger::console(out, buff, sm.settings.CAN1Filters[i].id, sm.settings.CAN1Filters[i].mask,
                        sm.settings.CAN1Filters[i].extended, sm.settings.CAN1Filters[i].enabled);
    }*/
    
    out.println();

    Logger::console(out, "SWCANEN=%i - Enable/Disable Single Wire CAN (0 = Disable, 1 = Enable)", sm.settings.SWCAN_Enabled);
    Logger::console(out, "SWCANSPEED=%i - Set speed of Single Wire CAN in baud (33000, 93000, etc)", sm.settings.SWCANSpeed);
    Logger::console(out, "SWCANLISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", sm.settings.SWCANListenOnly);
    out.println();
    
    Logger::console(out, "CAN0SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: CAN0SEND=0x200,4,1,2,3,4");
    Logger::console(out, "CAN1SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: CAN1SEND=0x200,8,00,00,00,10,0xAA,0xBB,0xA0,00");
    Logger::console(out, "SWSEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: SWSEND=0x100,4,10,20,30,40");
    Logger::console(out, "MARK=<Description of what you are doing> - Set a mark in the log file about what you are about to do.");
    out.println();

    Logger::console(out, "BINSERIAL=%i - Enable/Disable Binary Sending of CANBus Frames to Serial (0=Dis, 1=En)", sm.settings.useBinarySerialComm);
    Logger::console(out, "FILETYPE=%i - Set type of file output (0=None, 1 = Binary, 2 = GVRET, 3 = CRTD)", sm.settings.fileOutputType);
    out.println();

    Logger::console(out, "FILEBASE=%s - Set filename base for saving", (char *)sm.settings.fileNameBase);
    Logger::console(out, "FILEEXT=%s - Set filename ext for saving", (char *)sm.settings.fileNameExt);
    Logger::console(out, "FILENUM=%i - Set incrementing number for filename", sm.settings.fileNum);
    Logger::console(out, "FILEAPPEND=%i - Append to file (no numbers) or use incrementing numbers after basename (0=Incrementing Numbers, 1=Append)", sm.settings.appendFile);
    Logger::console(out, "FILEAUTO=%i - Automatically start logging at startup (0=No, 1 = Yes)", sm.settings.autoStartLogging);
    out.println();

    // Logger::console(out, "DIGTOGEN=%i - Enable digital toggling system (0 = Dis, 1 = En)", digToggleSettings.enabled);
    // Logger::console(out, "DIGTOGMODE=%i - Set digital toggle mode (0 = Read pin, send CAN, 1 = Receive CAN, set pin)", digToggleSettings.mode & 1);
    // Logger::console(out, "DIGTOGLEVEL=%i - Set default level of digital pin (0 = LOW, 1 = HIGH)", digToggleSettings.mode >> 7);
    // Logger::console(out, "DIGTOGPIN=%i - Pin to use for digital toggling system (Use Arduino Digital Pin Number)", digToggleSettings.pin);
    // Logger::console(out, "DIGTOGID=%X - CAN ID to use for Rx or Tx", digToggleSettings.rxTxID);
    // Logger::console(out, "DIGTOGCAN0=%i - Use CAN0 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 1) & 1);
    // Logger::console(out, "DIGTOGCAN1=%i - Use CAN1 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 2) & 1);
    // Logger::console(out, "DIGTOGLEN=%i - Length of frame to send (Tx) or validate (Rx)", digToggleSettings.length);
    // Logger::console(out, "DIGTOGPAYLOAD=%X,%X,%X,%X,%X,%X,%X,%X - Payload to send or validate against (comma separated list)", digToggleSettings.payload[0],
    //                 digToggleSettings.payload[1], digToggleSettings.payload[2], digToggleSettings.payload[3], digToggleSettings.payload[4],
    //                 digToggleSettings.payload[5], digToggleSettings.payload[6], digToggleSettings.payload[7]);
}

/*	There is a help menu (press H or h or ?)
 This is no longer going to be a simple single character console.
 Now the system can handle up to 80 input characters. Commands are submitted
 by sending line ending (LF, CR, or both)
 */
void SerialConsole::rcvCharacter(Print &out, uint8_t chr)
{
    if (chr == 10 || chr == 13) { //command done. Parse it.
        handleConsoleCmd(out);
        ptrBuffer = 0; //reset line counter once the line has been processed
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char) chr;
        if (ptrBuffer > 79)
            ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd(Print &out)
{
    if (ptrBuffer == 1) {
        //command is a single ascii character
        handleShortCmd(out);
    } else { //at least two bytes
        handleConfigCmd(out);
    }
    ptrBuffer = 0; //reset line counter once the line has been processed
}

void SerialConsole::handleShortCmd(Print &out)
{
    uint8_t val;

    switch (cmdBuffer[0]) {
    //non-lawicel commands
    case 'h':
    case '?':
    case 'H':
        printMenu(out);
        break;
    case 'R': //reset to factory defaults.
        sm.settings.version = 0xFF;
        sm.writeSettings();
        Logger::console(out, "Power cycle to reset to factory defaults");
        break;
    case 's': //start logging canbus to file
        Logger::console(out, "Starting logging to file.");
        sm.fileLoggingEnabled = true;
        break;
    case 'S': //stop logging canbus to file
        Logger::console(out, "Ceasing file logging.");
        sm.fileLoggingEnabled = false;
        break;
    }
}


/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void SerialConsole::handleConfigCmd(Print &out)
{
    int i;
    int newValue;
    char *newString;
    bool writeEEPROM = false;
    // bool writeDigEE = false;
    char *dataTok;

    //Logger::debug("Cmd size: %i", ptrBuffer);
    if (ptrBuffer < 6)
        return; //4 digit command, =, value is at least 6 characters
    cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
    String cmdString = String();
    unsigned char whichEntry = '0';
    i = 0;

    while (cmdBuffer[i] != '=' && i < ptrBuffer) {
        cmdString.concat(String(cmdBuffer[i++]));
    }
    i++; //skip the =
    if (i >= ptrBuffer) {
        Logger::console(out, "Command needs a value..ie TORQ=3000");
        Logger::console(out, "");
        return; //or, we could use this to display the parameter instead of setting
    }

    // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
    newValue = strtol((char *) (cmdBuffer + i), NULL, 0); //try to turn the string into a number
    newString = (char *)(cmdBuffer + i); //leave it as a string

    cmdString.toUpperCase();

    if (cmdString == String("CAN0EN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting CAN0 Enabled to %i", newValue);
        sm.settings.CAN0_Enabled = newValue;
        _driver.setup();
        writeEEPROM = true;
    } else if (cmdString == String("CAN1EN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting CAN1 Enabled to %i", newValue);
        sm.settings.CAN1_Enabled = newValue;
        _driver.setup();
        writeEEPROM = true;
    } else if (cmdString == String("SWCANEN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting SWCAN Enabled to %i", newValue);
        sm.settings.SWCAN_Enabled = newValue;
        _driver.setup();
        writeEEPROM = true;
    } else if (cmdString == String("CAN0SPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console(out, "Setting CAN0 Baud Rate to %i", newValue);
            sm.settings.CAN0Speed = newValue;
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("CAN1SPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console(out, "Setting CAN1 Baud Rate to %i", newValue);
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("SWCANSPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console(out, "Setting Single Wire CAN Baud Rate to %i", newValue);
            sm.settings.SWCANSpeed = newValue;
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("CAN0LISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console(out, "Setting CAN0 Listen Only to %i", newValue);
            sm.settings.CAN0ListenOnly = newValue;
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid setting! Enter a value 0 - 1");
    } else if (cmdString == String("CAN1LISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console(out, "Setting CAN1 Listen Only to %i", newValue);
            sm.settings.CAN1ListenOnly = newValue;
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid setting! Enter a value 0 - 1");
    } else if (cmdString == String("SWCANLISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console(out, "Setting SWCAN Listen Only to %i", newValue);
            sm.settings.SWCANListenOnly = newValue;
            _driver.setup();
            writeEEPROM = true;
        } else Logger::console(out, "Invalid setting! Enter a value 0 - 1");        
    
    } else if (cmdString.startsWith(String("CAN0FILTER")) ||
            cmdString.startsWith(String("CAN1FILTER"))) {
        unsigned int filterIdx = parseHexCharacter(cmdString.charAt(10));
        unsigned int busIdx = parseHexCharacter(cmdString.charAt(3));
        if (filterIdx < 7) {
            handleFilterSet(out, busIdx, filterIdx, newString);
        }
    }
    else if (cmdString == String("CAN0SEND"))
    {
        handleCANSend(out, DriverBase::BUS0, newString);
    }
    else if (cmdString == String("CAN1SEND"))
    {
        handleCANSend(out, DriverBase::BUS1, newString);
    }
    else if (cmdString == String("SWSEND"))
    {
        handleCANSend(out, DriverBase::SWBUS, newString);
    }
    else if (cmdString == String("MARK"))
    { //just ascii based for now
        if (sm.settings.fileOutputType == GVRET) Logger::file("Mark: %s", newString);
        if (sm.settings.fileOutputType == CRTD) {
            uint8_t buff[40];
            sprintf((char *)buff, "%f CEV ", millis() / 1000.0f);
            Logger::fileRaw(buff, strlen((char *)buff));
            Logger::fileRaw((uint8_t *)newString, strlen(newString));
            buff[0] = '\r';
            buff[1] = '\n';
            Logger::fileRaw(buff, 2);
        }
        if (!sm.settings.useBinarySerialComm) Logger::console(out, "Mark: %s", newString);
    }
    else if (cmdString == String("BINSERIAL"))
    {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting Serial Binary Comm to %i", newValue);
        sm.settings.useBinarySerialComm = newValue;
        writeEEPROM = true;
    }
    else if (cmdString == String("FILETYPE"))
    {
        if (newValue < 0) newValue = 0;
        if (newValue > 3) newValue = 3;
        Logger::console(out, "Setting File Output Type to %i", newValue);
        sm.settings.fileOutputType = (FILEOUTPUTTYPE)newValue; //the numbers all intentionally match up so this works
        writeEEPROM = true;
    }
    else if (cmdString == String("FILEBASE"))
    {
        Logger::console(out, "Setting File Base Name to %s", newString);
        strcpy((char *)sm.settings.fileNameBase, newString);
        writeEEPROM = true;
    }
    else if (cmdString == String("FILEEXT"))
    {
        Logger::console(out, "Setting File Extension to %s", newString);
        strcpy((char *)sm.settings.fileNameExt, newString);
        writeEEPROM = true;
    }
    else if (cmdString == String("FILENUM"))
    {
        Logger::console(out, "Setting File Incrementing Number Base to %i", newValue);
        sm.settings.fileNum = newValue;
        writeEEPROM = true;
    }
    else if (cmdString == String("FILEAPPEND"))
    {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting File Append Mode to %i", newValue);
        sm.settings.appendFile = newValue;
        writeEEPROM = true;
    }
    else if (cmdString == String("FILEAUTO"))
    {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console(out, "Setting Auto File Logging Mode to %i", newValue);
        sm.settings.autoStartLogging = newValue;
        writeEEPROM = true;
    // } else if (cmdString == String("SYSTYPE")) {
    //     if (newValue < 1 && newValue >= 0) {
    //         sm.settings.sysType = newValue;
    //         writeEEPROM = true;
    //         Logger::console(out, "System type updated. Power cycle to apply.");
    //     } else Logger::console(out, "Invalid system type. Please enter a value between 0 and 0. Yes, just 0");
    // } else if (cmdString == String("DIGTOGEN")) {
    //     if (newValue >= 0 && newValue <= 1) {
    //         Logger::console(out, "Setting Digital Toggle System Enable to %i", newValue);
    //         digToggleSettings.enabled = newValue;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid enable value. Must be either 0 or 1");
    // } else if (cmdString == String("DIGTOGMODE")) {
    //     if (newValue >= 0 && newValue <= 1) {
    //         Logger::console(out, "Setting Digital Toggle Mode to %i", newValue);
    //         if (newValue == 0) digToggleSettings.mode &= ~1;
    //         if (newValue == 1) digToggleSettings.mode |= 1;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid mode. Must be either 0 or 1");
    // } else if (cmdString == String("DIGTOGLEVEL")) {
    //     if (newValue >= 0 && newValue <= 1) {
    //         Logger::console(out, "Setting Digital Toggle Starting Level to %i", newValue);
    //         if (newValue == 0) digToggleSettings.mode &= ~0x80;
    //         if (newValue == 1) digToggleSettings.mode |= 0x80;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid default level. Must be either 0 or 1");
    // } else if (cmdString == String("DIGTOGPIN")) {
    //     if (newValue >= 0 && newValue <= 77) {
    //         Logger::console(out, "Setting Digital Toggle Pin to %i", newValue);
    //         digToggleSettings.pin = newValue;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid pin. Must be between 0 and 77");
    // } else if (cmdString == String("DIGTOGID")) {
    //     if (newValue >= 0 && newValue < (1 << 30)) {
    //         Logger::console(out, "Setting Digital Toggle CAN ID to %X", newValue);
    //         digToggleSettings.rxTxID = newValue;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid CAN ID. Must be either an 11 or 29 bit ID");
    // } else if (cmdString == String("DIGTOGCAN0")) {
    //     if (newValue >= 0 && newValue <= 1) {
    //         Logger::console(out, "Setting Digital Toggle CAN0 Usage to %i", newValue);
    //         if (newValue == 0) digToggleSettings.mode &= ~2;
    //         if (newValue == 1) digToggleSettings.mode |= 2;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid value. Must be either 0 or 1");
    // } else if (cmdString == String("DIGTOGCAN1")) {
    //     if (newValue >= 0 && newValue <= 1) {
    //         Logger::console(out, "Setting Digital Toggle CAN1 Usage to %i", newValue);
    //         if (newValue == 0) digToggleSettings.mode &= ~4;
    //         if (newValue == 1) digToggleSettings.mode |= 4;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid value. Must be either 0 or 1");
    // } else if (cmdString == String("DIGTOGLEN")) {
    //     if (newValue >= 0 && newValue <= 8) {
    //         Logger::console(out, "Setting Digital Toggle Frame Length to %i", newValue);
    //         digToggleSettings.length = newValue;
    //         writeDigEE = true;
    //     } else Logger::console(out, "Invalid length. Must be between 0 and 8");
    // } else if (cmdString == String("DIGTOGPAYLOAD")) {
    //     dataTok = strtok(newString, ",");
    //     if (dataTok) {
    //         digToggleSettings.payload[0] = strtol(dataTok, NULL, 0);
    //         i = 1;
    //         while (i < 8 && dataTok) {
    //             dataTok = strtok(NULL, ",");
    //             if (dataTok) {
    //                 digToggleSettings.payload[i] = strtol(dataTok, NULL, 0);
    //                 i += 1;
    //             }
    //         }
    //         writeDigEE = true;
    //         Logger::console(out, "Set new payload bytes");
    //     } else Logger::console(out, "Error processing payload");
    }
    else if (cmdString == String("LOGLEVEL"))
    {
        switch (newValue) {
        case 0:
            Logger::setLoglevel(Logger::Debug);
            sm.settings.logLevel = 0;
            Logger::console(out, "setting loglevel to 'debug'");
            writeEEPROM = true;
            break;
        case 1:
            Logger::setLoglevel(Logger::Info);
            sm.settings.logLevel = 1;
            Logger::console(out, "setting loglevel to 'info'");
            writeEEPROM = true;
            break;
        case 2:
            Logger::console(out, "setting loglevel to 'warning'");
            sm.settings.logLevel = 2;
            Logger::setLoglevel(Logger::Warn);
            writeEEPROM = true;
            break;
        case 3:
            Logger::console(out, "setting loglevel to 'error'");
            sm.settings.logLevel = 3;
            Logger::setLoglevel(Logger::Error);
            writeEEPROM = true;
            break;
        case 4:
            Logger::console(out, "setting loglevel to 'off'");
            sm.settings.logLevel = 4;
            Logger::setLoglevel(Logger::Off);
            writeEEPROM = true;
            break;
        }
    }
    else
    {
        Logger::console(out, "Unknown command");
    }
    if (writeEEPROM) {
        sm.writeSettings();
    }
    // if (writeDigEE) {
    //     EEPROM.write(EEPROM_ADDR + 1024, digTogglesm.settings);
    // }
}

//CAN0FILTER%i=%%i,%%i,%%i,%%i (ID, Mask, Extended, Enabled)", i);
bool SerialConsole::handleFilterSet(Print &out, uint8_t bus, uint8_t filter, char *values)
{
    if (filter < 0 || filter > 7) return false;
    if (bus < 0 || bus > 1) return false;

    //there should be four tokens
    char *idTok = strtok(values, ",");
    char *maskTok = strtok(NULL, ",");
    char *extTok = strtok(NULL, ",");
    char *enTok = strtok(NULL, ",");

    if (!idTok) return false; //if any of them were null then something was wrong. Abort.
    if (!maskTok) return false;
    if (!extTok) return false;
    if (!enTok) return false;

    int idVal = strtol(idTok, NULL, 0);
    int maskVal = strtol(maskTok, NULL, 0);
    int extVal = strtol(extTok, NULL, 0);
    int enVal = strtol(enTok, NULL, 0);

    Logger::console(out, "Setting CAN%iFILTER%i to ID 0x%x Mask 0x%x Extended %i Enabled %i", bus, filter, idVal, maskVal, extVal, enVal);

    if (bus == 0) {
        //sm.settings.CAN0Filters[filter].id = idVal;
        //sm.settings.CAN0Filters[filter].mask = maskVal;
        //sm.settings.CAN0Filters[filter].extended = extVal;
        //sm.settings.CAN0Filters[filter].enabled = enVal;
        //Can0.setRXFilter(filter, idVal, maskVal, extVal);
    } else if (bus == 1) {
        //sm.settings.CAN1Filters[filter].id = idVal;
        //sm.settings.CAN1Filters[filter].mask = maskVal;
        //sm.settings.CAN1Filters[filter].extended = extVal;
        //sm.settings.CAN1Filters[filter].enabled = enVal;
        //Can1.setRXFilter(filter, idVal, maskVal, extVal);
    }

    return true;
}

bool SerialConsole::handleCANSend(Print &out, DriverBase::BUS whichBus, char *inputString)
{
    char *idTok = strtok(inputString, ",");
    char *lenTok = strtok(NULL, ",");
    char *dataTok;
    CAN_FRAME frame;

    if (!idTok) return false;
    if (!lenTok) return false;

    int idVal = strtol(idTok, NULL, 0);
    int lenVal = strtol(lenTok, NULL, 0);

    for (int i = 0; i < lenVal; i++) {
        dataTok = strtok(NULL, ",");
        if (!dataTok) return false;
        frame.data.byte[i] = strtol(dataTok, NULL, 0);
    }

    //things seem good so try to send the frame.
    frame.id = idVal;
    if (idVal >= 0x7FF) frame.extended = true;
    else frame.extended = false;
    frame.rtr = 0;
    frame.length = lenVal;
    _driver.sendFrame(whichBus, frame);
    
    Logger::console(out, "Sending frame with id: 0x%x len: %i", frame.id, frame.length);
    return true;
}

unsigned int SerialConsole::parseHexCharacter(char chr)
{
    unsigned int result = 0;
    if (chr >= '0' && chr <= '9') result = chr - '0';
    else if (chr >= 'A' && chr <= 'F') result = 10 + chr - 'A';
    else if (chr >= 'a' && chr <= 'f') result = 10 + chr - 'a';

    return result;
}
