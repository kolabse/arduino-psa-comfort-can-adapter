#include <Arduino.h>
#include "PsaComfortCanAdapter.h"


PsaComfortCanAdapter::PsaComfortCanAdapter(uint8_t csPinCan0, uint8_t csPinCan1) : _can0(getMCP(csPinCan0)), _can1(getMCP(csPinCan1)) {

}

MCP2515 PsaComfortCanAdapter::getMCP(uint8_t mcp_cs_pin) {
    MCP2515 can(mcp_cs_pin);
    return can;
}

void PsaComfortCanAdapter::resetEEPROM() {
    EEPROM.update(0, 0);
    EEPROM.update(1, 0);
    EEPROM.update(2, 0);
    EEPROM.update(3, 0);
    EEPROM.update(4, 0);
    EEPROM.update(5, 0);
    EEPROM.update(6, 0);
    EEPROM.update(7, 0);
    EEPROM.update(10, 0);
    EEPROM.update(11, 0);
    EEPROM.update(12, 0);
    EEPROM.update(13, 0);
    EEPROM.update(14, 0);
    EEPROM.update(15, 0);
    EEPROM.update(16, 0);
}

void PsaComfortCanAdapter::adapterInit() {
    int tmpVal;

  if (_resetEEPROM) {
    resetEEPROM();
  }

  if (_debugCAN0 || _debugCAN1 || _debugGeneral) {
    _serialEnabled = true;
  }

  // Read data from EEPROM
  tmpVal = EEPROM.read(0);
  if (tmpVal >= 128) {
    _languageAndUnitNum = tmpVal;
  }

  if ((_languageAndUnitNum % 2) == 0 && _kmL) {
    _languageAndUnitNum = _languageAndUnitNum + 1;
  }

  tmpVal = EEPROM.read(1);
  if (tmpVal <= 32) {
    _languageID_CAN2004 = tmpVal;
  }

  tmpVal = EEPROM.read(2);
  if (tmpVal <= 32) {
    _languageID = tmpVal;
  }

  tmpVal = EEPROM.read(3);
  if (tmpVal == 1) {
    _temperatureInF = true;
  }

  tmpVal = EEPROM.read(4);
  if (tmpVal == 1) {
    _mpgMi = true;
  }

  tmpVal = EEPROM.read(5);
  if (tmpVal <= 31) {
    _time_day = tmpVal;
  }

  tmpVal = EEPROM.read(6);
  if (tmpVal <= 12) {
    _time_month = tmpVal;
  }

  EEPROM.get(7, tmpVal); // int
  if (tmpVal >= 1872 && tmpVal <= 2127) {
    _time_year = tmpVal;
  }

  _personalizationSettings[0] = EEPROM.read(10);
  _personalizationSettings[1] = EEPROM.read(11);
  _personalizationSettings[2] = EEPROM.read(12);
  _personalizationSettings[3] = EEPROM.read(13);
  _personalizationSettings[4] = EEPROM.read(14);
  _personalizationSettings[5] = EEPROM.read(15);
  _personalizationSettings[6] = EEPROM.read(16);

  if (_hasAnalogicButtons) {
    //Initialize buttons - MENU/VOL+/VOL-
    pinMode(_menuButton, INPUT_PULLUP);
    pinMode(_volDownButton, INPUT_PULLUP);
    pinMode(_volUpButton, INPUT_PULLUP);
  }

  if (_serialEnabled) {
    // Initalize Serial for debug
    Serial.begin(_serialSpeed);

    // CAN-BUS from car
    Serial.println(F("Initialization CAN0"));
  }

  _can0.reset();
  _can0.setBitrate(_canSpeed, _canFreq);
  while (_can0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  if (_serialEnabled) {
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN1");
  }

  _can1.reset();
  _can1.setBitrate(_canSpeed, _canFreq);
  while (_can1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
  }

  setSyncProvider(RTC.get); // Get time from the RTC module
  if (timeStatus() != timeSet) {
    if (_serialEnabled) {
      Serial.println("Unable to sync with the RTC");
    }

    // Set default time (01/01/2020 00:00)
    setTime(_time_hour, _time_minute, 0, _time_day, _time_month, _time_year);
    EEPROM.update(5, _time_day);
    EEPROM.update(6, _time_month);
    EEPROM.put(7, _time_year);
  } else if (_serialEnabled) {
    Serial.println("RTC has set the system time");
  }

  // Set hour on CAN-BUS Clock
  _canMsgSnd.data[0] = hour();
  _canMsgSnd.data[1] = minute();
  _canMsgSnd.can_id = 0x228;
  _canMsgSnd.can_dlc = 2;
  _can0.sendMessage( & _canMsgSnd);

  // Send fake EMF version
  _canMsgSnd.data[0] = 0x25;
  _canMsgSnd.data[1] = 0x0A;
  _canMsgSnd.data[2] = 0x0B;
  _canMsgSnd.data[3] = 0x04;
  _canMsgSnd.data[4] = 0x0C;
  _canMsgSnd.data[5] = 0x01;
  _canMsgSnd.data[6] = 0x20;
  _canMsgSnd.data[7] = 0x11;
  _canMsgSnd.can_id = 0x5E5;
  _canMsgSnd.can_dlc = 8;
  _can0.sendMessage( & _canMsgSnd);

  if (_serialEnabled) {
    Serial.print("Current Time: ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year());

    Serial.print(" ");

    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());

    Serial.println();
  }
}

void PsaComfortCanAdapter::checkAnalogButtonsPressed() {
    int tmpVal;
      if (_hasAnalogicButtons) {
        // Receive buttons from the car
        if (((millis() - _lastDebounceTime) > _debounceDelay)) {
        tmpVal = 0;
        if (!digitalRead(_menuButton)) tmpVal += 0b001;
        if (!digitalRead(_volDownButton)) tmpVal += 0b010;
        if (!digitalRead(_volUpButton)) tmpVal += 0b100;
        if (tmpVal != _lastButtonState) {
            _buttonPushTime = millis();
            _buttonSendTime = 0;
            //buttonPushState = 0;
        }
        if ((millis() - _buttonPushTime) > 100) {
            switch (tmpVal) {
            case 0b001:
            //canMsgSnd.data[0] = 0x02; // MENU button
            _canMsgSnd.data[0] = 0x02;
            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = 0x00;
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0xFF;
            _canMsgSnd.data[6] = 0x00;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x122;
            _canMsgSnd.can_dlc = 8;
            // Menu button
            if (_buttonSendTime == 0) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Menu");
                }
                _lastDebounceTime = millis();
                _buttonSendTime = millis();
                //buttonPushState = 1;
            } else if (millis() - _buttonPushTime > 800 && ((millis() - _buttonPushTime < 2000 && millis() - _buttonSendTime > 600) || (millis() - _buttonPushTime > 2000 && millis() - _buttonSendTime > 350))) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Menu");
                }
                _buttonSendTime = millis();
                _lastDebounceTime = millis();
            }
            break;
            case 0b010:
            _canMsgSnd.data[0] = 0x04; //Volume down
            _canMsgSnd.data[1] = _scrollValue;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.can_id = 0x21F;
            _canMsgSnd.can_dlc = 3;
            // Menu button
            if (_buttonSendTime == 0) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Vol -");
                }
                _lastDebounceTime = millis();
                _buttonSendTime = millis();
                //buttonPushState = 1;
            } else if (millis() - _buttonPushTime > 800 && ((millis() - _buttonPushTime < 2000 && millis() - _buttonSendTime > 600) || (millis() - _buttonPushTime > 2000 && millis() - _buttonSendTime > 350))) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Vol -");
                }
                _buttonSendTime = millis();
                _lastDebounceTime = millis();
            }
            break;
            case 0b100:
            _canMsgSnd.data[0] = 0x08; //Volume down
            _canMsgSnd.data[1] = _scrollValue;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.can_id = 0x21F;
            _canMsgSnd.can_dlc = 3;
            // Menu button
            if (_buttonSendTime == 0) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Vol +");
                }
                _lastDebounceTime = millis();
                _buttonSendTime = millis();
                //buttonPushState = 1;
            } else if (millis() - _buttonPushTime > 800 && ((millis() - _buttonPushTime < 2000 && millis() - _buttonSendTime > 600) || (millis() - _buttonPushTime > 2000 && millis() - _buttonSendTime > 350))) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Vol +");
                }
                _buttonSendTime = millis();
                _lastDebounceTime = millis();
            }
            break;
            case 0b110:
            _canMsgSnd.data[0] = 0x0C; //Mute
            _canMsgSnd.data[1] = _scrollValue;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.can_id = 0x21F;
            _canMsgSnd.can_dlc = 3;
            // Menu button
            if (_buttonSendTime == 0) {
                _can1.sendMessage( & _canMsgSnd);
                if (_serialEnabled) {
                Serial.println("Mute");
                }
                _lastDebounceTime = millis();
                _buttonSendTime = millis();
                //buttonPushState = 1;
            }
            break;
            default:
            //buttonPushState = 0;
            _lastDebounceTime = millis();
            }
        }
        _lastButtonState = tmpVal;
        }
    }
}

void PsaComfortCanAdapter::transferCan2004Messages() {
    int tmpVal;
    // Receive CAN messages from the car
    if (_can0.readMessage( & _canMsgRcv) == MCP2515::ERROR_OK) {
        int id = _canMsgRcv.can_id;
        int len = _canMsgRcv.can_dlc;

        if (_debugCAN0) {
        Serial.print("FRAME:ID=");
        Serial.print(id);
        Serial.print(":LEN=");
        Serial.print(len);

        char tmp[3];
        for (int i = 0; i < len; i++) {
            Serial.print(":");

            snprintf(tmp, 3, "%02X", _canMsgRcv.data[i]);

            Serial.print(tmp);
        }

        Serial.println();

        _can1.sendMessage( & _canMsgRcv);
        } else if (!_debugCAN1) {
        if (id == 0x15B) {
            // Do not send back converted frames between networks
        } else if (id == 0x36 && len == 8) { // Economy Mode detection
            if (bitRead(_canMsgRcv.data[2], 7) == 1) {
            if (!_economyMode && _serialEnabled) {
                Serial.println("Economy mode ON");
            }

            _economyMode = true;
            } else {
            if (_economyMode && _serialEnabled) {
                Serial.println("Economy mode OFF");
            }

            _economyMode = false;
            }

            tmpVal = _canMsgRcv.data[3];

            // Fix brightness when car lights are ON - Brightness Instrument Panel "20" > "2F" (32 > 47) - Depends on your car
            if (_fixedBrightness && tmpVal >= 32) {
            _canMsgRcv.data[3] = 0x28; // Set fixed value to avoid low brightness due to incorrect CAN2010 Telematic calibration
            }
            _can1.sendMessage( & _canMsgRcv);
        } else if (id == 0xB6 && len == 8) {
            _engineRPM = ((_canMsgRcv.data[0] << 8) | _canMsgRcv.data[1]) * 0.125;
            if (_engineRPM > 0) {
            _engineRunning = true;
            } else {
            _engineRunning = false;
            }
            _vehicleSpeed = ((_canMsgRcv.data[2] << 8) | _canMsgRcv.data[3]) * 0.01;
            _can1.sendMessage( & _canMsgRcv);
        } else if (id == 0x336 && len == 3 && _emulateVIN) { // ASCII coded first 3 letters of VIN
            _canMsgSnd.data[0] = _vinNumber[0]; //V
            _canMsgSnd.data[1] = _vinNumber[1]; //F
            _canMsgSnd.data[2] = _vinNumber[2]; //3
            _canMsgSnd.can_id = 0x336;
            _canMsgSnd.can_dlc = 3;
            _can1.sendMessage( & _canMsgSnd);
        } else if (id == 0x3B6 && len == 6 && _emulateVIN) { // ASCII coded 4-9 letters of VIN
            _canMsgSnd.data[0] = _vinNumber[3]; //X
            _canMsgSnd.data[1] = _vinNumber[4]; //X
            _canMsgSnd.data[2] = _vinNumber[5]; //X
            _canMsgSnd.data[3] = _vinNumber[6]; //X
            _canMsgSnd.data[4] = _vinNumber[7]; //X
            _canMsgSnd.data[5] = _vinNumber[8]; //X
            _canMsgSnd.can_id = 0x3B6;
            _canMsgSnd.can_dlc = 6;
            _can1.sendMessage( & _canMsgSnd);
        } else if (id == 0x2B6 && len == 8 && _emulateVIN) { // ASCII coded 10-17 letters (last 8) of VIN
            _canMsgSnd.data[0] = _vinNumber[9]; //X
            _canMsgSnd.data[1] = _vinNumber[10]; //X
            _canMsgSnd.data[2] = _vinNumber[11]; //X
            _canMsgSnd.data[3] = _vinNumber[12]; //X
            _canMsgSnd.data[4] = _vinNumber[13]; //X
            _canMsgSnd.data[5] = _vinNumber[14]; //X
            _canMsgSnd.data[6] = _vinNumber[15]; //X
            _canMsgSnd.data[7] = _vinNumber[16]; //X
            _canMsgSnd.can_id = 0x2B6;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
        } else if (id == 0xE6 && len < 8) { // ABS status frame, increase length
            _canMsgSnd.data[0] = _canMsgRcv.data[0]; // Status lights / Alerts
            _canMsgSnd.data[1] = _canMsgRcv.data[1]; // Rear left rotations
            _canMsgSnd.data[2] = _canMsgRcv.data[2]; // Rear left rotations
            _canMsgSnd.data[3] = _canMsgRcv.data[3]; // Rear right rotations
            _canMsgSnd.data[4] = _canMsgRcv.data[4]; // Rear right rotations
            _canMsgSnd.data[5] = _canMsgRcv.data[5]; // Battery Voltage measured by ABS
            _canMsgSnd.data[6] = _canMsgRcv.data[6]; // STT / Slope / Emergency Braking
            _canMsgSnd.data[7] = 0x00; // Checksum / Counter : WIP
            _canMsgSnd.can_id = 0xE6;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
        } else if (id == 0x21F && len == 3) { // Steering wheel commands - Generic
            tmpVal = _canMsgRcv.data[0];
            _scrollValue = _canMsgRcv.data[1];

            if (bitRead(_canMsgRcv.data[0], 1) && _noFMUX && _steeringWheelCommands_Type == 0) { // Replace MODE/SRC by MENU (Valid for 208, C-Elysee calibrations for example)
            _canMsgSnd.data[0] = 0x80; // MENU button
            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = 0x00;
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0x02;
            _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x122;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
                _can0.sendMessage( & _canMsgSnd);
            }
            } else {
            _can1.sendMessage( & _canMsgRcv);

            if (_noFMUX || _hasAnalogicButtons) { // Fake FMUX Buttons in the car
                _canMsgSnd.data[0] = 0x00;
                _canMsgSnd.data[1] = 0x00;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _canMsgSnd.can_id = 0x122;
                _canMsgSnd.can_dlc = 8;
                _can1.sendMessage( & _canMsgSnd);
                if (_send_CAN2010_ForgedMessages) {
                _can0.sendMessage( & _canMsgSnd);
                }
            }
            }
        } else if (id == 0xA2 && _noFMUX && _steeringWheelCommands_Type == 1) { // Steering wheel commands - C4 I / C5 X7
            // Fake FMUX Buttons in the car
            _canMsgSnd.data[0] = 0x00;
            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = 0x00;
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0x02;
            _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            _canMsgSnd.data[7] = 0x00;

            if (bitRead(_canMsgRcv.data[1], 3)) { // MENU button pushed > MUSIC
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x00;
                _canMsgSnd.data[1] = 0x20;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 2)) { // MODE button pushed > NAV
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x00;
                _canMsgSnd.data[1] = 0x08;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 4)) { // ESC button pushed > APPS
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x00;
                _canMsgSnd.data[1] = 0x40;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 5)) { // OK button pushed > PHONE
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x00;
                _canMsgSnd.data[1] = 0x04;
                _canMsgSnd.data[2] = 0x08;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else {
            _pushA2 = false;
            _can1.sendMessage( & _canMsgRcv);
            }
            _canMsgSnd.can_id = 0x122;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0xA2 && _noFMUX && (_steeringWheelCommands_Type == 2 || _steeringWheelCommands_Type == 3 || _steeringWheelCommands_Type == 4 || _steeringWheelCommands_Type == 5)) { // Steering wheel commands - C4 I / C5 X7
            // Fake FMUX Buttons in the car
            _canMsgSnd.data[0] = 0x00;
            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = 0x00;
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0x02;
            _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
            _canMsgSnd.data[7] = 0x00;

            if (bitRead(_canMsgRcv.data[1], 3)) { // MENU button pushed > MENU
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x80;
                _canMsgSnd.data[1] = 0x00;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 2) && (_steeringWheelCommands_Type == 3 || _steeringWheelCommands_Type == 5)) { // Right push button / MODE/SRC > SRC
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x40;
                _canMsgSnd.data[1] = 0x00;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 4) && _steeringWheelCommands_Type == 4) { // ESC button pushed > SRC
            if (!_pushA2) {
                _canMsgSnd.data[0] = 0x40;
                _canMsgSnd.data[1] = 0x00;
                _canMsgSnd.data[2] = 0x00;
                _canMsgSnd.data[3] = 0x00;
                _canMsgSnd.data[4] = 0x00;
                _canMsgSnd.data[5] = 0x02;
                _canMsgSnd.data[6] = 0x00; // Volume potentiometer button
                _canMsgSnd.data[7] = 0x00;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 4) && _steeringWheelCommands_Type == 5) { // ESC button pushed > TRIP
            if (!_pushA2) {
                _pushTRIP = true;
                _pushA2 = true;
            }
            } else if (bitRead(_canMsgRcv.data[1], 2) && _steeringWheelCommands_Type == 4) { // Right push button / MODE/SRC > TRIP
            if (!_pushA2) {
                _pushTRIP = true;
                _pushA2 = true;
            }
            } else {
            _pushA2 = false;
            _can1.sendMessage( & _canMsgRcv);
            }
            _canMsgSnd.can_id = 0x122;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }

            if (_pushTRIP) {
            _pushTRIP = false;

            _canMsgSnd.data[0] = _statusTRIP[0];
            bitWrite(_canMsgSnd.data[0], 3, 1);
            _canMsgSnd.data[1] = _statusTRIP[1];
            _canMsgSnd.data[2] = _statusTRIP[2];
            _canMsgSnd.data[3] = _statusTRIP[3];
            _canMsgSnd.data[4] = _statusTRIP[4];
            _canMsgSnd.data[5] = _statusTRIP[5];
            _canMsgSnd.data[6] = _statusTRIP[6];
            _canMsgSnd.data[7] = _statusTRIP[7];
            _canMsgSnd.can_id = 0x221;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
                _can0.sendMessage( & _canMsgSnd);
            }
            }
        } else if (id == 0x217 && len == 8) { // Cache cluster status (CMB)
            _statusCMB[0] = _canMsgRcv.data[0];
            _statusCMB[1] = _canMsgRcv.data[1];
            _statusCMB[2] = _canMsgRcv.data[2];
            _statusCMB[3] = _canMsgRcv.data[3];
            _statusCMB[4] = _canMsgRcv.data[4];
            _statusCMB[5] = _canMsgRcv.data[5];
            _statusCMB[6] = _canMsgRcv.data[6];
            _statusCMB[7] = _canMsgRcv.data[7];

            _can1.sendMessage( & _canMsgRcv);
        } else if (id == 0x1D0 && len == 7 && _engineRunning) { // No fan activated if the engine is not ON on old models
            _leftTemp = _canMsgRcv.data[5];
            _rightTemp = _canMsgRcv.data[6];
            if (_leftTemp == _rightTemp) { // No other way to detect MONO mode
            _mono = true;
            _leftTemp = _leftTemp + 64;
            } else {
            _mono = false;
            }

            _fanOff = false;
            // Fan Speed BSI_2010 = "41" (Off) > "49" (Full speed)
            tmpVal = _canMsgRcv.data[2];
            if (tmpVal == 15) {
            _fanOff = true;
            _fanSpeed = 0x41;
            } else {
            _fanSpeed = (tmpVal + 66);
            }

            // Position Fan
            tmpVal = _canMsgRcv.data[3];

            if (tmpVal == 0x40) {
            _footAerator = false;
            _windShieldAerator = true;
            _centralAerator = false;
            } else if (tmpVal == 0x30) {
            _footAerator = false;
            _windShieldAerator = false;
            _centralAerator = true;
            } else if (tmpVal == 0x20) {
            _footAerator = true;
            _windShieldAerator = false;
            _centralAerator = false;
            } else if (tmpVal == 0x70) {
            _footAerator = false;
            _windShieldAerator = true;
            _centralAerator = true;
            } else if (tmpVal == 0x80) {
            _footAerator = true;
            _windShieldAerator = true;
            _centralAerator = true;
            } else if (tmpVal == 0x50) {
            _footAerator = true;
            _windShieldAerator = false;
            _centralAerator = true;
            } else if (tmpVal == 0x10) {
            _footAerator = false;
            _windShieldAerator = false;
            _centralAerator = false;
            } else if (tmpVal == 0x60) {
            _footAerator = true;
            _windShieldAerator = true;
            _centralAerator = false;
            } else {
            _footAerator = false;
            _windShieldAerator = false;
            _centralAerator = false;
            }

            tmpVal = _canMsgRcv.data[4];
            if (tmpVal == 0x10) {
            _deMist = true;
            _airRecycle = false;
            } else if (tmpVal == 0x30) {
            _airRecycle = true;
            } else {
            _airRecycle = false;
            }

            _autoFan = false;
            _deMist = false;

            tmpVal = _canMsgRcv.data[0];
            if (tmpVal == 0x11) {
            _deMist = true;
            _airConditioningON = true;
            _fanOff = false;
            } else if (tmpVal == 0x12) {
            _deMist = true;
            _airConditioningON = false;
            _fanOff = false;
            } else if (tmpVal == 0x21) {
            _deMist = true;
            _airConditioningON = true;
            _fanOff = false;
            } else if (tmpVal == 0xA2) {
            _fanOff = true;
            _airConditioningON = false;
            } else if (tmpVal == 0x22) {
            _airConditioningON = false;
            } else if (tmpVal == 0x20) {
            _airConditioningON = true;
            } else if (tmpVal == 0x02) {
            _airConditioningON = false;
            _autoFan = false;
            } else if (tmpVal == 0x00) {
            _airConditioningON = true;
            _autoFan = true;
            }

            if (!_footAerator && !_windShieldAerator && _centralAerator) {
            _fanPosition = 0x34;
            } else if (_footAerator && _windShieldAerator && _centralAerator) {
            _fanPosition = 0x84;
            } else if (!_footAerator && _windShieldAerator && _centralAerator) {
            _fanPosition = 0x74;
            } else if (_footAerator && !_windShieldAerator && _centralAerator) {
            _fanPosition = 0x54;
            } else if (_footAerator && !_windShieldAerator && !_centralAerator) {
            _fanPosition = 0x24;
            } else if (!_footAerator && _windShieldAerator && !_centralAerator) {
            _fanPosition = 0x44;
            } else if (_footAerator && _windShieldAerator && !_centralAerator) {
            _fanPosition = 0x64;
            } else {
            _fanPosition = 0x04; // Nothing
            }

            if (_deMist) {
            _fanSpeed = 0x10;
            _fanPosition = _fanPosition + 16;
            } else if (_autoFan) {
            _fanSpeed = 0x10;
            }

            if (_fanOff) {
            _airConditioningON = false;
            _fanSpeed = 0x41;
            _leftTemp = 0x00;
            _rightTemp = 0x00;
            _fanPosition = 0x04;
            }

            if (_airConditioningON) {
            _canMsgSnd.data[0] = 0x01; // A/C ON - Auto Soft : "00" / Auto Normal "01" / Auto Fast "02"
            } else {
            _canMsgSnd.data[0] = 0x09; // A/C OFF - Auto Soft : "08" / Auto Normal "09" / Auto Fast "0A"
            }

            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = _leftTemp;
            _canMsgSnd.data[4] = _rightTemp;
            _canMsgSnd.data[5] = _fanSpeed;
            _canMsgSnd.data[6] = _fanPosition;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x350;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0xF6 && len == 8) {
            tmpVal = _canMsgRcv.data[0];
            if (tmpVal > 128) {
            if (!_ignition && _serialEnabled) {
                Serial.println("Ignition ON");
            }

            _ignition = true;
            } else {
            if (_ignition && _serialEnabled) {
                Serial.println("Ignition OFF");
            }

            _ignition = false;
            }

            tmpVal = ceil(_canMsgRcv.data[5] / 2.0) - 40; // Temperatures can be negative but we only have 0 > 255, the new range is starting from -40°C
            if (_temperature != tmpVal) {
            _temperature = tmpVal;

            if (_serialEnabled) {
                Serial.print("Ext. Temperature: ");
                Serial.print(tmpVal);
                Serial.println("°C");
            }
            }

            _can1.sendMessage( & _canMsgRcv);
        } else if (id == 0x168 && len == 8) { // Instrument Panel - WIP
            _canMsgSnd.data[0] = _canMsgRcv.data[0]; // Alerts
            _canMsgSnd.data[1] = _canMsgRcv.data[1];
            _canMsgSnd.data[2] = _canMsgRcv.data[2];
            _canMsgSnd.data[3] = _canMsgRcv.data[3];
            _canMsgSnd.data[4] = _canMsgRcv.data[4];
            _canMsgSnd.data[5] = _canMsgRcv.data[5];
            bitWrite(_canMsgSnd.data[6], 7, 0);
            bitWrite(_canMsgSnd.data[6], 6, 1); // Ambiance
            bitWrite(_canMsgSnd.data[6], 5, 1); // EMF availability
            bitWrite(_canMsgSnd.data[6], 4, bitRead(_canMsgRcv.data[5], 0)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[6], 3, bitRead(_canMsgRcv.data[6], 7)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[6], 2, bitRead(_canMsgRcv.data[6], 6)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[6], 1, bitRead(_canMsgRcv.data[6], 5)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[6], 0, 0);
            _canMsgSnd.data[7] = _canMsgRcv.data[7];
            _canMsgSnd.can_id = 0x168;
            _canMsgSnd.can_dlc = 8;

            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x120 && _generatePOPups) { // Alerts journal / Diagnostic > Popup notifications - Work in progress
            // C5 (X7) Cluster is connected to CAN High Speed, no notifications are sent on CAN Low Speed, let's rebuild alerts from the journal (slighly slower than original alerts)
            // Bloc 1
            if (bitRead(_canMsgRcv.data[0], 7) == 0 && bitRead(_canMsgRcv.data[0], 6) == 1) {
            sendPOPup(bitRead(_canMsgRcv.data[1], 7), 5, 1, 0x00); // Engine oil pressure fault: stop the vehicle (STOP)
            sendPOPup(bitRead(_canMsgRcv.data[1], 6), 1, 1, 0x00); // Engine temperature fault: stop the vehicle (STOP)
            sendPOPup(bitRead(_canMsgRcv.data[1], 5), 138, 6, 0x00); // Charging system fault: repair needed (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[1], 4), 106, 1, 0x00); // Braking system fault: stop the vehicle (STOP)
            // bitRead(_canMsgRcv.data[1], 3); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[1], 2), 109, 2, 0x00); // Power steering fault: stop the vehicle (STOP)
            sendPOPup(bitRead(_canMsgRcv.data[1], 1), 3, 4, 0x00); // Top up coolant level (WARNING)
            // bitRead(_canMsgRcv.data[1], 0); // Fault with LKA (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[2], 7), 4, 4, 0x00); // Top up engine oil level (WARNING)
            // bitRead(_canMsgRcv.data[2], 6); // N/A
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[2], 5)); // Front right door
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[2], 4)); // Front left door
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[2], 3)); // Rear right door
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[2], 2)); // Rear left door
            bitWrite(_notificationParameters, 3, bitRead(_canMsgRcv.data[2], 0)); // Boot open
            // bitWrite(_notificationParameters, 2, ?); // Hood open
            bitWrite(_notificationParameters, 1, bitRead(_canMsgRcv.data[3], 7)); // Rear Screen open
            // bitWrite(_notificationParameters, 0, ?); // Fuel door open
            sendPOPup((bitRead(_canMsgRcv.data[2], 5) || bitRead(_canMsgRcv.data[2], 4) || bitRead(_canMsgRcv.data[2], 3) || bitRead(_canMsgRcv.data[2], 2) || bitRead(_canMsgRcv.data[2], 0) || bitRead(_canMsgRcv.data[3], 7)), 8, 8, _notificationParameters); // Left hand front door opened (WARNING) || Right hand front door opened (WARNING) || Left hand rear door opened (WARNING) || Right hand rear door opened (WARNING) || Boot open (WARNING) || Rear screen open (WARNING)
            // bitRead(_canMsgRcv.data[2], 1); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[3], 6), 107, 2, 0x00); // ESP/ASR system fault, repair the vehicle (WARNING)
            // bitRead(_canMsgRcv.data[3], 5); // Battery charge fault, stop the vehicle (WARNING)
            // bitRead(_canMsgRcv.data[3], 4); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[3], 3), 125, 6, 0x00); // Water in diesel fuel filter (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[3], 2), 103, 6, 0x00); // Have brake pads replaced (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[3], 1), 224, 10, 0x00); // Fuel level low (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[3], 0), 120, 6, 0x00); // Airbag(s) or seatbelt(s) pretensioner fault(s) (WARNING)
            // bitRead(_canMsgRcv.data[4], 7); // N/A
            // bitRead(_canMsgRcv.data[4], 6); // Engine fault, repair the vehicle (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[4], 5), 106, 2, 0x00); // ABS braking system fault, repair the vehicle (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[4], 4), 15, 4, 0x00); // Particle filter is full, please drive 20min to clean it (WARNING)
            // bitRead(_canMsgRcv.data[4], 3); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[4], 2), 129, 6, 0x00); // Particle filter additive level low (WARNING)
            // bitRead(_canMsgRcv.data[4], 1); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[4], 0), 17, 4, 0x00); // Suspension fault, repair the vehicle (WARNING)
            // bitRead(_canMsgRcv.data[5], 7); // Preheating deactivated, battery charge too low (INFO)
            // bitRead(_canMsgRcv.data[5], 6); // Preheating deactivated, fuel level too low (INFO)
            // bitRead(_canMsgRcv.data[5], 5); // Check the centre brake lamp (WARNING)
            // bitRead(_canMsgRcv.data[5], 4); // Retractable roof mechanism fault (WARNING)
            // sendPOPup(bitRead(_canMsgRcv.data[5], 3), ?, 8, 0x00); // Steering lock fault, repair the vehicle (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[5], 2), 131, 6, 0x00); // Electronic immobiliser fault (WARNING)
            // bitRead(_canMsgRcv.data[5], 1); // N/A
            // bitRead(_canMsgRcv.data[5], 0); // Roof operation not possible, system temperature too high (WARNING)
            // bitRead(_canMsgRcv.data[6], 7); // Roof operation not possible, start the engine (WARNING)
            // bitRead(_canMsgRcv.data[6], 6); // Roof operation not possible, apply parking brake (WARNING)
            // bitRead(_canMsgRcv.data[6], 5); // Hybrid system fault (STOP)
            // bitRead(_canMsgRcv.data[6], 4); // Automatic headlamp adjustment fault (WARNING)
            // bitRead(_canMsgRcv.data[6], 3); // Hybrid system fault (WARNING)
            // bitRead(_canMsgRcv.data[6], 2); // Hybrid system fault: speed restricted (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[6], 1), 223, 10, 0x00); // Top Up screenwash fluid level (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[6], 0), 227, 14, 0x00); // Replace remote control battery (INFO)
            // bitRead(_canMsgRcv.data[7], 7); // N/A
            // bitRead(_canMsgRcv.data[7], 6); // Preheating deactivated, set the clock (INFO)
            // bitRead(_canMsgRcv.data[7], 5); // Trailer connection fault (WARNING)
            // bitRead(_canMsgRcv.data[7], 4); // N/A
            // bitRead(_canMsgRcv.data[7], 3); // Tyre under-inflation (WARNING)
            // bitRead(_canMsgRcv.data[7], 2); // Driving aid camera limited visibility (INFO)
            // bitRead(_canMsgRcv.data[7], 1); // N/A
            // bitRead(_canMsgRcv.data[7], 0); // N/A
            }

            // Bloc 2
            if (bitRead(_canMsgRcv.data[0], 7) == 1 && bitRead(_canMsgRcv.data[0], 6) == 0) {
            // bitRead(_canMsgRcv.data[1], 7); // N/A
            // bitRead(_canMsgRcv.data[1], 6); // Electric mode not available : Particle filter regenerating (INFO)
            // bitRead(_canMsgRcv.data[1], 5); // N/A
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[1], 4)); // Front left tyre
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[1], 3)); // Front right tyre
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[1], 2)); // Rear right tyre
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[1], 1)); // Rear left tyre
            sendPOPup((bitRead(_canMsgRcv.data[1], 4) || bitRead(_canMsgRcv.data[1], 3) || bitRead(_canMsgRcv.data[1], 2) || bitRead(_canMsgRcv.data[1], 1)), 13, 6, _notificationParameters); // Puncture: Replace or repair the wheel (STOP)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[1], 0)); // Front right sidelamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[2], 7)); // Front left sidelamp
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[2], 6)); // Rear right sidelamp
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[2], 5)); // Rear left sidelamp
            sendPOPup((bitRead(_canMsgRcv.data[1], 0) || bitRead(_canMsgRcv.data[2], 7) || bitRead(_canMsgRcv.data[2], 6) || bitRead(_canMsgRcv.data[2], 5)), 160, 6, _notificationParameters); // Check sidelamps (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[2], 4)); // Right dipped beam headlamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[2], 3)); // Left dipped beam headlamp
            sendPOPup((bitRead(_canMsgRcv.data[2], 4) || bitRead(_canMsgRcv.data[2], 3)), 154, 6, _notificationParameters); // Check the dipped beam headlamps (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[2], 2)); // Right main beam headlamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[2], 1)); // Left main beam headlamp
            sendPOPup((bitRead(_canMsgRcv.data[2], 2) || bitRead(_canMsgRcv.data[2], 1)), 155, 6, _notificationParameters); // Check the main beam headlamps (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[2], 0)); // Right brake lamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[3], 7)); // Left brake lamp
            sendPOPup((bitRead(_canMsgRcv.data[2], 0) || bitRead(_canMsgRcv.data[3], 7)), 156, 6, _notificationParameters); // Check the RH brake lamp (WARNING) || Check the LH brake lamp (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[3], 6)); // Front right foglamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[3], 5)); // Front left foglamp
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[3], 4)); // Rear right foglamp
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[3], 3)); // Rear left foglamp
            sendPOPup((bitRead(_canMsgRcv.data[3], 6) || bitRead(_canMsgRcv.data[3], 5) || bitRead(_canMsgRcv.data[3], 4) || bitRead(_canMsgRcv.data[3], 3)), 157, 6, _notificationParameters); // Check the front foglamps (WARNING) || Check the front foglamps (WARNING) || Check the rear foglamps (WARNING) || Check the rear foglamps (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[3], 2)); // Front right direction indicator
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[3], 1)); // Front left direction indicator
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[3], 0)); // Rear right direction indicator
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[4], 7)); // Rear left direction indicator
            sendPOPup((bitRead(_canMsgRcv.data[3], 2) || bitRead(_canMsgRcv.data[3], 1) || bitRead(_canMsgRcv.data[3], 0) || bitRead(_canMsgRcv.data[4], 7)), 159, 6, _notificationParameters); // Check the direction indicators (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[4], 6)); // Right reversing lamp
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[4], 5)); // Left reversing lamp
            sendPOPup((bitRead(_canMsgRcv.data[4], 6) || bitRead(_canMsgRcv.data[4], 5)), 159, 6, _notificationParameters); // Check the reversing lamp(s) (WARNING)
            // bitRead(_canMsgRcv.data[4], 4); // N/A
            // bitRead(_canMsgRcv.data[4], 3); // N/A
            // bitRead(_canMsgRcv.data[4], 2); // N/A
            // bitRead(_canMsgRcv.data[4], 1); // N/A
            // bitRead(_canMsgRcv.data[4], 0); // N/A
            // bitRead(_canMsgRcv.data[5], 7); // N/A
            // bitRead(_canMsgRcv.data[5], 6); // N/A
            // bitRead(_canMsgRcv.data[5], 5); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[5], 4), 136, 8, 0x00); // Parking assistance system fault (WARNING)
            // bitRead(_canMsgRcv.data[5], 3); // N/A
            // bitRead(_canMsgRcv.data[5], 2); // N/A
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[5], 1)); // Front left tyre
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[5], 0)); // Front right tyre
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[6], 7)); // Rear right tyre
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[6], 5)); // Rear left tyre
            sendPOPup((bitRead(_canMsgRcv.data[5], 1) || bitRead(_canMsgRcv.data[5], 0) || bitRead(_canMsgRcv.data[6], 7) || bitRead(_canMsgRcv.data[6], 5)), 13, 8, _notificationParameters); // Adjust tyre pressures (WARNING)
            // bitRead(_canMsgRcv.data[6], 5); // Switch off lighting (INFO)
            // bitRead(_canMsgRcv.data[6], 4); // N/A
            sendPOPup((bitRead(_canMsgRcv.data[6], 3) || bitRead(_canMsgRcv.data[6], 1)), 190, 8, 0x00); // Emissions fault (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[6], 2), 192, 8, 0x00); // Emissions fault: Starting Prevented (WARNING)
            // bitRead(_canMsgRcv.data[6], 0); // N/A
            // bitRead(_canMsgRcv.data[7], 7); // N/A
            // bitRead(_canMsgRcv.data[7], 6); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[7], 5), 215, 10, 0x00); // "P" (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[7], 4), 216, 10, 0x00); // Ice warning (INFO)
            bitWrite(_statusOpenings, 7, bitRead(_canMsgRcv.data[7], 3)); // Front right door
            bitWrite(_statusOpenings, 6, bitRead(_canMsgRcv.data[7], 2)); // Front left door
            bitWrite(_statusOpenings, 5, bitRead(_canMsgRcv.data[7], 1)); // Rear right door
            bitWrite(_statusOpenings, 4, bitRead(_canMsgRcv.data[7], 0)); // Rear left door
            sendPOPup((bitRead(_canMsgRcv.data[7], 3) || bitRead(_canMsgRcv.data[7], 2) || bitRead(_canMsgRcv.data[7], 1) || bitRead(_canMsgRcv.data[7], 0) || bitRead(_statusOpenings, 3) || bitRead(_statusOpenings, 1)), 222, 8, _statusOpenings); // Front right door opened (INFO) || Front left door opened (INFO) || Rear right door opened (INFO) || Rear left door opened (INFO)
            }

            // Bloc 3
            if (bitRead(_canMsgRcv.data[0], 7) == 1 && bitRead(_canMsgRcv.data[0], 6) == 1) {
            bitWrite(_statusOpenings, 3, bitRead(_canMsgRcv.data[1], 7)); // Boot open
            // bitWrite(_statusOpenings, 2, ?); // Hood open
            bitWrite(_statusOpenings, 1, bitRead(_canMsgRcv.data[1], 5)); // Rear Screen open
            // bitWrite(_statusOpenings, 0, ?); // Fuel door open
            sendPOPup((bitRead(_canMsgRcv.data[1], 7) || bitRead(_canMsgRcv.data[1], 5) ||  bitRead(_statusOpenings, 7) ||  bitRead(_statusOpenings, 6) ||  bitRead(_statusOpenings, 5) ||  bitRead(_statusOpenings, 4)), 222, 8, _statusOpenings); // Boot open (INFO) || Rear Screen open (INFO)
            // bitRead(_canMsgRcv.data[1], 6); // Collision detection risk system fault (INFO)
            // bitRead(_canMsgRcv.data[1], 4); // N/A
            // bitRead(_canMsgRcv.data[1], 3); // N/A
            // bitRead(_canMsgRcv.data[1], 2); // N/A
            // bitRead(_canMsgRcv.data[1], 1); // N/A
            // bitRead(_canMsgRcv.data[1], 0); // N/A
            // bitRead(_canMsgRcv.data[2], 7); // N/A
            // bitRead(_canMsgRcv.data[2], 6); // N/A
            // bitRead(_canMsgRcv.data[2], 5); // N/A
            sendPOPup(bitRead(_canMsgRcv.data[2], 4), 100, 6, 0x00); // Parking brake fault (WARNING)
            // bitRead(_canMsgRcv.data[2], 3); // Active spoiler fault: speed restricted (WARNING)
            // bitRead(_canMsgRcv.data[2], 2); // Automatic braking system fault (INFO)
            // bitRead(_canMsgRcv.data[2], 1); // Directional headlamps fault (WARNING)
            // bitRead(_canMsgRcv.data[2], 0); // N/A
            // bitRead(_canMsgRcv.data[3], 7); // N/A
            // bitRead(_canMsgRcv.data[3], 6); // N/A
            // bitRead(_canMsgRcv.data[3], 5); // N/A
            // bitRead(_canMsgRcv.data[3], 4); // N/A
            // bitRead(_canMsgRcv.data[3], 3); // N/A
            if (_isBVMP) {
                sendPOPup(bitRead(_canMsgRcv.data[3], 2), 122, 4, 0x00); // Gearbox fault (WARNING)
            } else {
                sendPOPup(bitRead(_canMsgRcv.data[3], 2), 110, 4, 0x00); // Gearbox fault (WARNING)
            }
            // bitRead(_canMsgRcv.data[3], 1); // N/A
            // bitRead(_canMsgRcv.data[3], 0); // N/A
            // bitRead(_canMsgRcv.data[4], 7); // N/A
            // bitRead(_canMsgRcv.data[4], 6); // N/A
            // bitRead(_canMsgRcv.data[4], 5); // N/A
            // bitRead(_canMsgRcv.data[4], 4); // N/A
            // bitRead(_canMsgRcv.data[4], 3); // N/A
            // bitRead(_canMsgRcv.data[4], 2); // Engine fault (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[4], 1), 17, 3, 0x00); // Suspension fault: limit your speed to 90km/h (WARNING)
            // bitRead(_canMsgRcv.data[4], 0); // N/A
            // bitRead(_canMsgRcv.data[5], 7); // N/A
            // bitRead(_canMsgRcv.data[5], 6); // N/A
            // bitRead(_canMsgRcv.data[5], 5); // N/A
            // bitRead(_canMsgRcv.data[5], 4); // N/A
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[5], 3)); // Front left tyre
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[5], 2)); // Front right tyre
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[5], 1)); // Rear right tyre
            bitWrite(_notificationParameters, 4, bitRead(_canMsgRcv.data[5], 0)); // Rear left tyre
            sendPOPup((bitRead(_canMsgRcv.data[5], 3) || bitRead(_canMsgRcv.data[5], 2) || bitRead(_canMsgRcv.data[5], 1) || bitRead(_canMsgRcv.data[5], 0)), 229, 10, _notificationParameters); // Sensor fault: Left hand front tyre pressure not monitored (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[6], 7), 18, 4, 0x00); // Suspension fault: repair the vehicle (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[6], 6), 109, 4, 0x00); // Power steering fault: repair the vehicle (WARNING)
            // bitRead(_canMsgRcv.data[6], 5); // N/A
            // bitRead(_canMsgRcv.data[6], 4); // N/A
            // bitRead(_canMsgRcv.data[6], 3); // Inter-vehicle time measurement fault (WARNING)
            // bitRead(_canMsgRcv.data[6], 2); // Engine fault, stop the vehicle (STOP)
            // bitRead(_canMsgRcv.data[6], 1); // Fault with LKA (INFO)
            // bitRead(_canMsgRcv.data[6], 0); // Tyre under-inflation detection system fault (WARNING)
            _notificationParameters = 0x00;
            bitWrite(_notificationParameters, 7, bitRead(_canMsgRcv.data[7], 7)); // Front left tyre
            bitWrite(_notificationParameters, 6, bitRead(_canMsgRcv.data[7], 6)); // Front right tyre
            bitWrite(_notificationParameters, 5, bitRead(_canMsgRcv.data[7], 5)); // Rear right tyre
            //bitWrite(_notificationParameters, 4, ?); // Rear left tyre
            sendPOPup((bitRead(_canMsgRcv.data[7], 7) || bitRead(_canMsgRcv.data[7], 6) || bitRead(_canMsgRcv.data[7], 5)), 183, 8, _notificationParameters); // Underinflated wheel, ajust pressure and reset (INFO)
            // bitRead(_canMsgRcv.data[7], 4); // Spare wheel fitted: driving aids deactivated (INFO)
            // bitRead(_canMsgRcv.data[7], 3); // Automatic braking disabled (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[7], 2), 188, 6, 0x00); // Refill AdBlue (WARNING)
            sendPOPup(bitRead(_canMsgRcv.data[7], 1), 187, 10, 0x00); // Refill AdBlue (INFO)
            sendPOPup(bitRead(_canMsgRcv.data[7], 0), 189, 4, 0x00); // Impossible engine start, refill AdBlue (WARNING)
            }

            _can1.sendMessage( & _canMsgRcv); // Forward original frame
        } else if (id == 0x221) { // Trip info
            _statusTRIP[0] = _canMsgRcv.data[0];
            _statusTRIP[1] = _canMsgRcv.data[1];
            _statusTRIP[2] = _canMsgRcv.data[2];
            _statusTRIP[3] = _canMsgRcv.data[3];
            _statusTRIP[4] = _canMsgRcv.data[4];
            _statusTRIP[5] = _canMsgRcv.data[5];
            _statusTRIP[6] = _canMsgRcv.data[6];
            _statusTRIP[7] = _canMsgRcv.data[7];
            _can1.sendMessage( & _canMsgRcv); // Forward original frame

            _customTimeStamp = (long) hour() * (long) 3600 + minute() * 60 + second();
            _daysSinceYearStart = daysSinceYearStartFct();

            _canMsgSnd.data[0] = (((1 << 8) - 1) & (_customTimeStamp >> (12)));
            _canMsgSnd.data[1] = (((1 << 8) - 1) & (_customTimeStamp >> (4)));
            _canMsgSnd.data[2] = (((((1 << 4) - 1) & (_customTimeStamp)) << 4)) + (((1 << 4) - 1) & (_daysSinceYearStart >> (8)));
            _canMsgSnd.data[3] = (((1 << 8) - 1) & (_daysSinceYearStart));
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0xC0;
            _canMsgSnd.data[6] = _languageID;
            _canMsgSnd.can_id = 0x3F6; // Fake EMF Time frame
            _canMsgSnd.can_dlc = 7;

            _can0.sendMessage( & _canMsgSnd);
        } else if (id == 0x128 && len == 8) { // Instrument Panel
            _canMsgSnd.data[0] = _canMsgRcv.data[4]; // Main driving lights
            bitWrite(_canMsgSnd.data[1], 7, bitRead(_canMsgRcv.data[6], 7)); // Gearbox report
            bitWrite(_canMsgSnd.data[1], 6, bitRead(_canMsgRcv.data[6], 6)); // Gearbox report
            bitWrite(_canMsgSnd.data[1], 5, bitRead(_canMsgRcv.data[6], 5)); // Gearbox report
            bitWrite(_canMsgSnd.data[1], 4, bitRead(_canMsgRcv.data[6], 4)); // Gearbox report
            bitWrite(_canMsgSnd.data[1], 3, bitRead(_canMsgRcv.data[6], 3)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[1], 2, bitRead(_canMsgRcv.data[6], 2)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[1], 1, bitRead(_canMsgRcv.data[6], 1)); // Gearbox report while driving
            bitWrite(_canMsgSnd.data[1], 0, bitRead(_canMsgRcv.data[6], 0)); // Gearbox report blinking
            bitWrite(_canMsgSnd.data[2], 7, bitRead(_canMsgRcv.data[7], 7)); // Arrow blinking
            bitWrite(_canMsgSnd.data[2], 6, bitRead(_canMsgRcv.data[7], 6)); // BVA mode
            bitWrite(_canMsgSnd.data[2], 5, bitRead(_canMsgRcv.data[7], 5)); // BVA mode
            bitWrite(_canMsgSnd.data[2], 4, bitRead(_canMsgRcv.data[7], 4)); // BVA mode
            bitWrite(_canMsgSnd.data[2], 3, bitRead(_canMsgRcv.data[7], 3)); // Arrow type
            bitWrite(_canMsgSnd.data[2], 2, bitRead(_canMsgRcv.data[7], 2)); // Arrow type
            if (bitRead(_canMsgRcv.data[7], 1) == 1 && bitRead(_canMsgRcv.data[7], 0) == 0) { // BVMP to BVA
            _isBVMP = true;
            bitWrite(_canMsgSnd.data[2], 1, 0); // Gearbox type
            bitWrite(_canMsgSnd.data[2], 0, 0); // Gearbox type
            } else {
            bitWrite(_canMsgSnd.data[2], 1, bitRead(_canMsgRcv.data[7], 1)); // Gearbox type
            bitWrite(_canMsgSnd.data[2], 0, bitRead(_canMsgRcv.data[7], 0)); // Gearbox type
            }
            bitWrite(_canMsgSnd.data[3], 7, bitRead(_canMsgRcv.data[1], 7)); // Service
            bitWrite(_canMsgSnd.data[3], 6, bitRead(_canMsgRcv.data[1], 6)); // STOP
            bitWrite(_canMsgSnd.data[3], 5, bitRead(_canMsgRcv.data[2], 5)); // Child security
            bitWrite(_canMsgSnd.data[3], 4, bitRead(_canMsgRcv.data[0], 7)); // Passenger Airbag
            bitWrite(_canMsgSnd.data[3], 3, bitRead(_canMsgRcv.data[3], 2)); // Foot on brake
            bitWrite(_canMsgSnd.data[3], 2, bitRead(_canMsgRcv.data[3], 1)); // Foot on brake
            bitWrite(_canMsgSnd.data[3], 1, bitRead(_canMsgRcv.data[0], 5)); // Parking brake
            bitWrite(_canMsgSnd.data[3], 0, 0); // Electric parking brake
            bitWrite(_canMsgSnd.data[4], 7, bitRead(_canMsgRcv.data[0], 2)); // Diesel pre-heating
            bitWrite(_canMsgSnd.data[4], 6, bitRead(_canMsgRcv.data[1], 4)); // Opening open
            bitWrite(_canMsgSnd.data[4], 5, bitRead(_canMsgRcv.data[3], 4)); // Automatic parking
            bitWrite(_canMsgSnd.data[4], 4, bitRead(_canMsgRcv.data[3], 3)); // Automatic parking blinking
            bitWrite(_canMsgSnd.data[4], 3, 0); // Automatic high beam
            bitWrite(_canMsgSnd.data[4], 2, bitRead(_canMsgRcv.data[2], 4)); // ESP Disabled
            bitWrite(_canMsgSnd.data[4], 1, bitRead(_canMsgRcv.data[2], 3)); // ESP active
            bitWrite(_canMsgSnd.data[4], 0, bitRead(_canMsgRcv.data[2], 2)); // Active suspension
            bitWrite(_canMsgSnd.data[5], 7, bitRead(_canMsgRcv.data[0], 4)); // Low fuel
            bitWrite(_canMsgSnd.data[5], 6, bitRead(_canMsgRcv.data[0], 6)); // Driver seatbelt
            bitWrite(_canMsgSnd.data[5], 5, bitRead(_canMsgRcv.data[3], 7)); // Driver seatbelt blinking
            bitWrite(_canMsgSnd.data[5], 4, bitRead(_canMsgRcv.data[0], 1)); // Passenger seatbelt
            bitWrite(_canMsgSnd.data[5], 3, bitRead(_canMsgRcv.data[3], 6)); // Passenger seatbelt Blinking
            bitWrite(_canMsgSnd.data[5], 2, 0); // SCR
            bitWrite(_canMsgSnd.data[5], 1, 0); // SCR
            bitWrite(_canMsgSnd.data[5], 0, bitRead(_canMsgRcv.data[5], 6)); // Rear left seatbelt
            bitWrite(_canMsgSnd.data[6], 7, bitRead(_canMsgRcv.data[5], 5)); // Rear seatbelt left blinking
            bitWrite(_canMsgSnd.data[6], 6, bitRead(_canMsgRcv.data[5], 2)); // Rear right seatbelt
            bitWrite(_canMsgSnd.data[6], 5, bitRead(_canMsgRcv.data[5], 1)); // Rear right seatbelt blinking
            bitWrite(_canMsgSnd.data[6], 4, bitRead(_canMsgRcv.data[5], 4)); // Rear middle seatbelt
            bitWrite(_canMsgSnd.data[6], 3, bitRead(_canMsgRcv.data[5], 3)); // Rear middle seatbelt blinking
            bitWrite(_canMsgSnd.data[6], 2, bitRead(_canMsgRcv.data[5], 7)); // Instrument Panel ON
            bitWrite(_canMsgSnd.data[6], 1, bitRead(_canMsgRcv.data[2], 1)); // Warnings
            bitWrite(_canMsgSnd.data[6], 0, 0); // Passenger protection
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x128;
            _canMsgSnd.can_dlc = 8;

            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x3A7 && len == 8) { // Maintenance
            _canMsgSnd.data[0] = 0x40;
            _canMsgSnd.data[1] = _canMsgRcv.data[5]; // Value x255 +
            _canMsgSnd.data[2] = _canMsgRcv.data[6]; // Value x1 = Number of days till maintenance (FF FF if disabled)
            _canMsgSnd.data[3] = _canMsgRcv.data[3]; // Value x5120 +
            _canMsgSnd.data[4] = _canMsgRcv.data[4]; // Value x20 = km left till maintenance
            _canMsgSnd.can_id = 0x3E7; // New maintenance frame ID
            _canMsgSnd.can_dlc = 5;

            if (_serialEnabled && !_maintenanceDisplayed) {
            Serial.print("Next maintenance in: ");
            if (_canMsgRcv.data[3] != 0xFF && _canMsgRcv.data[4] != 0xFF) {
                tmpVal = (_canMsgRcv.data[3] * 5120) + (_canMsgRcv.data[4] * 20);
                Serial.print(tmpVal);
                Serial.println(" km");
            }
            if (_canMsgRcv.data[5] != 0xFF && _canMsgRcv.data[6] != 0xFF) {
                tmpVal = (_canMsgRcv.data[5] * 255) + _canMsgRcv.data[6];
                Serial.print(tmpVal);
                Serial.println(" days");
            }
            _maintenanceDisplayed = true;
            }

            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x1A8 && len == 8) { // Cruise control
            _can1.sendMessage( & _canMsgRcv);

            _canMsgSnd.data[0] = _canMsgRcv.data[1];
            _canMsgSnd.data[1] = _canMsgRcv.data[2];
            _canMsgSnd.data[2] = _canMsgRcv.data[0];
            _canMsgSnd.data[3] = 0x80;
            _canMsgSnd.data[4] = 0x14;
            _canMsgSnd.data[5] = 0x7F;
            _canMsgSnd.data[6] = 0xFF;
            _canMsgSnd.data[7] = 0x98;
            _canMsgSnd.can_id = 0x228; // New cruise control frame ID
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x2D7 && len == 5 && _listenCAN2004Language) { // CAN2004 Matrix
            tmpVal = _canMsgRcv.data[0];
            if (tmpVal > 32) {
            _kmL = true;
            tmpVal = tmpVal - 32;
            }

            if (tmpVal <= 32 && _languageID_CAN2004 != tmpVal) {
            _languageID_CAN2004 = tmpVal;
            EEPROM.update(1, _languageID_CAN2004);

            // Change language and unit on ID 608 for CAN2010 Telematic language change
            _languageAndUnitNum = (_languageID_CAN2004 * 4) + 128;
            if (_kmL) {
                _languageAndUnitNum = _languageAndUnitNum + 1;
            }
            EEPROM.update(0, _languageAndUnitNum);

            if (_serialEnabled) {
                Serial.print("CAN2004 Matrix - Change Language: ");
                Serial.print(tmpVal);
                Serial.println();
            }
            } else {
            Serial.print("CAN2004 Matrix - Unsupported language ID: ");
            Serial.print(tmpVal);
            Serial.println();
            }
        } else if (id == 0x361) { // Personalization menus availability
            bitWrite(_canMsgSnd.data[0], 7, 1); // Parameters availability
            bitWrite(_canMsgSnd.data[0], 6, bitRead(_canMsgRcv.data[2], 3)); // Beam
            bitWrite(_canMsgSnd.data[0], 5, 0); // Lighting
            bitWrite(_canMsgSnd.data[0], 4, bitRead(_canMsgRcv.data[3], 7)); // Adaptative lighting
            bitWrite(_canMsgSnd.data[0], 3, bitRead(_canMsgRcv.data[4], 1)); // SAM
            bitWrite(_canMsgSnd.data[0], 2, bitRead(_canMsgRcv.data[4], 2)); // Ambiance lighting
            bitWrite(_canMsgSnd.data[0], 1, bitRead(_canMsgRcv.data[2], 0)); // Automatic headlights
            bitWrite(_canMsgSnd.data[0], 0, bitRead(_canMsgRcv.data[3], 6)); // Daytime running lights
            bitWrite(_canMsgSnd.data[1], 7, bitRead(_canMsgRcv.data[5], 5)); // AAS
            bitWrite(_canMsgSnd.data[1], 6, bitRead(_canMsgRcv.data[3], 5)); // Wiper in reverse
            bitWrite(_canMsgSnd.data[1], 5, bitRead(_canMsgRcv.data[2], 4)); // Guide-me home lighting
            bitWrite(_canMsgSnd.data[1], 4, bitRead(_canMsgRcv.data[1], 2)); // Driver welcome
            bitWrite(_canMsgSnd.data[1], 3, bitRead(_canMsgRcv.data[2], 6)); // Motorized tailgate
            bitWrite(_canMsgSnd.data[1], 2, bitRead(_canMsgRcv.data[2], 0)); // Selective openings - Rear
            bitWrite(_canMsgSnd.data[1], 1, bitRead(_canMsgRcv.data[2], 7)); // Selective openings - Key
            bitWrite(_canMsgSnd.data[1], 0, 0); // Selective openings
            bitWrite(_canMsgSnd.data[2], 7, 1); // TNB - Seatbelt indicator
            bitWrite(_canMsgSnd.data[2], 6, 1); // XVV - Custom cruise limits
            bitWrite(_canMsgSnd.data[2], 5, bitRead(_canMsgRcv.data[1], 4)); // Configurable button
            bitWrite(_canMsgSnd.data[2], 4, bitRead(_canMsgRcv.data[2], 2)); // Automatic parking brake
            bitWrite(_canMsgSnd.data[2], 3, 0); // Sound Harmony
            bitWrite(_canMsgSnd.data[2], 2, 0); // Rear mirror index
            bitWrite(_canMsgSnd.data[2], 1, 0);
            bitWrite(_canMsgSnd.data[2], 0, 0);
            bitWrite(_canMsgSnd.data[3], 7, 1); // DSG Reset
            bitWrite(_canMsgSnd.data[3], 6, 0); // Front Collision Warning
            bitWrite(_canMsgSnd.data[3], 5, 0);
            bitWrite(_canMsgSnd.data[3], 4, 1); // XVV - Custom cruise limits Menu
            bitWrite(_canMsgSnd.data[3], 3, 1); // Recommended speed indicator
            bitWrite(_canMsgSnd.data[3], 2, bitRead(_canMsgRcv.data[5], 6)); // DSG - Underinflating (3b)
            bitWrite(_canMsgSnd.data[3], 1, bitRead(_canMsgRcv.data[5], 5)); // DSG - Underinflating (3b)
            bitWrite(_canMsgSnd.data[3], 0, bitRead(_canMsgRcv.data[5], 4)); // DSG - Underinflating (3b)
            _canMsgSnd.data[4] = 0x00;
            _canMsgSnd.data[5] = 0x00;
            _canMsgSnd.can_id = 0x361;
            _canMsgSnd.can_dlc = 6;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x260 && len == 8) { // Personalization settings status
            // Do not forward original message, it has been completely redesigned on CAN2010
            // Also forge missing messages from CAN2004

            if (_canMsgRcv.data[0] == 0x01) { // User profile 1
            _canMsgSnd.data[0] = _languageAndUnitNum;
            bitWrite(_canMsgSnd.data[1], 7, (_mpgMi)?1:0);
            bitWrite(_canMsgSnd.data[1], 6, (_temperatureInF)?1:0);
            bitWrite(_canMsgSnd.data[1], 5, 0); // Ambiance level
            bitWrite(_canMsgSnd.data[1], 4, 1); // Ambiance level
            bitWrite(_canMsgSnd.data[1], 3, 1); // Ambiance level
            bitWrite(_canMsgSnd.data[1], 2, 1); // Parameters availability
            bitWrite(_canMsgSnd.data[1], 1, 0); // Sound Harmony
            bitWrite(_canMsgSnd.data[1], 0, 0); // Sound Harmony
            bitWrite(_canMsgSnd.data[2], 7, bitRead(_canMsgRcv.data[1], 0)); // Automatic parking brake
            bitWrite(_canMsgSnd.data[2], 6, bitRead(_canMsgRcv.data[1], 7)); // Selective openings - Key
            bitWrite(_canMsgSnd.data[2], 5, bitRead(_canMsgRcv.data[1], 4)); // Selective openings
            bitWrite(_canMsgSnd.data[2], 4, bitRead(_canMsgRcv.data[1], 5)); // Selective openings - Rear
            bitWrite(_canMsgSnd.data[2], 3, bitRead(_canMsgRcv.data[1], 1)); // Driver Welcome
            bitWrite(_canMsgSnd.data[2], 2, bitRead(_canMsgRcv.data[2], 7)); // Adaptative lighting
            bitWrite(_canMsgSnd.data[2], 1, bitRead(_canMsgRcv.data[3], 6)); // Daytime running lights
            bitWrite(_canMsgSnd.data[2], 0, bitRead(_canMsgRcv.data[3], 7)); // Ambiance lighting
            bitWrite(_canMsgSnd.data[3], 7, bitRead(_canMsgRcv.data[2], 5)); // Guide-me home lighting
            bitWrite(_canMsgSnd.data[3], 6, bitRead(_canMsgRcv.data[2], 1)); // Duration Guide-me home lighting (2b)
            bitWrite(_canMsgSnd.data[3], 5, bitRead(_canMsgRcv.data[2], 0)); // Duration Guide-me home lighting (2b)
            bitWrite(_canMsgSnd.data[3], 4, bitRead(_canMsgRcv.data[2], 6)); // Beam
            bitWrite(_canMsgSnd.data[3], 3, 0); // Lighting ?
            bitWrite(_canMsgSnd.data[3], 2, 0); // Duration Lighting (2b) ?
            bitWrite(_canMsgSnd.data[3], 1, 0); // Duration Lighting (2b) ?
            bitWrite(_canMsgSnd.data[3], 0, bitRead(_canMsgRcv.data[2], 4)); // Automatic headlights
            bitWrite(_canMsgSnd.data[4], 7, bitRead(_canMsgRcv.data[5], 6)); // AAS
            bitWrite(_canMsgSnd.data[4], 6, bitRead(_canMsgRcv.data[6], 5)); // SAM
            bitWrite(_canMsgSnd.data[4], 5, bitRead(_canMsgRcv.data[5], 4)); // Wiper in reverse
            bitWrite(_canMsgSnd.data[4], 4, 0); // Motorized tailgate
            bitWrite(_canMsgSnd.data[4], 3, bitRead(_canMsgRcv.data[7], 7)); // Configurable button
            bitWrite(_canMsgSnd.data[4], 2, bitRead(_canMsgRcv.data[7], 6)); // Configurable button
            bitWrite(_canMsgSnd.data[4], 1, bitRead(_canMsgRcv.data[7], 5)); // Configurable button
            bitWrite(_canMsgSnd.data[4], 0, bitRead(_canMsgRcv.data[7], 4)); // Configurable button

            _personalizationSettings[7] = _canMsgSnd.data[1];
            _personalizationSettings[8] = _canMsgSnd.data[2];
            _personalizationSettings[9] = _canMsgSnd.data[3];
            _personalizationSettings[10] = _canMsgSnd.data[4];
            } else { // Cached information if any other profile
            _canMsgSnd.data[0] = _languageAndUnitNum;
            _canMsgSnd.data[1] = _personalizationSettings[7];
            _canMsgSnd.data[2] = _personalizationSettings[8];
            _canMsgSnd.data[3] = _personalizationSettings[9];
            _canMsgSnd.data[4] = _personalizationSettings[10];
            }
            _canMsgSnd.data[5] = 0x00;
            _canMsgSnd.data[6] = 0x00;
            _canMsgSnd.can_id = 0x260;
            _canMsgSnd.can_dlc = 7;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }

            bitWrite(_canMsgSnd.data[0], 7, 0);
            bitWrite(_canMsgSnd.data[0], 6, 0);
            bitWrite(_canMsgSnd.data[0], 5, 0);
            bitWrite(_canMsgSnd.data[0], 4, 0);
            bitWrite(_canMsgSnd.data[0], 3, 0);
            bitWrite(_canMsgSnd.data[0], 2, 1); // Parameters validity
            bitWrite(_canMsgSnd.data[0], 1, 0); // User profile
            bitWrite(_canMsgSnd.data[0], 0, 1); // User profile = 1
            _canMsgSnd.data[1] = _personalizationSettings[0];
            _canMsgSnd.data[2] = _personalizationSettings[1];
            _canMsgSnd.data[3] = _personalizationSettings[2];
            _canMsgSnd.data[4] = _personalizationSettings[3];
            _canMsgSnd.data[5] = _personalizationSettings[4];
            _canMsgSnd.data[6] = _personalizationSettings[5];
            _canMsgSnd.data[7] = _personalizationSettings[6];
            _canMsgSnd.can_id = 0x15B; // Personalization frame status
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);

            if (!_telematicPresent && _ignition) {
            _canMsgSnd.data[0] = 0x00;
            _canMsgSnd.data[1] = 0x10;
            _canMsgSnd.data[2] = 0xFF;
            _canMsgSnd.data[3] = 0xFF;
            _canMsgSnd.data[4] = 0x7F;
            _canMsgSnd.data[5] = 0xFF;
            _canMsgSnd.data[6] = 0x00;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x167; // Fake EMF status frame
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);
            }

            // Economy mode simulation
            if (_economyMode && _economyModeEnabled) {
            _canMsgSnd.data[0] = 0x14;
            if (_ignition) {
                _canMsgSnd.data[5] = 0x0E;
            } else {
                _canMsgSnd.data[5] = 0x0C;
            }
            } else {
            if (_engineRunning) {
                _canMsgSnd.data[0] = 0x54;
            } else {
                _canMsgSnd.data[0] = 0x04;
            }
            _canMsgSnd.data[5] = 0x0F;
            }
            _canMsgSnd.data[1] = 0x03;
            _canMsgSnd.data[2] = 0xDE;

            _canMsgSnd.data[3] = 0x00; // Increasing value,
            _canMsgSnd.data[4] = 0x00; // counter ?

            _canMsgSnd.data[6] = 0xFE;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x236;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }

            // Current Time
            // If time is synced
            if (timeStatus() != timeNotSet) {
            _canMsgSnd.data[0] = (year() - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
            _canMsgSnd.data[1] = month();
            _canMsgSnd.data[2] = day();
            _canMsgSnd.data[3] = hour();
            _canMsgSnd.data[4] = minute();
            _canMsgSnd.data[5] = 0x3F;
            _canMsgSnd.data[6] = 0xFE;
            } else {
            _canMsgSnd.data[0] = (_time_year - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
            _canMsgSnd.data[1] = _time_month;
            _canMsgSnd.data[2] = _time_day;
            _canMsgSnd.data[3] = _time_hour;
            _canMsgSnd.data[4] = _time_minute;
            _canMsgSnd.data[5] = 0x3F;
            _canMsgSnd.data[6] = 0xFE;
            }
            _canMsgSnd.can_id = 0x276;
            _canMsgSnd.can_dlc = 7;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
            _can0.sendMessage( & _canMsgSnd);
            }

            if (!_engineRunning) {
            _airConditioningON = false;
            _fanSpeed = 0x41;
            _leftTemp = 0x00;
            _rightTemp = 0x00;
            _fanPosition = 0x04;

            _canMsgSnd.data[0] = 0x09;
            _canMsgSnd.data[1] = 0x00;
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = _leftTemp;
            _canMsgSnd.data[4] = _rightTemp;
            _canMsgSnd.data[5] = _fanSpeed;
            _canMsgSnd.data[6] = _fanPosition;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x350;
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
            if (_send_CAN2010_ForgedMessages) {
                _can0.sendMessage( & _canMsgSnd);
            }
            }
        } else {
            _can1.sendMessage( & _canMsgRcv);
        }
        } else {
        _can1.sendMessage( & _canMsgRcv);
        }
    }
}

void PsaComfortCanAdapter::transferCan2010Messages() {
    int tmpVal;
    // Forward messages from the CAN2010 device(s) to the car
    if (_can1.readMessage( & _canMsgRcv) == MCP2515::ERROR_OK) {
        int id = _canMsgRcv.can_id;
        int len = _canMsgRcv.can_dlc;

        if (_debugCAN1) {
        Serial.print("FRAME:ID=");
        Serial.print(id);
        Serial.print(":LEN=");
        Serial.print(len);

        char tmp[3];
        for (int i = 0; i < len; i++) {
            Serial.print(":");

            snprintf(tmp, 3, "%02X", _canMsgRcv.data[i]);

            Serial.print(tmp);
        }

        Serial.println();

        _can0.sendMessage( & _canMsgRcv);
        } else if (!_debugCAN0) {
        if (id == 0x260 || id == 0x361) {
            // Do not send back converted frames between networks
        } else if (id == 0x39B && len == 5) {
            _time_year = _canMsgRcv.data[0] + 1872; // Year would not fit inside one byte (0 > 255), add 1872 and you get this new range (1872 > 2127)
            _time_month = _canMsgRcv.data[1];
            _time_day = _canMsgRcv.data[2];
            _time_hour = _canMsgRcv.data[3];
            _time_minute = _canMsgRcv.data[4];

            setTime(_time_hour, _time_minute, 0, _time_day, _time_month, _time_year);
            RTC.set(now()); // Set the time on the RTC module too
            EEPROM.update(5, _time_day);
            EEPROM.update(6, _time_month);
            EEPROM.put(7, _time_year);

            // Set hour on CAN-BUS Clock
            _canMsgSnd.data[0] = hour();
            _canMsgSnd.data[1] = minute();
            _canMsgSnd.can_id = 0x228;
            _canMsgSnd.can_dlc = 1;
            _can0.sendMessage( & _canMsgSnd);

            if (_serialEnabled) {
            Serial.print("Change Hour/Date: ");
            Serial.print(day());
            Serial.print("/");
            Serial.print(month());
            Serial.print("/");
            Serial.print(year());

            Serial.print(" ");

            Serial.print(hour());
            Serial.print(":");
            Serial.print(minute());

            Serial.println();
            }
        } else if (id == 0x1A9 && len == 8) { // Telematic commands
            _telematicPresent = true;

            _darkMode = bitRead(_canMsgRcv.data[0], 7); // Dark mode
            _resetTrip1 = bitRead(_canMsgRcv.data[0], 1); // Reset Trip 1
            _resetTrip2 = bitRead(_canMsgRcv.data[0], 0); // Reset Trip 2
            _pushAAS = bitRead(_canMsgRcv.data[3], 2); // AAS
            _pushSAM = bitRead(_canMsgRcv.data[3], 2); // SAM
            _pushDSG = bitRead(_canMsgRcv.data[5], 0); // Indirect DSG reset
            _pushSTT = bitRead(_canMsgRcv.data[6], 7); // Start&Stop
            _pushCHECK = bitRead(_canMsgRcv.data[6], 0); // Check
            _stopCHECK = bitRead(_canMsgRcv.data[1], 7); // Stop Check
            _pushBLACK = bitRead(_canMsgRcv.data[5], 0); // Black Panel

            if (_ignition) {
            _canMsgSnd.data[0] = 0x00;
            bitWrite(_canMsgSnd.data[0], 7, _resetTrip1); // Reset Trip 1
            bitWrite(_canMsgSnd.data[0], 6, _resetTrip2); // Reset Trip 2
            _canMsgSnd.data[1] = 0x10;
            bitWrite(_canMsgSnd.data[1], 5, _darkMode); // Dark mode
            _canMsgSnd.data[2] = 0xFF;
            _canMsgSnd.data[3] = 0xFF;
            _canMsgSnd.data[4] = 0x7F;
            _canMsgSnd.data[5] = 0xFF;
            _canMsgSnd.data[6] = 0x00;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x167; // Fake EMF Status frame
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);
            }

            if (!_clusterPresent && _ignition && (_resetTrip1 || _resetTrip2 || _pushAAS || _pushSAM || _pushDSG || _pushSTT || _pushCHECK)) {
            _canMsgSnd.data[0] = _statusCMB[0];
            _canMsgSnd.data[1] = _statusCMB[1];
            bitWrite(_canMsgSnd.data[1], 4, _pushCHECK);
            bitWrite(_canMsgSnd.data[1], 2, _resetTrip1);
            _canMsgSnd.data[2] = _statusCMB[2];
            bitWrite(_canMsgSnd.data[2], 7, _pushAAS);
            bitWrite(_canMsgSnd.data[2], 6, _pushASR);
            _canMsgSnd.data[3] = _statusCMB[3];
            bitWrite(_canMsgSnd.data[3], 3, _pushSAM);
            bitWrite(_canMsgSnd.data[3], 0, _resetTrip2);
            _canMsgSnd.data[4] = _statusCMB[4];
            bitWrite(_canMsgSnd.data[4], 7, _pushDSG);
            _canMsgSnd.data[5] = _statusCMB[5];
            _canMsgSnd.data[6] = _statusCMB[6];
            bitWrite(_canMsgSnd.data[6], 7, _pushSTT);
            _canMsgSnd.data[7] = _statusCMB[7];
            _canMsgSnd.can_id = 0x217;
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);
            }
        } else if (id == 0x329 && len == 8) {
            _pushASR = bitRead(_canMsgRcv.data[3], 0); // ESP
        } else if (id == 0x31C && len == 5) { // MATT status
            _canMsgSnd.data[0] = _canMsgRcv.data[0];
            // Rewrite if necessary to make BTEL commands working
            if (_resetTrip1) { // Reset Trip 1
            bitWrite(_canMsgSnd.data[0], 3, 1);
            }
            if (_resetTrip2) { // Reset Trip 2
            bitWrite(_canMsgSnd.data[0], 2, 1);
            }
            _canMsgSnd.data[1] = _canMsgRcv.data[1];
            _canMsgSnd.data[2] = _canMsgRcv.data[2];
            _canMsgSnd.data[3] = _canMsgRcv.data[3];
            _canMsgSnd.data[4] = _canMsgRcv.data[4];
            _canMsgSnd.can_id = 0x31C;
            _canMsgSnd.can_dlc = 5;
            _can0.sendMessage( & _canMsgSnd);
        } else if (id == 0x217 && len == 8) { // Rewrite Cluster status (CIROCCO for example) for tactile touch buttons (telematic) because it is not listened by BSI
            _clusterPresent = true;

            _canMsgSnd.data[0] = _canMsgRcv.data[0];
            _canMsgSnd.data[1] = _canMsgRcv.data[1];
            bitWrite(_canMsgSnd.data[1], 4, _pushCHECK);
            bitWrite(_canMsgSnd.data[1], 2, _resetTrip1);
            _canMsgSnd.data[2] = _canMsgRcv.data[2];
            bitWrite(_canMsgSnd.data[2], 7, _pushAAS);
            bitWrite(_canMsgSnd.data[2], 6, _pushASR);
            _canMsgSnd.data[3] = _canMsgRcv.data[3];
            bitWrite(_canMsgSnd.data[3], 3, _pushSAM);
            bitWrite(_canMsgSnd.data[3], 0, _resetTrip2);
            _canMsgSnd.data[4] = _canMsgRcv.data[4];
            bitWrite(_canMsgSnd.data[4], 7, _pushDSG);
            _canMsgSnd.data[5] = _canMsgRcv.data[5];
            _canMsgSnd.data[6] = _canMsgRcv.data[6];
            bitWrite(_canMsgSnd.data[6], 7, _pushSTT);
            _canMsgSnd.data[7] = _canMsgRcv.data[7];
            _canMsgSnd.can_id = 0x217;
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);
        } else if (id == 0x15B && len == 8) {
            if (bitRead(_canMsgRcv.data[1], 2)) { // Parameters validity
            tmpVal = _canMsgRcv.data[0];
            if (tmpVal >= 128) {
                _languageAndUnitNum = tmpVal;
                EEPROM.update(0, _languageAndUnitNum);

                if (_serialEnabled) {
                Serial.print("Telematic - Change Language and Unit (Number): ");
                Serial.print(tmpVal);
                Serial.println();
                }

                tmpVal = _canMsgRcv.data[1];
                if (tmpVal >= 128) {
                _mpgMi = true;
                EEPROM.update(4, 1);

                tmpVal = tmpVal - 128;
                } else {
                _mpgMi = false;
                EEPROM.update(4, 0);
                }

                if (tmpVal >= 64) {
                _temperatureInF = true;
                EEPROM.update(3, 1);

                if (_serialEnabled) {
                    Serial.print("Telematic - Change Temperature Type: Fahrenheit");
                    Serial.println();
                }
                } else if (tmpVal >= 0) {
                _temperatureInF = false;
                EEPROM.update(3, 0);

                if (_serialEnabled) {
                    Serial.print("Telematic - Change Temperature Type: Celcius");
                    Serial.println();
                }
                }
            } else {
                tmpVal = ceil(tmpVal / 4.0);
                if (_canMsgRcv.data[1] >= 128) {
                tmpVal--;
                }
                _languageID = tmpVal;

                // CAN2004 Head-up panel is only one-way talking, we can't change the language on it from the CAN2010 Telematic :-(

                if (_serialEnabled) {
                Serial.print("Telematic - Change Language (ID): ");
                Serial.print(tmpVal);
                Serial.println();
                }
            }

            // Personalization settings change
            bitWrite(_canMsgSnd.data[0], 7, 0);
            bitWrite(_canMsgSnd.data[0], 6, 0);
            bitWrite(_canMsgSnd.data[0], 5, 0);
            bitWrite(_canMsgSnd.data[0], 4, 0);
            bitWrite(_canMsgSnd.data[0], 3, 0);
            bitWrite(_canMsgSnd.data[0], 2, 0); // Parameters validity, 0 = Changed parameter(s) the BSI must take into account
            bitWrite(_canMsgSnd.data[0], 1, 0); // User profile
            bitWrite(_canMsgSnd.data[0], 0, 1); // User profile = 1
            bitWrite(_canMsgSnd.data[1], 7, bitRead(_canMsgRcv.data[2], 6)); // Selective openings
            bitWrite(_canMsgSnd.data[1], 6, 1);
            bitWrite(_canMsgSnd.data[1], 5, bitRead(_canMsgRcv.data[2], 4)); // Selective rear openings
            bitWrite(_canMsgSnd.data[1], 4, bitRead(_canMsgRcv.data[2], 5)); // Selective openings
            bitWrite(_canMsgSnd.data[1], 3, 0);
            bitWrite(_canMsgSnd.data[1], 2, 0);
            bitWrite(_canMsgSnd.data[1], 1, bitRead(_canMsgRcv.data[2], 3)); // Driver welcome
            bitWrite(_canMsgSnd.data[1], 0, bitRead(_canMsgRcv.data[2], 7)); // Parking brake
            bitWrite(_canMsgSnd.data[2], 7, bitRead(_canMsgRcv.data[2], 2)); // Adaptative lighting
            bitWrite(_canMsgSnd.data[2], 6, bitRead(_canMsgRcv.data[3], 4)); // Beam
            bitWrite(_canMsgSnd.data[2], 5, bitRead(_canMsgRcv.data[3], 7)); // Guide-me home lighting
            bitWrite(_canMsgSnd.data[2], 4, bitRead(_canMsgRcv.data[3], 0)); // Automatic headlights
            bitWrite(_canMsgSnd.data[2], 3, 0);
            bitWrite(_canMsgSnd.data[2], 2, 0);
            bitWrite(_canMsgSnd.data[2], 1, bitRead(_canMsgRcv.data[3], 6)); // Duration Guide-me home lighting (2b)
            bitWrite(_canMsgSnd.data[2], 0, bitRead(_canMsgRcv.data[3], 5)); // Duration Guide-me home lighting (2b)
            bitWrite(_canMsgSnd.data[3], 7, bitRead(_canMsgRcv.data[2], 0)); // Ambiance lighting
            bitWrite(_canMsgSnd.data[3], 6, bitRead(_canMsgRcv.data[2], 1)); // Daytime running lights
            bitWrite(_canMsgSnd.data[3], 5, 0);
            bitWrite(_canMsgSnd.data[3], 4, 0);
            bitWrite(_canMsgSnd.data[3], 3, 0);
            bitWrite(_canMsgSnd.data[3], 2, 0);
            bitWrite(_canMsgSnd.data[3], 1, 0);
            bitWrite(_canMsgSnd.data[3], 0, 0);
            _canMsgSnd.data[4] = 0x00;
            bitWrite(_canMsgSnd.data[5], 7, bitRead(_canMsgRcv.data[4], 7)); // AAS
            bitWrite(_canMsgSnd.data[5], 6, bitRead(_canMsgRcv.data[4], 7)); // AAS
            bitWrite(_canMsgSnd.data[5], 5, 0);
            bitWrite(_canMsgSnd.data[5], 4, bitRead(_canMsgRcv.data[4], 5)); // Wiper in reverse
            bitWrite(_canMsgSnd.data[5], 3, 0);
            bitWrite(_canMsgSnd.data[5], 2, 0);
            bitWrite(_canMsgSnd.data[5], 1, 0);
            bitWrite(_canMsgSnd.data[5], 0, 0);
            bitWrite(_canMsgSnd.data[6], 7, 0);
            bitWrite(_canMsgSnd.data[6], 6, bitRead(_canMsgRcv.data[4], 6)); // SAM
            bitWrite(_canMsgSnd.data[6], 5, bitRead(_canMsgRcv.data[4], 6)); // SAM
            bitWrite(_canMsgSnd.data[6], 4, 0);
            bitWrite(_canMsgSnd.data[6], 3, 0);
            bitWrite(_canMsgSnd.data[6], 2, 0);
            bitWrite(_canMsgSnd.data[6], 1, 0);
            bitWrite(_canMsgSnd.data[6], 0, 0);
            bitWrite(_canMsgSnd.data[7], 7, bitRead(_canMsgRcv.data[4], 3)); // Configurable button
            bitWrite(_canMsgSnd.data[7], 6, bitRead(_canMsgRcv.data[4], 2)); // Configurable button
            bitWrite(_canMsgSnd.data[7], 5, bitRead(_canMsgRcv.data[4], 1)); // Configurable button
            bitWrite(_canMsgSnd.data[7], 4, bitRead(_canMsgRcv.data[4], 0)); // Configurable button
            bitWrite(_canMsgSnd.data[7], 3, 0);
            bitWrite(_canMsgSnd.data[7], 2, 0);
            bitWrite(_canMsgSnd.data[7], 1, 0);
            bitWrite(_canMsgSnd.data[7], 0, 0);
            _canMsgSnd.can_id = 0x15B;
            _canMsgSnd.can_dlc = 8;
            _can0.sendMessage( & _canMsgSnd);

            // Store personalization settings for the recurring frame
            _personalizationSettings[0] = _canMsgSnd.data[1];
            _personalizationSettings[1] = _canMsgSnd.data[2];
            _personalizationSettings[2] = _canMsgSnd.data[3];
            _personalizationSettings[3] = _canMsgSnd.data[4];
            _personalizationSettings[4] = _canMsgSnd.data[5];
            _personalizationSettings[5] = _canMsgSnd.data[6];
            _personalizationSettings[6] = _canMsgSnd.data[7];
            EEPROM.update(10, _personalizationSettings[0]);
            EEPROM.update(11, _personalizationSettings[1]);
            EEPROM.update(12, _personalizationSettings[2]);
            EEPROM.update(13, _personalizationSettings[3]);
            EEPROM.update(14, _personalizationSettings[4]);
            EEPROM.update(15, _personalizationSettings[5]);
            EEPROM.update(16, _personalizationSettings[6]);
            }
        } else if (id == 0x1E9 && len >= 2 && _CVM_Emul) { // Telematic suggested speed to fake CVM frame
            _can0.sendMessage( & _canMsgRcv);

            tmpVal = (_canMsgRcv.data[3] >> 2); // POI type - Gen2 (6b)

            _canMsgSnd.data[0] = _canMsgRcv.data[1];
            _canMsgSnd.data[1] = ((tmpVal > 0 && _vehicleSpeed > _canMsgRcv.data[0]) ? 0x30 : 0x10); // POI Over-speed, make speed limit blink
            _canMsgSnd.data[2] = 0x00;
            _canMsgSnd.data[3] = 0x00;
            _canMsgSnd.data[4] = 0x7C;
            _canMsgSnd.data[5] = 0xF8;
            _canMsgSnd.data[6] = 0x00;
            _canMsgSnd.data[7] = 0x00;
            _canMsgSnd.can_id = 0x268; // CVM Frame ID
            _canMsgSnd.can_dlc = 8;
            _can1.sendMessage( & _canMsgSnd);
        } else if (id == 0x1E5 && len == 7) {
            // Ambience mapping
            tmpVal = _canMsgRcv.data[5];
            if (tmpVal == 0x00) { // User
            _canMsgRcv.data[6] = 0x40;
            } else if (tmpVal == 0x08) { // Classical
            _canMsgRcv.data[6] = 0x44;
            } else if (tmpVal == 0x10) { // Jazz
            _canMsgRcv.data[6] = 0x48;
            } else if (tmpVal == 0x18) { // Pop-Rock
            _canMsgRcv.data[6] = 0x4C;
            } else if (tmpVal == 0x28) { // Techno
            _canMsgRcv.data[6] = 0x54;
            } else if (tmpVal == 0x20) { // Vocal
            _canMsgRcv.data[6] = 0x50;
            } else { // Default : User
            _canMsgRcv.data[6] = 0x40;
            }

            // Loudness / Volume linked to speed
            tmpVal = _canMsgRcv.data[4];
            if (tmpVal == 0x10) { // Loudness / not linked to speed
            _canMsgRcv.data[5] = 0x40;
            } else if (tmpVal == 0x14) { // Loudness / Volume linked to speed
            _canMsgRcv.data[5] = 0x47;
            } else if (tmpVal == 0x04) { // No Loudness / Volume linked to speed
            _canMsgRcv.data[5] = 0x07;
            } else if (tmpVal == 0x00) { // No Loudness / not linked to speed
            _canMsgRcv.data[5] = 0x00;
            } else { // Default : No Loudness / not linked to speed
            _canMsgRcv.data[5] = 0x00;
            }

            // Bass
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = _canMsgRcv.data[2];
            _canMsgRcv.data[2] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Treble
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = _canMsgRcv.data[3];
            _canMsgRcv.data[4] = ((tmpVal - 32) / 4) + 57; // Converted value on position 4 (while it's on 3 on a old amplifier)

            // Balance - Left / Right
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = _canMsgRcv.data[1];
            _canMsgRcv.data[1] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Balance - Front / Back
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = _canMsgRcv.data[0];
            _canMsgRcv.data[0] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Mediums ?
            _canMsgRcv.data[3] = 63; // 0x3F = 63

            _can0.sendMessage( & _canMsgRcv);
        } else {
            _can0.sendMessage( & _canMsgRcv);
        }
        } else {
        _can0.sendMessage( & _canMsgRcv);
        }

    }
}

void PsaComfortCanAdapter::sendPOPup(bool present, int id, byte priority, byte parameters) {
  bool clear = false;
  byte firstEmptyPos = 8;

  for (int i = 0; i < 8; i++) {
    if (_alertsCache[i] == id) {
      if (!present) { // Remove from cache and clear popup
        _alertsCache[i] = _alertsParametersCache[i] = firstEmptyPos = 0;
        clear = true;
        break;
      } else if (parameters == _alertsParametersCache[i]) { // Already sent
        return;
      } else {
        return sendPOPup(false, id, priority, 0x00); // Clear previous popup first
      }
    } else if (_alertsCache[i] == 0 && firstEmptyPos >= 8) {
      firstEmptyPos = i;
    }
  }

  if (firstEmptyPos >= 8) {
    return; // Avoid overflow
  }
  if (!present && !clear) {
    return;
  } else if (!clear) {
    _alertsCache[firstEmptyPos] = id;
    _alertsParametersCache[firstEmptyPos] = parameters;

    if (_serialEnabled && present) {
      Serial.print("Notification sent with message ID: ");
      Serial.println(id);
    }
  }

  if (priority > 14) {
    priority = 14;
  }

  if (present) {
    _canMsgSnd.data[0] = highByte(id);
    _canMsgSnd.data[1] = lowByte(id);
    bitWrite(_canMsgSnd.data[0], 7, present); // New message
  } else { // Close Popup
    _canMsgSnd.data[0] = 0x7F;
    _canMsgSnd.data[1] = 0xFF;
  }
  _canMsgSnd.data[2] = priority; // Priority (0 > 14)
  bitWrite(_canMsgSnd.data[2], 7, 1); // Destination: NAC / EMF / MATT
  bitWrite(_canMsgSnd.data[2], 6, 1); // Destination: CMB
  _canMsgSnd.data[3] = parameters; // Parameters
  _canMsgSnd.data[4] = 0x00; // Parameters
  _canMsgSnd.data[5] = 0x00; // Parameters
  _canMsgSnd.data[6] = 0x00; // Parameters
  _canMsgSnd.data[7] = 0x00; // Parameters
  _canMsgSnd.can_id = 0x1A1;
  _canMsgSnd.can_dlc = 8;
  _can1.sendMessage( & _canMsgSnd);

  return;
}

int PsaComfortCanAdapter::daysSinceYearStartFct() {
  // Given a day, month, and year (4 digit), returns
  // the day of year. Errors return 999.
  int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  // Check if it is a leap year, this is confusing business
  // See: https://support.microsoft.com/en-us/kb/214019
  if (year()%4  == 0) {
    if (year()%100 != 0) {
      daysInMonth[1] = 29;
    }
    else {
      if (year()%400 == 0) {
        daysInMonth[1] = 29;
      }
    }
   }

  int doy = 0;
  for (int i = 0; i < month() - 1; i++) {
    doy += daysInMonth[i];
  }

  doy += day();
  return doy;
}

void PsaComfortCanAdapter::setDebugGeneral(bool state) { _debugGeneral = state; }
void PsaComfortCanAdapter::setDebugCAN0(bool state) { _debugCAN0 = state; }
void PsaComfortCanAdapter::setDebugCAN1(bool state) { _debugCAN1 = state; }
void PsaComfortCanAdapter::setEconomyModeEnabled(bool state) { _economyModeEnabled = state; }
void PsaComfortCanAdapter::setSend_CAN2010_ForgedMessages(bool state) { _send_CAN2010_ForgedMessages = state; }
void PsaComfortCanAdapter::setTemperatureInF(bool state) { _temperatureInF = state; }
void PsaComfortCanAdapter::setMpgMi(bool state) { _mpgMi = state; }
void PsaComfortCanAdapter::setKmL(bool state) { _kmL = state; }
void PsaComfortCanAdapter::setFixedBrightness(bool state) { _fixedBrightness = state; }
void PsaComfortCanAdapter::setNoFMUX(bool state) { _noFMUX = state; }
void PsaComfortCanAdapter::setSteeringWheelCommands_Type(byte type) { _steeringWheelCommands_Type = type; }
void PsaComfortCanAdapter::setLanguageID(byte languageID) { _languageID = languageID; }
void PsaComfortCanAdapter::setListenCAN2004Language(bool state) { _listenCAN2004Language = state; }
void PsaComfortCanAdapter::setResetEEPROM(bool state) { _resetEEPROM = state; }
void PsaComfortCanAdapter::setCVM_Emul(bool state) { _CVM_Emul = state; }
void PsaComfortCanAdapter::setGeneratePOPups(bool state) { _generatePOPups = state; }
void PsaComfortCanAdapter::setEmulateVIN(bool state) { _emulateVIN = state; }
void PsaComfortCanAdapter::setVinNumber(char vin[18]) { strcpy(_vinNumber, vin); }
void PsaComfortCanAdapter::setHasAnalogicButtons(bool state) { _hasAnalogicButtons = state; }
void PsaComfortCanAdapter::setMenuButton(byte menuButton) { _menuButton = menuButton; }
void PsaComfortCanAdapter::setVolDownButton(byte volDownButton) { _volDownButton = volDownButton; }
void PsaComfortCanAdapter::setVolUpButton(byte volUpButton) { _volUpButton = volUpButton; }
void PsaComfortCanAdapter::setScrollValue(byte scrollValue) { _scrollValue = scrollValue; }
