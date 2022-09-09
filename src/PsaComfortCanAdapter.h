#ifndef PSA_COMFORT_CAN_ADAPTER_H
#define PSA_COMFORT_CAN_ADAPTER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Time.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h> // https://github.com/PaulStoffregen/DS1307RTC
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

enum LANG_IDS {
        FR = 0,
        EN = 1,
        DE = 2,
        ES = 3,
        IT = 4,
        PT = 5,
        NL = 6,
        BR = 9,
        TR = 12,
        RU = 14
    };

class PsaComfortCanAdapter {

    public:

        PsaComfortCanAdapter(uint8_t csPinCan0, uint8_t csPinCan1);

        void adapterInit();
        void transfer();

        //Configuration methods

        // Get some debug informations on Serial
        void setDebugGeneral(bool state);
        // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
        void setDebugCAN0(bool state);
        // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
        void setDebugCAN1(bool state);
        // You can disable economy mode on the Telematic if you want to - Not recommended at all
        void setEconomyModeEnabled(bool state);
        // Send forged CAN2010 messages to the CAR CAN-BUS Network (useful for testing CAN2010 device(s) from already existent connectors)
        void setSend_CAN2010_ForgedMessages(bool state);
        // Default Temperature in Celcius
        void setTemperatureInF(bool state);
        void setMpgMi(bool state);
        // km/L statistics instead of L/100
        void setKmL(bool state);
        // Force Brightness value in case the calibration does not match your brightness value range
        void setFixedBrightness(bool state);
        // If you don't have any useful button on the main panel, turn the SRC button on steering wheel commands into MENU - only works for CAN2010 SMEG / NAC -
        void setNoFMUX(bool state);
        // noFMUX extra setting : 0 = Generic, 1 = C4 I / C5 X7 NAV+MUSIC+APPS+PHONE mapping, 2 = C4 I / C5 X7 MENU mapping, 3 = C4 I / C5 X7 MENU mapping + SRC on wiper command button, 4 = C4 I / C5 X7 MENU mapping + TRIP on wiper command button, 5 = C4 I / C5 X7 MENU mapping + SRC on wiper command button + TRIP on ESC button
        void setSteeringWheelCommands_Type(byte type);
        // Default is FR: 0 - EN: 1 / DE: 2 / ES: 3 / IT: 4 / PT: 5 / NL: 6 / BR: 9 / TR: 12 / RU: 14
        void setLanguageID(byte languageID);
        // Switch language on CAN2010 devices if changed on supported CAN2004 devices, default: no
        void setListenCAN2004Language(bool state);
        // Switch to true to reset all EEPROM values
        void setResetEEPROM(bool state);
        // Send suggested speed from Telematic to fake CVM (Multifunction camera inside the windshield) frame
        void setCVM_Emul(bool state);
        // Generate notifications from alerts journal - useful for C5 (X7)
        void setGeneratePOPups(bool state);
        // Replace network VIN by another (donor car for example)
        void setEmulateVIN(bool state);
        void setVinNumber(char vin[18]);
        // Analog buttons instead of FMUX
        void setHasAnalogicButtons(bool state);
        void setMenuButton(byte menuButton);
        void setVolDownButton(byte volDownButton);
        void setVolUpButton(byte volUpButton);
        void setScrollValue(byte scrollValue);

    private:

        MCP2515 getMCP(uint8_t mcp_cs_pin);
        void resetEEPROM();
        void checkAnalogButtonsPressed();
        void transferCan2004Messages();
        void transferCan2010Messages();
        void sendPOPup(bool present, int id, byte priority, byte parameters);
        int daysSinceYearStartFct();

        struct can_frame _canMsgSnd;
        struct can_frame _canMsgRcv;
        MCP2515 _can0;
        MCP2515 _can1;

        const uint16_t _serialSpeed {115200};
        const CAN_SPEED _canSpeed {CAN_125KBPS}; // Entertainment CAN bus - Low speed
        const CAN_CLOCK _canFreq {MCP_16MHZ}; // Switch to 8MHZ if you have a 8Mhz module

        // My variables
        bool _debugGeneral = false; // Get some debug informations on Serial
        bool _debugCAN0 = false; // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
        bool _debugCAN1 = false; // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
        bool _economyModeEnabled = true; // You can disable economy mode on the Telematic if you want to - Not recommended at all
        bool _send_CAN2010_ForgedMessages = false; // Send forged CAN2010 messages to the CAR CAN-BUS Network (useful for testing CAN2010 device(s) from already existent connectors)
        bool _temperatureInF = false; // Default Temperature in Celcius
        bool _mpgMi = false;
        bool _kmL = false; // km/L statistics instead of L/100
        bool _fixedBrightness = false; // Force Brightness value in case the calibration does not match your brightness value range
        bool _noFMUX = false; // If you don't have any useful button on the main panel, turn the SRC button on steering wheel commands into MENU - only works for CAN2010 SMEG / NAC -
        byte _steeringWheelCommands_Type = 0; // noFMUX extra setting : 0 = Generic, 1 = C4 I / C5 X7 NAV+MUSIC+APPS+PHONE mapping, 2 = C4 I / C5 X7 MENU mapping, 3 = C4 I / C5 X7 MENU mapping + SRC on wiper command button, 4 = C4 I / C5 X7 MENU mapping + TRIP on wiper command button, 5 = C4 I / C5 X7 MENU mapping + SRC on wiper command button + TRIP on ESC button
        byte _languageID = 0; // Default is FR: 0 - EN: 1 / DE: 2 / ES: 3 / IT: 4 / PT: 5 / NL: 6 / BR: 9 / TR: 12 / RU: 14
        bool _listenCAN2004Language = false; // Switch language on CAN2010 devices if changed on supported CAN2004 devices, default: no
        byte _time_day = 1; // Default day if the RTC module is not configured
        byte _time_month = 1; // Default month if the RTC module is not configured
        int _time_year = 2022; // Default year if the RTC module is not configured
        byte _time_hour = 0; // Default hour if the RTC module is not configured
        byte _time_minute = 0; // Default minute if the RTC module is not configured
        bool _resetEEPROM = false; // Switch to true to reset all EEPROM values
        bool _CVM_Emul = true; // Send suggested speed from Telematic to fake CVM (Multifunction camera inside the windshield) frame
        bool _generatePOPups = false; // Generate notifications from alerts journal - useful for C5 (X7)

        bool _emulateVIN = false; // Replace network VIN by another (donor car for example)
        char _vinNumber[18] = "VF3XXXXXXXXXXXXXX";

        bool _hasAnalogicButtons = false; // Analog buttons instead of FMUX
        byte _menuButton = 4;
        byte _volDownButton = 5;
        byte _volUpButton = 6;
        byte _scrollValue = 0;

        // Default variables
        bool _ignition = false;
        bool _serialEnabled = false;
        int16_t _temperature = 0;
        bool _economyMode = false;
        bool _engineRunning = false;
        uint8_t _languageID_CAN2004 = 0;
        bool _airConditioningON = false;
        uint8_t _fanSpeed = 0;
        bool _fanOff = false;
        bool _airRecycle = false;
        bool _deMist = false;
        bool _deFrost = false;
        uint8_t _leftTemp = 0;
        uint8_t _rightTemp = 0;
        bool _mono = false;
        bool _footAerator = false;
        bool _windShieldAerator = false;
        bool _centralAerator = false;
        bool _autoFan = false;
        uint8_t _fanPosition = 0;
        bool _maintenanceDisplayed = false;
        int16_t _buttonState = 0;
        int16_t _lastButtonState = 0;
        uint32_t _lastDebounceTime = 0;
        uint32_t _buttonPushTime = 0;
        uint32_t _buttonSendTime = 0;
        uint32_t _debounceDelay = 100;
        int16_t _daysSinceYearStart = 0;
        uint32_t _customTimeStamp = 0;
        int16_t _vehicleSpeed = 0;
        int16_t _engineRPM = 0;
        bool _darkMode = false;
        bool _resetTrip1 = false;
        bool _resetTrip2 = false;
        bool _pushAAS = false;
        bool _pushSAM = false;
        bool _pushDSG = false;
        bool _pushSTT = false;
        bool _pushCHECK = false;
        bool _stopCHECK = false;
        bool _pushBLACK = false;
        bool _pushASR = false;
        bool _pushTRIP = false;
        uint8_t _personalizationSettings[11];
        uint8_t _statusCMB[9];
        uint8_t _statusTRIP[9];
        bool _telematicPresent = false;
        bool _clusterPresent = false;
        bool _pushA2 = false;
        int16_t _alertsCache[8]; // Max 8
        uint8_t _alertsParametersCache[8]; // Max 8
        bool _isBVMP = false;
        uint8_t _statusOpenings = 0;
        uint8_t _notificationParameters = 0;

        byte _languageAndUnitNum = (_languageID * 4) + 128;

};

#endif