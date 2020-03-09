//
// Wireless Command Station
//
// Copyright 2020 Robert Nagowski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//

#ifndef WICS_Z21_H
#define WICS_Z21_H

// flagi stanu centralki Z21
#define CS_EMERGENCY_STOP           0x01
#define CS_TRACK_VOLTAGE_OFF        0x02
#define CS_SHORT_CIRCUIT            0x04
#define CS_PROGMODE_ACTIVE          0x20

#define CSE_HIGH_TEMPERATURE        0x01
#define CSE_POWER_LOST              0x02
#define CSE_SHORT_CIRCUIT_EXT       0x04
#define CSE_SHORT_CIRCUIT_INT       0x08

// struktura pakietu ZLAN
#define i_Z21_DATALEN               0
#define i_Z21_HEADER                2
#define i_Z21_DATA0                 4
#define i_X21_HEADER                4
#define i_X21_DATA0                 5
#define i_XBUS_HEADER       	    0
#define i_XBUS_DATA			        1

// numery wersji
// biała z21, x-bus 3.0
#define Z21_HW_VERSION              0x00000203
#define Z21_FW_VERSION              0x00000124
#define XBUS_VERSION_MAJOR          0x01
#define XBUS_VERSION_MINOR          0x24
#define XBUS_PROTOCOL               0x30
#define XBUS_STATIONTYPE            0x12

// flagi Broadcast
#define Z21_BC_NONE				    0x00000000
#define Z21_BC_OPER                 0x00000001
#define Z21_BC_SYSSTATE             0x00000100
#define Z21_BC_LOCOINFO             0x00010000

// obsługiwane protokoły
#define LAN_XBUS_MESSAGE            0x40

// funkcje podstawowe
#define LAN_GET_SERIAL_NUMBER       0x10
#define LAN_SERIAL_NUMBER           0x10
#define LAN_GET_HWINFO              0x1A
#define LAN_HWINFO                  0x1A
#define LAN_GET_CODE				0x18
#define LAN_Z21CODE					0x18
	#define Z21_CODE_NOLOCK			0

#define LAN_SET_BROADCASTFLAGS      0x50
#define LAN_GET_BROADCASTFLAGS      0x51
#define LAN_BROADCASTFLAGS          0x51

#define LAN_GET_LOCOMODE            0x60
#define LAN_LOCOMODE                0x60
    #define LOCOMODE_DCC            0

#define LAN_SYSTEMSTATE_GETDATA     0x85
#define LAN_SYSTEMSTATE_DATACHANGED 0x84

// obsługa X-Bus
#define LAN_X_STATION_CONFIG        0x21
    #define X_GET_VERSION           0x21
    #define X_GET_STATUS			0x24
    #define X_SET_TRACK_POWER_OFF   0x80
    #define X_SET_TRACK_POWER_ON    0x81
#define LAN_X_STATION_STATUS        0x61
    #define X_BC_TRACK_POWER_OFF    0x00
    #define X_BC_TRACK_POWER_ON     0x01
    #define X_BC_PROGRAMMING_MODE   0x02
    #define X_BC_SHORT_CIRCUIT      0x08
    #define X_CV_NACK_SC            0x12
    #define X_CV_NACK               0x13
    #define X_UNKNOWN_COMMANMD      0x82
#define LAN_X_STATUS_CHANGED        0x62
#define LAN_X_STATUS                0x63
    #define X_VERSION               0x21
    #define X_CENTRAL_STATE         0x22
    #define X_STATUS                0x24

#define LAN_X_DCC_READ_REGISTER     0x22
#define LAN_X_DCC_WRITE_REGISTER    0x23
    #define X_READ_REG              0x11
    #define X_WRITE_REG             0x12
#define LAN_X_CV_RESULT             0x64
    #define X_CV_RESULT             0x14

#define LAN_X_SET_LOCO_OPER         0xE4
    #define X_LOCO_DRIVE14          0x10
    #define X_LOCO_DRIVE28          0x12
    #define X_LOCO_DRIVE128         0x13
    #define X_LOCO_FUNCTION         0xF8
    #define X_LOCO_INFO             0xF0
#define LAN_X_GET_LOCO_INFO         0xE3
#define LAN_X_LOCO_INFO             0xEF

#define LAN_X_SET_STOP              0x80
#define LAN_X_BC_STOPPED            0x81

#define LAN_X_CV_POM                0xE6
    #define X_DIGIDEC               0x30
    #define X_ACCESSORY             0x31

#define LAN_X_GET_TURNOUT_INFO      0x43
#define LAN_X_SET_TURNOUT           0x53
    #define X_TURNOUT_INFO          0x43

#define LAN_X_GET_XBUSCONFIG        0xF1
#define LAN_X_XBUSCONFIG            0xF3
    #define X_FIRMWARE_VERSION      0x0A

#endif // WICS_Z21_H
