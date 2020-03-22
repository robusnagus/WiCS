//
// Wireless Command Station
//
// Copyright 2020 Robert Nagowski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// See gpl-3.0.md file for details.
//

#ifndef WICS_STATION_H
#define WICS_STATION_H

#include <esp_types.h>

// dane stanu centralki
typedef struct {
    int16_t  mainCurrent;           // prąd toru głównego
    int16_t  progCurrent;           // prąd toru programującego
    int16_t  filteredMainCurrent;
    int16_t  temperature;
    uint16_t supplyVoltage;         // napięcie zasilania
    uint16_t vccVoltage;
    uint8_t  centralState;          // stan centralki z21
    uint8_t  centralStateEx;        // stan centralki z21

} stationStatus_td;

void STATION_TrackVoltage_On();
void STATION_TrackVoltage_Off();
void STATION_EmergencyStop();

#endif // WICS_STATION_H
