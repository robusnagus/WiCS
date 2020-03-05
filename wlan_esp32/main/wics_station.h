//
// Project: Wireless Command Station
// File:    wics_station.h
// Author:  Nagus
// Version: 20200218
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
    uint16_t supplyVoltage;
    uint16_t vccVoltage;
    uint8_t  centralState;          // stan centralki z21
    uint8_t  centralStateEx;        // stan centralki z21

} stationStatus_td;

void STATION_TrackVoltage_On();
void STATION_TrackVoltage_Off();
void STATION_EmergencyStop();

#endif // WICS_STATION_H
