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

#ifndef WICS_DCCGEN_H
#define WICS_DCCGEN_H

#include <stdint.h>

#define DCC_PKT_MAXLEN      6

typedef struct __attribute__ ((packed)) {
    uint8_t data[DCC_PKT_MAXLEN];   // dane pakietu DCC
    uint8_t bytes;				    // rozmiar pakietu DCC, 0-brak pakietu
    uint8_t repeat;                 // liczba powtórzeń
    uint8_t preamble;
} dccPacket_td;

esp_err_t DCCGEN_MainT_PutPacket(dccPacket_td *pkt);
void      DCCGEN_MainT_Start(uint8_t preamble);
void      DCCGEN_MainT_Stop();
void      DCCGEN_Initialize();

#endif // WICS_DCCGEN_H
