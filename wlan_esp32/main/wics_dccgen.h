//
// Project: Wireless Command Station
// File:    wics_dccgen.h
// Author:  Nagus
// Version: 20200215
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
