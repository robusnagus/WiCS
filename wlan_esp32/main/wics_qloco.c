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

#include <string.h>

#include "esp_system.h"
#include "esp_log.h"

#include "wics_config.h"
#include "wics_qloco.h"

static const char *TAG = "wiCS.locq";

extern stationConfig_td appConfig;

// kolejka lokomotyw
locoQueue_td *locoQueue;    // kolejka lokomotyw
uint8_t      locoCurrent;   // bieżący rekord

// znajdź pojazd (lokomotywę) o podanym adresie
// addr:  adres lokomotywy
// zwrot: index w kolejce, max=nie znaleziono
uint8_t QLOCO_FindLoco(uint16_t addr)
{
    uint8_t i;
    uint8_t imax = appConfig.locoMaxQueue;

    for (i = 0; i < imax; i++) {
        if (locoQueue[i].timeout > 0)
            if (locoQueue[i].address == addr)
                return i;
    }

    // nie znaleziono
    return imax;

} // QLOCO_FindLoco

// znajdź pierwszy wolny rekord kolejki lokomotyw
// param: brak
// zwrot: index w kolejce, max=nie znaleziono
uint8_t QLOCO_FindFree()
{
    uint8_t idx;
    uint8_t imax = appConfig.locoMaxQueue;

    for (idx = 0; idx < imax; idx++) {
        if (locoQueue[idx].timeout == 0)
            return idx;
    }

    // nie znaleziono
    return imax;

} // QLOCO_FindFree

// znajdź następną pracującą lokomotywę
// param: brak
// zwrot: wskazanie rekordu, NULL=nie znaleziono
locoQueue_td* QLOCO_FindNext()
{
    if (locoQueue == NULL)
        return NULL;

    uint8_t iloco = locoCurrent;
    do {
        locoCurrent++;
        if (locoCurrent == appConfig.locoMaxQueue)
            locoCurrent = 0;
    } while ((locoQueue[locoCurrent].timeout == 0) && (iloco != locoCurrent));

    if (locoQueue[locoCurrent].timeout == 0) {
        // nie ma aktywnych lokomotyw
        return NULL;
    }

    return &(locoQueue[locoCurrent]);

} // QLOCO_FindNext

// informacje o danej lokomotywie
// datagram: nowe dane w formacie XBus
// response: odpowiedź w formacie XBus
// zwrot:    długość odpowiedzi
uint8_t QLOCO_GetInfo(uint8_t *datagram, uint8_t *response)
{
    if (locoQueue == NULL) {
        // brak pamięci
        return 0;
    }

    uint8_t iloco = QLOCO_FindLoco(*((uint16_t*)(&datagram[1])));
    response[0] = datagram[1]; // Lok adres
    response[1] = datagram[2];

    //ESP_LOGI(TAG, "Loco info: %X, addr %X", iloco, *((uint16_t*)(&datagram[1])));
    if (iloco == appConfig.locoMaxQueue) {
        // nie znaleziono - pusty rekord
        response[2] = 0x02;
        response[3] = 0x80;
        response[4] = 0;
        response[5] = 0;
        response[6] = 0;
        response[7] = 0;
    }
    else {
        response[2] = locoQueue[iloco].flags;
        response[3] = locoQueue[iloco].speed;
        response[4] = (uint8_t)(((locoQueue[iloco].funct >> 1) & 0x0F) |
                                ((locoQueue[iloco].funct << 4) & 0x10));
        response[5] = (uint8_t)((locoQueue[iloco].funct >> 5) & 0xFF);
        response[6] = (uint8_t)((locoQueue[iloco].funct >> 13) & 0xFF);
        response[7] = (uint8_t)((locoQueue[iloco].funct >> 21) & 0xFF);
    }

    return 8;

} // QLOCO_GetInfo

extern void STATION_CreateOperPacket(locoQueue_td *pLoco, uint8_t pkttype);

// aktualizacja stanu danej lokomotywy
// datagram: nowe dane w formacie XBus
// response: odpowiedź w formacie XBus
// zwrot:    długość odpowiedzi
uint8_t QLOCO_UpdateOper(uint8_t *datagram, uint8_t *response)
{
    if (locoQueue == NULL) {
        // brak pamięci
        return 0;
    }

    // sprawdzenie kolejki
    uint8_t iloco = QLOCO_FindLoco(*((uint16_t*)&(datagram[1])));
    if (iloco == appConfig.locoMaxQueue) {
        // nie znaleziono - dodanie nowego
        iloco = QLOCO_FindFree();
        if (iloco == appConfig.locoMaxQueue) {
            // pełna kolejka
            return 0;
        }
        locoQueue[iloco].address = *((uint16_t*)&(datagram[1]));
    }

    //ESP_LOGI(TAG, "Update oper %X, adr %X", datagram[0], *((uint16_t*)&(datagram[1])));
    // aktualizacja danych
    uint8_t fOper = 0;
    if ((datagram[0] & 0xF0) == 0x10) {
        // aktualizacja jazdy
        locoQueue[iloco].flags = datagram[0] & 0x0F;
        locoQueue[iloco].speed = datagram[3];
        fOper = QLOCO_SEND_DRIVE;
    }
    else if ((datagram[0] & 0xF0) == 0xF0) {
        // aktualizacja funkcji
        // przygotowanie
        uint8_t  iFun = datagram[3] & 0x3F;
        uint32_t iMask = (1 << iFun);
        if (iFun < 5) {
            fOper = QLOCO_SEND_FUN1;
        }
        else if (iFun < 13) {
            if (iFun < 9)
                fOper = QLOCO_SEND_FUN2;
            else
                fOper = QLOCO_SEND_FUN2A;
        }
        else if (iFun < 21) {
            fOper = QLOCO_SEND_FUN3;
        }
        else {
             fOper = QLOCO_SEND_FUN4;
        }
        locoQueue[iloco].fmask |= fOper;
        // modyfikacja
        if ((datagram[3] & 0x80) == 0x80) {
            // przełączenie
            locoQueue[iloco].funct = locoQueue[iloco].funct ^ iMask;
        }
        else if ((datagram[3] & 0x40) == 0x40) {
            // włączenie
            locoQueue[iloco].funct = locoQueue[iloco].funct | iMask;
        }
        else {
            // wyłączenie
            locoQueue[iloco].funct = locoQueue[iloco].funct & ~(iMask);
        }
    } // aktualizacja funkcji

    locoQueue[iloco].timeout = appConfig.locoTimeout;
    STATION_CreateOperPacket(&(locoQueue[iloco]), fOper);

    // odpowiedź
    response[0] = datagram[1];
    response[1] = datagram[2];
    response[2] = locoQueue[iloco].flags;
    response[3] = locoQueue[iloco].speed;
    response[4] = (uint8_t)(((locoQueue[iloco].funct >> 1) & 0x0F) |
                            ((locoQueue[iloco].funct << 4) & 0x10));
    response[5] = (uint8_t)((locoQueue[iloco].funct >> 5) & 0xFF);
    response[6] = (uint8_t)((locoQueue[iloco].funct >> 13) & 0xFF);
    response[7] = (uint8_t)((locoQueue[iloco].funct >> 21) & 0xFF);
    return 8;

} // QLOCO_UpdateOper

// sprawdzenie aktualności rekordów kolejki lokomotyw
// uruchamiany co 0,5 sekundy
void QLOCO_QueueRefresh()
{
    if (locoQueue == NULL) {
        // brak pamięci
        return;
    }

    uint8_t idx;
    uint8_t imax = appConfig.locoMaxQueue;
    for (idx = 0; idx < imax; idx++) {
        if (locoQueue[idx].timeout != 0) {
            locoQueue[idx].timeout--;
        }
    }

} // QLOCO_QueueRefresh

// inicjalizacja kolejki lokomotyw
// param: brak
// zwrot: brak
void QLOCO_Initialize()
{
    locoCurrent = 0;
    locoQueue = (locoQueue_td *)malloc(sizeof(locoQueue_td)
                                       * appConfig.locoMaxQueue);
    if (locoQueue != NULL) {
        bzero((void*)locoQueue, sizeof(locoQueue_td) * appConfig.locoMaxQueue);
    }
    else {
        ESP_LOGE(TAG, "Lok queue allocation error, free: %dB",
                 esp_get_free_heap_size());
        return;
    }

} // QLOCO_Initialize

// EOF wics_qloco.c
