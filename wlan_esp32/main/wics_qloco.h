//
// Project: Wireless Command Station
// File:    wics_qloco.h
// Author:  Nagus
// Version: 20200217
//

#ifndef WICS_QLOCO_H
#define WICS_QLOCO_H

#define QLOCO_SEND_FUN1     0x01
#define QLOCO_SEND_FUN2     0x02
#define QLOCO_SEND_FUN2A    0x04
#define QLOCO_SEND_FUN3     0x08
#define QLOCO_SEND_FUN4     0x10
#define QLOCO_SEND_DRIVE    0x80

// rekord kolejki lokowotyw
typedef struct {
    uint16_t address;   // adres
    uint8_t  flags;     // flagi
                        // b0:1 - liczba kroków 14/28/128
                        // b2:3 - zarezerwowane
    uint8_t  speed;     // prędkość
    uint32_t funct;     // funkcje
    uint8_t  fmask;     // maska wysyłania
    uint8_t  timeout;   // czas wygaśnięcia
} locoQueue_td;

void          QLOCO_Initialize();
locoQueue_td* QLOCO_FindNext();
uint8_t       QLOCO_GetInfo(uint8_t *datagram, uint8_t *response);
uint8_t       QLOCO_UpdateOper(uint8_t *datagram, uint8_t *response);
void          QLOCO_QueueRefresh();

#endif // WICS_QLOCO_H
