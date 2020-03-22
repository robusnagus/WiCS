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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "wics_config.h"
#include "wics_dccgen.h"
#include "wics_qloco.h"
#include "wics_station.h"
#include "wics_z21.h"

static const char *TAG = "WiCS.sta";

extern stationConfig_td appConfig;      // konfiguracja centralki
extern esp_adc_cal_characteristics_t *appADCchars;
       stationStatus_td stationStatus;  // stan centralki Z21
extern EventGroupHandle_t appEventGroup;

dccPacket_td  *dccQueue;        // kolejka pakietów DCC do wysłania
uint8_t       qDccHead;
uint8_t       qDccTail;

uint32_t      sensMtrVolt;      // napięcie na torze głównym
uint32_t      sensMtrCurr;      // prąd na torze głównym
int           sensCount;        // licznik pomiarów

xQueueHandle stationQueue;
esp_timer_handle_t stationTimer;

void XBUS_SendStopped();
void STATION_MainT_EStop();

// włączenie napięcia na torach
// param: brak
// zwrot: brak
void STATION_TrackVoltage_On()
{
    if (stationStatus.centralState &
            (CS_TRACK_VOLTAGE_OFF | CS_SHORT_CIRCUIT)) {
        // napięcie na torach było wyłączone
        DCCGEN_MainT_Start(appConfig.dccLongPreamble);
        xEventGroupSetBits(appEventGroup, DCCG_RUNNING_BIT);
        stationStatus.centralState = stationStatus.centralState &
                            ~(CS_TRACK_VOLTAGE_OFF | CS_SHORT_CIRCUIT);
        GPIO_LED_DccS_On();
    }

    if (stationStatus.centralState & CS_EMERGENCY_STOP) {
        stationStatus.centralState =
                            stationStatus.centralState & ~(CS_EMERGENCY_STOP);
        GPIO_LED_DccS_On();
    }

    if (stationStatus.centralState & CS_PROGMODE_ACTIVE) {
        stationStatus.centralState =
                            stationStatus.centralState & ~(CS_PROGMODE_ACTIVE);
        GPIO_LED_Prog_Off();
    }

} // STATION_TrackVoltage_On

// wyłączenie napięcia na torach
// param: brak
// zwrot: brak
void STATION_TrackVoltage_Off()
{
    stationStatus.centralState |= CS_TRACK_VOLTAGE_OFF;

    xEventGroupClearBits(appEventGroup, DCCG_RUNNING_BIT);
    DCCGEN_MainT_Stop();
    GPIO_LED_DccS_Off();

} // STATION_TrackVoltage_Off

// wysłanie pakietu - awaryjne zatrzymanie
// param: brak
// zwrot: brak
void STATION_EmergencyStop()
{
    while (dccQueue[qDccHead].bytes) {
        dccQueue[qDccHead].bytes = 0;
        qDccHead++;
        if (qDccHead == appConfig.dccMaxQueue) {
            qDccHead = 0;
        }
    }
    qDccHead = 0;
    qDccTail = 0;

    STATION_MainT_EStop();

} // STATION_EmergencyStop

// dodanie pakietu DCC do kolejki
// packet: dane pakietu
// zwrot:  brak
void STATION_AddPacket(dccPacket_td *packet)
{
     if (dccQueue == NULL) {
        // brak pamięci
        return;
    }
    if (dccQueue[qDccTail].bytes != 0) {
        // pełna kolejka
        return;
    }

    memcpy(&(dccQueue[qDccTail]), packet, sizeof(dccPacket_td));
    uint8_t inext = qDccTail + 1;
    if (inext == appConfig.dccMaxQueue)
        inext = 0;
    if (inext != qDccHead)
        qDccTail = inext;

} // STATION_AddPacket

// wygenerowanie pakietu jazdy (DCC)
// pLoco:   wskazanie do rekordu lokomotywy
// pkttype: rodzaj generowanego pakietu
// zwrot:   brak
void STATION_CreateOperPacket(locoQueue_td *pLoco, uint8_t pkttype)
{
    dccPacket_td thePacket;
    uint8_t xsum;
    uint8_t cnt = 0;

    //ESP_LOGI(TAG, "CreateOper %X %.4X", pkttype, pLoco->address);
    thePacket.repeat   = 1;
    thePacket.preamble = appConfig.dccPreamble;
    //adres
    if ((pLoco->address & 0x00FF) == 0) {
        // krótki adres
        xsum = (uint8_t)((pLoco->address >> 8) & 0xFF);
        thePacket.data[cnt++] = xsum;
    }
    else {
        // długi adres
        xsum = (uint8_t)((pLoco->address & 0xFF) | 0xC0);
        thePacket.data[cnt++] = xsum;
        thePacket.data[cnt] = (uint8_t)((pLoco->address >> 8) & 0xFF);
        xsum = xsum ^ thePacket.data[cnt++];
    }

    // jazda / funkcje
    switch (pkttype) {
    case QLOCO_SEND_FUN1:
        // grupa funkcji 1
        thePacket.data[cnt] = (uint8_t)(0x80 | ((pLoco->funct >> 1) & 0x0F)
                                             | ((pLoco->funct << 4) & 0x10));
        break;
    case QLOCO_SEND_FUN2:
        // grupa funkcji 2
        thePacket.data[cnt] = (uint8_t)(0xB0 | ((pLoco->funct >> 5) & 0x0F));
        break;
    case QLOCO_SEND_FUN2A:
        // grupa funkcji 2a
        thePacket.data[cnt] = (uint8_t)(0xA0 | ((pLoco->funct >> 9) & 0x0F));
        break;
    case QLOCO_SEND_FUN3:
        // grupa funkcji 3
        thePacket.data[cnt++] = 0xDE;
        xsum = xsum ^ 0xDE;
        thePacket.data[cnt] = (uint8_t)((pLoco->funct >> 13) & 0xFF);
        thePacket.repeat = 3;
        break;
    case QLOCO_SEND_FUN4:
        // grupa funkcji 4
        thePacket.data[cnt++] = 0xDF;
        xsum = xsum ^ 0xDF;
        thePacket.data[cnt] = (uint8_t)((pLoco->funct >> 21) & 0xFF);
        thePacket.repeat = 3;
        break;
    default:
        // jazda
        if ((pLoco->flags & 0x03) == 0x03) {
            // 128 kroków
            thePacket.data[cnt++] = 0x3F;
            xsum = xsum ^ 0x3F;
            thePacket.data[cnt] = pLoco->speed;
        }
        else {
            // 14/28 kroków
            if (pLoco->speed & 0x80) {
                // do przodu
                thePacket.data[cnt] = 0x60 | (pLoco->speed & 0x1F);
            }
            else {
                // do tyłu
                thePacket.data[cnt] = 0x40 | (pLoco->speed & 0x1F);
            }
        }
        break;
    } // switch pkttype

    xsum = xsum ^ thePacket.data[cnt++];
    thePacket.data[cnt++] = xsum;
    thePacket.bytes = cnt;

    STATION_AddPacket(&thePacket);

} // STATION_CreateOperPacket

// wygenerowanie pakietu jazdy: odświeżenie (DCC)
// pLoco:   wskazanie do rekordu lokomotywy
// zwrot:   brak
void STATION_CreateOperRefresh(locoQueue_td *pLoco)
{
    if (dccQueue == NULL) {
        // brak pamięci
        return;
    }
    if (pLoco == NULL) {
        // nie znaleziony rekord
        return;
    }

    // wygenerowanie pakietów jazdy (odświeżenie)
    STATION_CreateOperPacket(pLoco, QLOCO_SEND_DRIVE);
    if (pLoco->fmask & QLOCO_SEND_FUN1) {
        STATION_CreateOperPacket(pLoco, QLOCO_SEND_FUN1);
    }
    if (pLoco->fmask & QLOCO_SEND_FUN2) {
        STATION_CreateOperPacket(pLoco, QLOCO_SEND_FUN2);
    }
    if (pLoco->fmask & QLOCO_SEND_FUN2A) {
        STATION_CreateOperPacket(pLoco, QLOCO_SEND_FUN2A);
    }

} // DCCGEN_CreateOperRefresh

// elementy czasowe
static void STATION_TimerHandler(void *arg)
{
    // odświerzenie kolejki
    static int locoRefresh = 0;
    locoRefresh++;
    if (locoRefresh == 10) {
        QLOCO_QueueRefresh();
        locoRefresh = 0;
    }

    // pomiary napięć
    if (locoRefresh & 0x01) {
        sensMtrCurr += adc1_get_raw(DCCG_ADC_ISENS);
        sensMtrVolt += adc1_get_raw(DCCG_ADC_VSENS);
        sensCount++;
        if (sensCount == DCCG_ADC_SAMPLES) {
            stationStatus.mainCurrent = (int16_t)(esp_adc_cal_raw_to_voltage(
                                        sensMtrCurr / DCCG_ADC_SAMPLES, appADCchars)
                                        * DCCG_ADC_ISENSDIV);
            stationStatus.supplyVoltage = (uint16_t)(esp_adc_cal_raw_to_voltage(
                                        sensMtrVolt / DCCG_ADC_SAMPLES, appADCchars)
                                        * DCCG_ADC_VSENSDIV);
            sensMtrCurr = 0;
            sensMtrVolt = 0;
            sensCount = 0;
        }
    }

    // dioda PROG
    static int blinkProg = 0;
    if (stationStatus.centralState & CS_SHORT_CIRCUIT) {
        blinkProg++;
        if (blinkProg == 1)
            GPIO_LED_Prog_On();
        else if (blinkProg == 10)
            GPIO_LED_Prog_Off();
        else if (blinkProg > 20)
            blinkProg = 0;
    }

    // dioda DCC
    static int blinkDccs = 0;
    if (stationStatus.centralState & (CS_EMERGENCY_STOP | CS_SHORT_CIRCUIT)) {
        blinkDccs++;
        if (blinkDccs == 1)
            GPIO_LED_DccS_On();
        else if (blinkDccs == 10)
            GPIO_LED_DccS_Off();
        else if (blinkDccs > 20)
            blinkDccs = 0;
    }

} // STATION_TimerHandler

// obsługa naciśnięcia klawisza
static void STATION_EventHandler(void* arg)
{
    switch (*((uint32_t*)arg)) {
    case GPIO_BTN_WIFI:
        // klawisz WiFi

        break;
    case GPIO_BTN_STOP:
        // klawisz STOP
        if (stationStatus.centralState & (CS_TRACK_VOLTAGE_OFF | CS_SHORT_CIRCUIT |
                                          CS_EMERGENCY_STOP | CS_PROGMODE_ACTIVE)) {
            STATION_TrackVoltage_On();
        }
        else {
            stationStatus.centralState |= CS_EMERGENCY_STOP;
            STATION_EmergencyStop();
            XBUS_SendStopped();
        }
        break;
    case GPIO_DCC_ERR:
        // alarm mostka

        break;
    default:
        ESP_LOGI(TAG, "Unknown event: %X", *((uint32_t*)arg));
        break;
    } // switch arg

} // STATION_EventHandler

// wątek obsługi generatora DCC
void STATION_DCCgenerator(void *arg)
{
    while (true) {
        int evt;
        xQueueReceive(stationQueue, &evt, portMAX_DELAY);

        if (stationStatus.centralState & CS_EMERGENCY_STOP) {
            STATION_MainT_EStop();
            continue;
        }

        if (dccQueue[qDccHead].bytes == 0) {
            // kolejka pusta - wygenerowanie odświeżania
            STATION_CreateOperRefresh(QLOCO_FindNext());
        }
        if (dccQueue[qDccHead].bytes == 0) {
            // kolejka nadal pusta
            DCCGEN_MainT_PutPacket(NULL);
        }
        else {
            // wysłanie pakietu
            if (ESP_OK ==
                    DCCGEN_MainT_PutPacket(&(dccQueue[qDccHead]))) {
                // pakiet przyjęty - zdjęcie z kolejki
                dccQueue[qDccHead].bytes = 0;
                qDccHead++;
                if (qDccHead == appConfig.dccMaxQueue) {
                    qDccHead = 0;
                }
            }
        }
    } // pętla główna

    vTaskDelete(NULL);

} // STATION_DCCgenerator

// Emergency STOP na torze głównym
// param: brak
// zwrot: brak
void STATION_MainT_EStop()
{
    dccPacket_td dccPacket;
    dccPacket.data[0]  = 0;
    dccPacket.data[1]  = 0x41;
    dccPacket.data[2]  = 0x41;
    dccPacket.repeat   = 1;
    dccPacket.preamble = appConfig.dccPreamble;
    dccPacket.bytes    = 3;

    DCCGEN_MainT_PutPacket(&dccPacket);

} // STATION_MainT_EStop

// reset na torze głównym
// param: brak
// zwrot: brak
void STATION_MainT_Reset()
{
    qDccHead = 0;
    qDccTail = 0;
    bzero((void*)dccQueue, sizeof(dccPacket_td) * appConfig.dccMaxQueue);

    dccQueue[0].repeat   = appConfig.dccInitSequence;
    dccQueue[0].preamble = appConfig.dccLongPreamble;
    dccQueue[0].bytes    = 3;
    if (ESP_OK == DCCGEN_MainT_PutPacket(&(dccQueue[0]))) {
        dccQueue[0].bytes = 0;
    }
    else {
        qDccTail = 1;
    }

} // STATION_MainT_Reset

static gpio_num_t gpioBtnWiFi = GPIO_BTN_WIFI;
static gpio_num_t gpioBtnStop = GPIO_BTN_STOP;
static gpio_num_t gpioDccErr  = GPIO_DCC_ERR;

// inicjalizacja centralki sterującej DCC
// param: brak
// zwrot: brak
void STATION_Initialize()
{
    ESP_LOGI(TAG, "STATION starting: %dB", esp_get_free_heap_size());

    bzero(&stationStatus, sizeof(stationStatus));
    stationStatus.centralState = CS_TRACK_VOLTAGE_OFF;
    sensMtrVolt = 0;
    sensMtrCurr = 0;
    sensCount   = 0;
    stationQueue = xQueueCreate(16, sizeof(int));

    // timer pomocniczy
    const esp_timer_create_args_t stationTimerArgs = {
        .callback = &STATION_TimerHandler,
        .arg = NULL,
        .name = "statimer"
    };
    if (ESP_OK != esp_timer_create(&stationTimerArgs, &stationTimer)) {
        return;
    }
    esp_timer_start_periodic(stationTimer, 50000);

    dccQueue = (dccPacket_td*)malloc(sizeof(dccPacket_td)
                                        * appConfig.dccMaxQueue);
    if (dccQueue == NULL) {
        ESP_LOGE(TAG, "DCC queue allocation error, free: %dB",
                 esp_get_free_heap_size());
    }
    else {
        gpio_install_isr_service(0);
        gpio_isr_handler_add(GPIO_BTN_WIFI, STATION_EventHandler,
                             (void*)&gpioBtnWiFi);
        gpio_isr_handler_add(GPIO_BTN_STOP, STATION_EventHandler,
                             (void*)&gpioBtnStop);
        gpio_isr_handler_add(GPIO_DCC_ERR, STATION_EventHandler,
                             (void*)&gpioDccErr);

        STATION_MainT_Reset();
        xTaskCreate(STATION_DCCgenerator, "station_task", 2048, NULL, 5, NULL);
        QLOCO_Initialize();
        DCCGEN_Initialize();
        STATION_TrackVoltage_On();
    }

} // STATION_Initialize

// EOF wics_station.c
