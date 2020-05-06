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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"

#include "wics_config.h"
#include "wics_dccgen.h"

extern xQueueHandle stationQueue;
extern EventGroupHandle_t appEventGroup;

static const char *TAG = "WiCS.DCCg";

DRAM_ATTR const uint64_t dccgPulseOne = (uint64_t)(DCCG_PULSE_ONE);
DRAM_ATTR const uint64_t dccgPulseZero = (uint64_t)(DCCG_PULSE_ZERO);

// tor główny
static uint64_t mtPulse;
static uint8_t  mtBufOut;		// bufor wyjściowy
static uint8_t  mtBitCnt;		// licznik bitów
static uint8_t  mtByteCnt;		// licznik bajtów, 0=preambuła
static uint8_t  mtLastByte;
static uint8_t  mtEmpty;
static dccPacket_td mainT;      // dane pakietu DCC

static portMUX_TYPE mtSpinlock = portMUX_INITIALIZER_UNLOCKED;
static gpio_dev_t *dccgen = GPIO_HAL_GET_HW(GPIO_PORT_0);

static inline void IRAM_ATTR GPIO_isr_LevelSet(gpio_num_t gpio_num)
{
    if (gpio_num < 32)
        dccgen->out_w1ts = (1 << gpio_num);
    else
        dccgen->out1_w1ts.data = (1 << (gpio_num - 32));
}

static inline void IRAM_ATTR GPIO_isr_LevelClr(gpio_num_t gpio_num)
{
    if (gpio_num < 32)
        dccgen->out_w1tc = (1 << gpio_num);
    else
        dccgen->out1_w1tc.data = (1 << (gpio_num - 32));
}

// generator sygnału DCC - tor główny, przerwanie
static void IRAM_ATTR DCCGEN_isrHandler(void *param)
{
    portENTER_CRITICAL_SAFE(&mtSpinlock); // timer_spinlock_take
    volatile uint64_t timCount =
            timer_group_get_counter_value_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);
    timer_group_clr_intr_status_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);

    // tor główny
    mtBitCnt--;
    if (mtBitCnt & 0x01) {
        // środek bitu
        GPIO_isr_LevelClr(GPIO_DCC_SIG1);
    }
    else {
        // początek bitu
        GPIO_isr_LevelSet(GPIO_DCC_SIG1);
        if (mtBitCnt) {
            // następny bit
           	if (mtByteCnt != 0) {
           		// bit danych
            	if (mtBufOut & 0x80)
            		mtPulse = dccgPulseOne;
                else
            		mtPulse = dccgPulseZero;
                mtBufOut = mtBufOut << 1;
            }
            else {
                // preambuła: zawsze 1
                mtPulse = dccgPulseOne;
            }
        }
        else {
           	// wszystkie bity wysłane - następny krok
           	if (mtByteCnt == 0) {
                // koniec preambuły - co dalej?
            	if (mainT.bytes) {
                    // pakiet gotowy do wysłania
            		mtPulse = dccgPulseZero; // bit startu
                    mtBitCnt = 18; // 8b danych + 1b startu
                    mtByteCnt = 1;
                    mtLastByte = mainT.bytes;
                    mtBufOut = mainT.data[0];
                }
            	else {
                    // nadal nic do wysłania, preambuła
                    mtBitCnt = 8;
                    mtEmpty = 1;
                }
            } // preambuła
            else {
                // dane
           		if (mtByteCnt == mtLastByte) {
           			// nie ma więcej danych: koniec pakietu
           			mtPulse = dccgPulseOne;
                    mtByteCnt = 0;
            		mtBitCnt = mainT.preamble + 2; // bit stopu + preambuła
                }
                else {
                    // są dane do wysłania
            		mtPulse = dccgPulseZero;
            		mtBitCnt = 18; // 8b danych + 1b startu
            		mtBufOut = mainT.data[mtByteCnt++];
            		if (mtByteCnt == mtLastByte) {
                        if (--mainT.repeat) {
                            mainT.bytes = 0; // zwolnienie bufora
                            mtEmpty = 1;
                        }
            		}
                }
            } // dane
        }
    } // następny bit

    timCount += mtPulse;
    timer_group_set_alarm_value_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER, timCount);
    timer_group_enable_alarm_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);
    if (mtEmpty != 0) {
        xEventGroupSetBitsFromISR(appEventGroup, DCCG_PACKET_REQUEST, NULL);
        mtEmpty = 0;
    }
    portEXIT_CRITICAL_SAFE(&mtSpinlock); // timer_spinlock_give

} // DCCGEN_isrHandler

// uruchomienie generatora DCC (tor główny)
// preamble: rozmiar preambuły pakietu
// zwrot:    brak
void DCCGEN_MainT_Start(uint8_t preamble)
{
    mainT.preamble = preamble;
	mtBitCnt   = preamble;
	mtByteCnt  = 0;
	mtLastByte = 0;
	mtEmpty = 0;
	GPIO_DCC_Enable();
	GPIO_DCC_Sig1_Hi();

	mtPulse = dccgPulseOne;
	timer_set_counter_value(DCCG_TIMER_GRP, DCCG_MT_TIMER, 0x00000000ULL);
	timer_set_alarm_value(DCCG_TIMER_GRP, DCCG_MT_TIMER, mtPulse);
	timer_start(DCCG_TIMER_GRP, DCCG_MT_TIMER);

} // DCCGEN_MainT_Start

// zatrzymanie generatora DCC (tor główny)
// param: brak
// zwrot: brak
void DCCGEN_MainT_Stop()
{
    timer_pause(DCCG_TIMER_GRP, DCCG_MT_TIMER);

    GPIO_DCC_Disable();
    GPIO_DCC_Sig1_Hi();
    mainT.bytes = 0;

} // DCCGEN_MainT_Stop

// wprowadzenie pakietu do wygenerowania
// pkt:   dane pakietu DCC
// zwrot: status skopiowania pakietu
esp_err_t DCCGEN_MainT_PutPacket(dccPacket_td *pkt)
{
    if (mainT.bytes != 0) {
        return ESP_FAIL;
    }

    if (pkt != NULL) {
        memcpy((void *)&mainT, pkt, sizeof(dccPacket_td));
        ESP_LOGI(TAG, "Pkt: %X %X %X %X %X", pkt->data[0], pkt->data[1],
                 pkt->data[2], pkt->data[3], pkt->data[4]);
    }
    else {
        // brak pakietu > idle
        mainT.data[0] = 0xFF;
        mainT.data[1] = 0x00;
        mainT.data[2] = 0xFF;
        mainT.repeat  = 1;
        mainT.bytes   = 3;
    }
    return ESP_OK;

} // DCCGEN_MainT_PutPacket

// inicjalizacja timera generatora DCC
// param: brak
// zwrot: brak
static void DCCGEN_TimerInit()
{
    timer_config_t config;
    config.divider      = DCCG_TIMER_DIV;
    config.counter_dir  = TIMER_COUNT_UP;
    config.counter_en   = TIMER_PAUSE;
    config.alarm_en     = TIMER_ALARM_EN;
    config.intr_type    = TIMER_INTR_LEVEL;
    config.auto_reload  = 0;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    timer_init(DCCG_TIMER_GRP, DCCG_MT_TIMER, &config);

    timer_set_counter_value(DCCG_TIMER_GRP, DCCG_MT_TIMER, 0x00000000ULL);
    timer_enable_intr(DCCG_TIMER_GRP, DCCG_MT_TIMER);
    timer_isr_register(DCCG_TIMER_GRP, DCCG_MT_TIMER, DCCGEN_isrHandler,
                       (void *)NULL, ESP_INTR_FLAG_IRAM, NULL);

} // DCCGEN_TimerInit

// inicjalizacja generatora DCC
// param: brak
// zwrot: brak
void DCCGEN_Initialize()
{
    ESP_LOGI(TAG, "DCCGEN init: %dB", esp_get_free_heap_size());

    mainT.bytes = 0;
    DCCGEN_TimerInit();

} // DCCGEN_Initialize

// EOF wics_dccgen.c
