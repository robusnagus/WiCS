//
// Project: Wireless Command Station
// File:    wics_dccgen.c
// Author:  Nagus
// Version: 20200303
//

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "hal/gpio_hal.h"

#include "wics_config.h"
#include "wics_dccgen.h"

extern xQueueHandle stationQueue;

static const char *TAG = "WiCS.DCCg";

DRAM_ATTR const uint64_t dccgPulseOne = (uint64_t)(DCCG_PULSE_ONE);
DRAM_ATTR const uint64_t dccgPulseZero = (uint64_t)(DCCG_PULSE_ZERO);

// tor główny
static uint64_t mtPulse;
static uint8_t  mtBufOut;		// bufor wyjściowy
static uint8_t  mtBitCnt;		// bufor-licznik bitów
static uint8_t  mtByteCnt;		// bufor-licznik bajtów, 0=preambuła
static dccPacket_td mainT;      // dane pakietu

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

// przerwanie generatora sygnału DCC - tor główny
static void IRAM_ATTR DCCGEN_isrHandler(void *param)
{
    portENTER_CRITICAL_SAFE(&mtSpinlock); // timer_spinlock_take
    int trackId = 0;
    volatile uint64_t timCount =
            timer_group_get_counter_value_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);
    timer_group_clr_intr_status_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);

    // zmiana stanu sygnału na: tor główny
    if (mtBitCnt & 0x01) {
        // środek bitu
        GPIO_isr_LevelClr(GPIO_DCC_SIG1);
        mtBitCnt--;
    }
    else {
        // koniec bitu
        GPIO_isr_LevelSet(GPIO_DCC_SIG1);
        if (mtBitCnt) {
            // następny bit
           	mtBitCnt--;
           	if (mtByteCnt != 0) {
           		// następny bit danych
            	if (mtBufOut & 0x80)
            		mtPulse = dccgPulseOne;
                else
            		mtPulse = dccgPulseZero;
                mtBufOut = mtBufOut << 1;
            }
            // preambuła: następny bit zawsze 1
        }
        else {
           	// bity wysłane - następny krok
           	if (mtByteCnt) {
           		// pakiet danych
           		if (mtByteCnt == mainT.bytes) {
           			// nie ma więcej danych: koniec pakietu
           			mtPulse = dccgPulseOne;
                    mtByteCnt = 0;
            		mtBitCnt = mainT.preamble + 2; // bit stopu + preambuła
            		if (--mainT.repeat) {
                        mainT.bytes = 0; // zwolnienie bufora
                        xQueueSendFromISR(stationQueue, &trackId, NULL);
                    }
                }
            	else {
                    // są jeszcze dane do wysłania
            		mtPulse = dccgPulseZero;
                    mtBitCnt = 18;
            		mtBufOut = mainT.data[mtByteCnt++];
                }
            } // pakiet danych
            // preambuła
            else {
            	// koniec preambuły - co dalej?
            	if (mainT.bytes) {
                    // jest pakiet do wysłania
            		mtPulse = dccgPulseZero; // bit startu
                    mtBitCnt = 18; // 8b danych + 1b startu
                    mtByteCnt = 1;
                    mtBufOut = mainT.data[0];
                }
            	else {
                    // nic do wysłania, nadal preambuła
                    mtBitCnt = 4;
                    xQueueSendFromISR(stationQueue, &trackId, NULL);
                }
            } // preambuła
        }
    } // następny bit

    timCount += mtPulse;
    timer_group_set_alarm_value_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER, timCount);
    timer_group_enable_alarm_in_isr(DCCG_TIMER_GRP, DCCG_MT_TIMER);
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
    ESP_LOGI(TAG, "DCCGEN starting: %dB", esp_get_free_heap_size());

    mainT.bytes = 0;
    DCCGEN_TimerInit();

} // DCCGEN_Initialize

// EOF wics_dccgen.c
