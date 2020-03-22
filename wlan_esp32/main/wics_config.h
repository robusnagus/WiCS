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

#ifndef WICS_CONFIG_H
#define WICS_CONFIG_H

#define SW_VERSION_TITLE        "20200322.3"
#define HW_VERSION_MAJOR        1
#define HW_VERSION_MINOR        0

#define GPIO_LED_WIFI           2
#define GPIO_LED_DCCR           22
#define GPIO_LED_PROG           12
#define GPIO_BTN_WIFI           0
#define GPIO_BTN_STOP           4

#define GPIO_DCC_SIG1           5
#define GPIO_DCC_BRK            25
#define GPIO_DCC_ENA            19
#define GPIO_DCC_ISEN           33
#define GPIO_DCC_VSEN           34
#define GPIO_DCC_ERR            15

#define GPIO_RCOM_ENA           26
#define GPIO_RCOM_SHORT         27
#define GPIO_RCOM_DIR           39
#define GPIO_RCOM_DATA          36

#define GPIO_LED_Wifi_On()      gpio_set_level(GPIO_LED_WIFI, 1)
#define GPIO_LED_Wifi_Off()     gpio_set_level(GPIO_LED_WIFI, 0)
#define GPIO_LED_DccS_On()      gpio_set_level(GPIO_LED_DCCR, 1)
#define GPIO_LED_DccS_Off()     gpio_set_level(GPIO_LED_DCCR, 0)
#define GPIO_LED_Prog_On()      gpio_set_level(GPIO_LED_PROG, 1)
#define GPIO_LED_Prog_Off()     gpio_set_level(GPIO_LED_PROG, 0)
#define GPIO_DCC_Sig1_Hi()      gpio_set_level(GPIO_DCC_SIG1, 1)
#define GPIO_DCC_Sig1_Lo()      gpio_set_level(GPIO_DCC_SIG1, 0)
#define GPIO_DCC_Enable()       gpio_set_level(GPIO_DCC_ENA, 1)
#define GPIO_DCC_Disable()      gpio_set_level(GPIO_DCC_ENA, 0)

#define Z21NET_HOSTNAME         "NGS_WiCS"
#define Z21NET_PORT_NUM         21105
#define Z21NET_RETRY_TOUT       (1000 / portTICK_PERIOD_MS)
#define Z21NET_MAX_BUFFER	    256
#define Z21NET_MAX_USER         16

#define WIFI_CONNECTED_BIT      BIT0
#define DCCG_RUNNING_BIT        BIT4

#define WIFI_MAX_POSTBUFSIZE    1024
#define WIFI_MAX_STARETRY       5
#define WIFI_PORT_WEBUI         80

#define DCCG_TIMER_GRP          TIMER_GROUP_1
#define DCCG_MT_TIMER           TIMER_0
#define DCCG_TIMER_DIV          8
#define DCCG_PULSE_ONE          570
#define DCCG_PULSE_ZERO         1140

#define DCCG_ADC_SAMPLES        10
#define DCCG_ADC_DEFVREF        1100
#define DCCG_ADC_ATTEN          ADC_ATTEN_DB_0
#define DCCG_ADC_VSENS          ADC_CHANNEL_5
#define DCCG_ADC_ISENS          ADC_CHANNEL_6
#define DCCG_ADC_VSENSDIV       23
#define DCCG_ADC_ISENSDIV       5

// konfiguracja urządzenia
#define MAX_WLAN_NAME           32
#define MAX_WLAN_PASS           32

#define CONFIG_NAMESPACE        "MemAppCfg"
#define CFG_NAME_APPCONFIG      "AppConfig"

#define CFGDEF_AP_SSID          "NagusZ21"
#define CFGDEF_AP_PASS          "pazzword"
#define CFGDEF_AP_CHANNEL       5
#define CFGDEF_AP_IPADDR        "192.168.0.111"
#define CFGDEF_AP_IPGW          "192.168.0.111"
#define CFGDEF_AP_IPMASK        "255.255.255.0"
#define CFGDEF_AP_IPSTART       "192.168.0.112"
#define CFGDEF_AP_IPEND         "192.168.0.250"

#define CFGDEF_LOCOQUEUE        32
#define CFGDEF_LOCOTOUT         40
#define CFGDEF_DCCQUEUE         16
#define CFGDEF_DCCLONGPRE       40
#define CFGDEF_DCCPREAMBLE      28
#define CFGDEF_DCCINITSEQ       20

// dane konfiguracji urządzenia
#include <esp_types.h>

typedef struct {
	// konfiguracja generatora DCC
    uint8_t  locoTimeout;
    uint8_t  locoMaxQueue;

    uint8_t  dccMaxQueue;
    uint8_t  dccLongPreamble;
    uint8_t  dccPreamble;
    uint8_t  dccInitSequence;
    // konfiguracja lokalnego AP
    uint8_t  apChannel;
    char     apSSID[MAX_WLAN_NAME+1];
    char     apPass[MAX_WLAN_PASS+1];
    // konfiguracja połączenia WiFi (STA)
    char     staSSID[MAX_WLAN_NAME+1];
    char     staPass[MAX_WLAN_PASS+1];
    // konfiguracja DHCP (lokalny AP))
    uint32_t apIpAddr;
    uint32_t apIpGw;
    uint32_t apIpMask;
    uint32_t apIpStart;
    uint32_t apIpEnd;

} stationConfig_td;

#endif // WICS_CONFIG_H
