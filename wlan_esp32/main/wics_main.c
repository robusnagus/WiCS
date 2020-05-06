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

#include <stdio.h>

#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "wics_config.h"

static const char *TAG = "wiCS";

// konfiguracja aplikacji
stationConfig_td appConfig;
esp_adc_cal_characteristics_t *appADCchars;

// stan aplikacji / połączenia
EventGroupHandle_t    appEventGroup;
int                   appStaConnFail;
esp_netif_t*          appNetIf;
static httpd_handle_t webHandle;

extern httpd_handle_t WEB_ServerStart();
extern void WEB_ServerStop(httpd_handle_t webhandle);
void WiFi_StopSTA();
void WIFI_StartAP();

// obsługa zdarzeń WiFi - tryb STA
static void WIFI_EventHandlerSTA(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START)) {
        esp_netif_set_hostname(appNetIf, Z21NET_HOSTNAME);
        esp_wifi_connect();
    }
    else if ((event_base == WIFI_EVENT)
                && (event_id == WIFI_EVENT_STA_DISCONNECTED)) {
        if (appStaConnFail < WIFI_MAX_STARETRY) {
            esp_wifi_connect();
            appStaConnFail++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        }
        else {
            WiFi_StopSTA();
            WIFI_StartAP();
        }
        xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);
        GPIO_LED_Wifi_Off();
    }
    else if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP)) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        appStaConnFail = 0;
        xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);
        GPIO_LED_Wifi_On();
        if (webHandle == 0) {
            webHandle = WEB_ServerStart();
        }
    }

} // WIFI_EventHandlerSTA

// obsługa zdarzeń WiFi - tryb AP
static void WIFI_EventHandlerAP(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        esp_netif_set_hostname(appNetIf, Z21NET_HOSTNAME);
        xEventGroupSetBits(appEventGroup, WIFI_CONNECTED_BIT);
        GPIO_LED_Wifi_On();
        if (webHandle == 0) {
            webHandle = WEB_ServerStart();
        }
    }
    else if (event_id == WIFI_EVENT_AP_STOP) {
        xEventGroupClearBits(appEventGroup, WIFI_CONNECTED_BIT);
        if (webHandle != 0) {
            WEB_ServerStop(webHandle);
            webHandle = 0;
        }
        GPIO_LED_Wifi_Off();
    }
    else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event =
                    (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event =
                    (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }

} // WIFI_EventHandlerAP

// uruchomienie softAP
// param: brak
// zwrot: brak
void WIFI_StartAP()
{
    appNetIf = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
                    WIFI_EVENT, ESP_EVENT_ANY_ID, &WIFI_EventHandlerAP, NULL));

    wifi_config_t wifiConfig;
    bzero(&wifiConfig, sizeof(wifi_config_t));
    strncpy((char*)(wifiConfig.ap.ssid), appConfig.apSSID, MAX_WLAN_NAME);
    wifiConfig.ap.ssid_len = strlen(appConfig.apSSID);
    strncpy((char*)(wifiConfig.ap.password), appConfig.apPass, MAX_WLAN_PASS);
    wifiConfig.ap.max_connection = 4;
    wifiConfig.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifiConfig.ap.channel = appConfig.apChannel;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (esp_netif_dhcps_stop(appNetIf) == ESP_OK) {
        esp_netif_ip_info_t ipInfo;
        ipInfo.ip.addr      = appConfig.apIpAddr;
        ipInfo.gw.addr      = appConfig.apIpGw;
        ipInfo.netmask.addr = appConfig.apIpMask;
        if (esp_netif_set_ip_info(appNetIf, &ipInfo) == ESP_OK) {
            dhcps_lease_t dhcpConfig;
            dhcpConfig.enable        = true;
            dhcpConfig.start_ip.addr = appConfig.apIpStart;
            dhcpConfig.end_ip.addr   = appConfig.apIpEnd;
            esp_netif_dhcps_option(appNetIf, ESP_NETIF_OP_SET,
                                   ESP_NETIF_REQUESTED_IP_ADDRESS,
                                   (void*)&dhcpConfig, sizeof(dhcps_lease_t));
        }
        ESP_ERROR_CHECK(esp_netif_dhcps_start(appNetIf));
    }

} // WIFI_StartAP

// wyłączenie trybu STA
// param: brak
// zwrot: brak
void WiFi_StopSTA()
{
    ESP_LOGI(TAG, "STA mode stop");
    esp_event_handler_unregister(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID,
                                 &WIFI_EventHandlerSTA);
    esp_netif_destroy(appNetIf);
}

// podłączenie do AP (tryb STA)
// param: brak
// zwrot: brak
void WIFI_StartSTA()
{
    appNetIf = esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
                    WIFI_EVENT, ESP_EVENT_ANY_ID, &WIFI_EventHandlerSTA, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
                    IP_EVENT, IP_EVENT_STA_GOT_IP, &WIFI_EventHandlerSTA, NULL));

    wifi_config_t wifiConfig;
    bzero(&wifiConfig, sizeof(wifi_config_t));
    strncpy((char*)(wifiConfig.sta.ssid), appConfig.staSSID, MAX_WLAN_NAME);
    strncpy((char*)(wifiConfig.sta.password), appConfig.staPass, MAX_WLAN_PASS);

    ESP_LOGI(TAG, "Sta: %s %s", wifiConfig.sta.ssid, wifiConfig.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

} // WIFI_StartSTA

// konfiguracja portów
// param: brak
// zwrot: brak
void APP_GPIO_Init()
{
    gpio_config_t ioConf;
    ioConf.intr_type = GPIO_INTR_DISABLE;

    // wyjścia: sterownik DCC
    ioConf.mode = GPIO_MODE_OUTPUT_OD;
    ioConf.pin_bit_mask = (1ULL << GPIO_DCC_ENA);
    ioConf.pull_down_en = 0;
    ioConf.pull_up_en = 0;
    gpio_config(&ioConf);
    GPIO_DCC_Disable();

    ioConf.mode = GPIO_MODE_OUTPUT_OD;
    ioConf.pin_bit_mask = (1ULL << GPIO_DCC_SIG1);
    ioConf.pull_down_en = 0;
    ioConf.pull_up_en = 0;
    gpio_config(&ioConf);
    GPIO_DCC_Sig1_Hi();

    // wyjścia: LEDy
    ioConf.mode = GPIO_MODE_OUTPUT;
    ioConf.pin_bit_mask = ((1ULL << GPIO_LED_WIFI) | (1ULL << GPIO_LED_DCCR) |
                           (1ULL << GPIO_LED_PROG));
    ioConf.pull_down_en = 0;
    ioConf.pull_up_en = 0;
    gpio_config(&ioConf);
    GPIO_LED_Wifi_Off();
    GPIO_LED_DccS_Off();
    GPIO_LED_Prog_Off();

    // wejścia
    ioConf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    ioConf.pin_bit_mask = ((1ULL << GPIO_BTN_WIFI) | (1ULL << GPIO_BTN_STOP) |
                           (1ULL << GPIO_DCC_ERR));
    ioConf.mode = GPIO_MODE_INPUT;
    gpio_config(&ioConf);

} // APP_GPIO_Init

// konfiguracja ADC
// param: brak
// zwrot: brak
void APP_ADC_Init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DCCG_ADC_VSENS, DCCG_ADC_ATTEN);
    adc1_config_channel_atten(DCCG_ADC_ISENS, DCCG_ADC_ATTEN);

    appADCchars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, DCCG_ADC_ATTEN, ADC_WIDTH_BIT_12,
                             DCCG_ADC_DEFVREF, appADCchars);

} // APP_ADC_Init

extern void CONFIG_Initialize();
extern void CONFIG_LoadSettings();
extern void STATION_Initialize();
extern void Z21NET_Start();

// główna funkcja
void app_main(void)
{
    const esp_app_desc_t* adesc = esp_ota_get_app_description();
    printf("Wireless Command Station %s\n", adesc->version);
    printf("SDK version:%s\n", esp_get_idf_version());
    APP_GPIO_Init();
    APP_ADC_Init();

    appNetIf = NULL;
    appStaConnFail = 0;
    webHandle = 0;
    CONFIG_Initialize();
    CONFIG_LoadSettings();

    appEventGroup = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    STATION_Initialize();
    WIFI_StartSTA();
    Z21NET_Start();

} // app_main

// EOF wics_main.c
