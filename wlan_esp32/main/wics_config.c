//
// Project: Wireless Command Station
// File:    wics_config.c
// Author:  Nagus
// Version: 20200215
//

#include <string.h>

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "wics_config.h"

static const char *TAG = "wiCS.cfg";

extern stationConfig_td appConfig;

// inicjalizacja pamięci
// param: brak
// zwrot: brak
void CONFIG_Initialize()
{
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (err == ESP_ERR_NVS_NEW_VERSION_FOUND)) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

} // CONFIG_Initialize

// zapis ustawień
// param: brak
// zwrot: brak
void CONFIG_SaveSettings()
{
    esp_err_t err;
    nvs_handle cfgHandle;

    err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &cfgHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!\n");
        return;
    }

    size_t requiredSize = sizeof(appConfig);
    err = nvs_set_blob(cfgHandle, "wics_config", &appConfig, requiredSize);
    if (err == ESP_OK) {
        err = nvs_commit(cfgHandle);
    }

    nvs_close(cfgHandle);

} // CONFIG_SaveSettings

// przywrócenie ustawień domyślnych
// param: brak
// zwrot: brak
void CONFIG_RestoreDefault()
{
	const stationConfig_td defConfig = {
        .locoMaxQueue   = CFGDEF_LOCOQUEUE,
        .locoTimeout    = CFGDEF_LOCOTOUT,

        .dccMaxQueue        = CFGDEF_DCCQUEUE,
        .dccLongPreamble    = CFGDEF_DCCLONGPRE,
        .dccPreamble        = CFGDEF_DCCPREAMBLE,
        .dccInitSequence    = CFGDEF_DCCINITSEQ,

        .apChannel	    = CFGDEF_AP_CHANNEL,
        .apSSID         = CFGDEF_AP_SSID,
        .apPass     	= CFGDEF_AP_PASS,
        .staSSID        = CONFIG_STA_WIFI_SSID,
        .staPass    	= CONFIG_STA_WIFI_PASS,

        .apIpAddr       = ipaddr_addr(CFGDEF_AP_IPADDR),
        .apIpGw         = ipaddr_addr(CFGDEF_AP_IPGW),
        .apIpMask       = ipaddr_addr(CFGDEF_AP_IPMASK),
        .apIpStart      = ipaddr_addr(CFGDEF_AP_IPSTART),
        .apIpEnd        = ipaddr_addr(CFGDEF_AP_IPEND)
	}; // defConfig

	ESP_LOGI(TAG, "Restore default settings");
    memcpy(&appConfig, &defConfig, sizeof(stationConfig_td));
    CONFIG_SaveSettings();

} // CONFIG_RestoreDefault

// wczytanie ustawień
// param: brak
// zwrot: brak
void CONFIG_LoadSettings()
{
    esp_err_t err;
    nvs_handle cfgHandle;

    err = nvs_open(CONFIG_NAMESPACE, NVS_READWRITE, &cfgHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!\n");
        CONFIG_RestoreDefault();
        return;
    }

    size_t requiredSize = 0;
    err = nvs_get_blob(cfgHandle, "wics_config", NULL, &requiredSize);
    if ((err != ESP_OK) || (requiredSize != sizeof(appConfig))) {
        ESP_LOGE(TAG, "Get config error, required: %d", requiredSize);
        CONFIG_RestoreDefault();
    }
    else {
        err = nvs_get_blob(cfgHandle, "wics_config", &appConfig, &requiredSize);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Config read error");
            CONFIG_RestoreDefault();
        }
    }

    nvs_close(cfgHandle);

} // CONFIG_LoadSettings

// EOF wics_config.c
