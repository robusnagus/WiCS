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

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "cJSON.h"

#include "wics_config.h"
#include "wics_station.h"

static const char *TAG = "WiCS.UI";

extern stationConfig_td appConfig;      // konfiguracja centralki
extern stationStatus_td stationStatus;  // stan centralki Z21

// ustawienie konfiguracji WLAN przez panel webUI
// parsed: nowe dane (json)
// zwrot:  brak
void WEB_ConfigWlan(cJSON *parsed)
{
    cJSON *item = cJSON_GetObjectItem(parsed, "stassid");
    char *theStr;
    if (item != NULL) {
        theStr = cJSON_GetStringValue(item);
        if (theStr != NULL) {
            strncpy(appConfig.staSSID, theStr, MAX_WLAN_NAME);
        }
    }

    item = cJSON_GetObjectItem(parsed, "stapass");
    if (item != NULL) {
        theStr = cJSON_GetStringValue(item);
        if (theStr != NULL) {
            strncpy(appConfig.staPass, theStr, MAX_WLAN_PASS);
        }
    }

} // WEB_ConfigWlan

extern void CONFIG_SaveSettings();

// obsługa danych wysyłanych przez panel webUI
// req:   dane żądania
// zwrot: kod wykonania operacji
esp_err_t WEB_PostConfigHandler(httpd_req_t *req)
{
    char *postData = malloc(req->content_len + 1);
    if (postData == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for post, free %dB",
                 esp_get_free_heap_size());
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    size_t dataOff = 0;
    int retVal;
    while (dataOff < req->content_len) {
        retVal = httpd_req_recv(req, postData + dataOff,
                                req->content_len - dataOff);
        if (retVal <= 0) {
            if (retVal == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            }
            free(postData);
            return ESP_FAIL;
        }
        dataOff += retVal;
    }
    postData[dataOff] = '\0';

    cJSON *parsed = cJSON_Parse(postData);
    if (parsed != NULL) {
        if (strcmp(req->uri + 5, "wlan") == 0) {
            WEB_ConfigWlan(parsed);
        }

        cJSON_Delete(parsed);
        CONFIG_SaveSettings();
    }

    free(postData);
    return ESP_OK;

} // WEB_PostConfigHandler

// obsługa aktualizacji oprogramowania
// req:   dane żądania
// zwrot: kod wykonania operacji
esp_err_t WEB_PostUpgradeHandler(httpd_req_t *req)
{
    esp_err_t err;
    esp_ota_handle_t updateHandle = 0 ;
    const esp_partition_t *updatePartition = NULL;
    const esp_partition_t *runningPartition = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             runningPartition->type, runningPartition->subtype,
             runningPartition->address);

    updatePartition = esp_ota_get_next_update_partition(NULL);
    assert(updatePartition != NULL);
    err = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &updateHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA begin failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    char *postData = malloc(WIFI_MAX_POSTBUFSIZE + 1);
    if (postData == NULL) {
        ESP_LOGE(TAG, "No memory for upgrade data, free: %dB",
                 esp_get_free_heap_size());
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Upgrade failed: no memory.");
        return ESP_FAIL;
    }

    int bytesRemaining = req->content_len;
    int bytesWritten = 0;
    int bytesReceived;
    while (bytesRemaining > 0) {
        if ((bytesReceived = httpd_req_recv(req, postData,
                    (bytesRemaining > WIFI_MAX_POSTBUFSIZE
                    ? WIFI_MAX_POSTBUFSIZE : bytesRemaining))) <= 0) {
            if (bytesReceived == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Firmware file reception failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "Failed to receive file");
            free(postData);
            return ESP_FAIL;
        }

        err = esp_ota_write(updateHandle, (const void *)postData, bytesReceived);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Flash write failed!");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "Flash write failed");
            free(postData);
            return ESP_FAIL;
        }
        bytesRemaining -= bytesReceived;
        bytesWritten += bytesReceived;

    } // bytesRemaining

    if (esp_partition_check_identity(esp_ota_get_running_partition(),
                                     updatePartition) == false) {
        err = esp_ota_set_boot_partition(updatePartition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!",
        			 esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "Set boot partition failed!");
            free(postData);
            return ESP_FAIL;
        }
        else {
            ESP_LOGI(TAG, "New parition set");
        }
    }
    else {
        ESP_LOGI(TAG, "Firmware already uploaded");
    }
    free(postData);
    return ESP_OK;

} // WEB_PostUpgradeHandler

extern uint32_t Z21NET_GetSerialNum();

// obsługa pobierania danych przez panel webUI
// req:   dane żądania
// zwrot: kod wykonania operacji
esp_err_t WEB_GetDataHandler(httpd_req_t *req)
{
    char *theBody = NULL;
    cJSON *item;
    cJSON *response = cJSON_CreateObject();
    httpd_resp_set_type(req, "application/json");
    char tempStr[32];

    if (strcmp(req->uri + 6, "stat") == 0) {
        // stan centralki - panel główny
        sprintf(tempStr, "%.2f V", (float)stationStatus.supplyVoltage * 0.001);
        item = cJSON_CreateString(tempStr);
        cJSON_AddItemToObject(response, "mvoltage", item);
        sprintf(tempStr, "%.3f A", (float)stationStatus.mainCurrent * 0.001);
        item = cJSON_CreateString(tempStr);
        cJSON_AddItemToObject(response, "mcurrent", item);

        theBody = cJSON_Print(response);
    }
    else if (strcmp(req->uri + 6, "wlan") == 0) {
        // konfiguracja centralki
        item = cJSON_CreateString(appConfig.staSSID);
        cJSON_AddItemToObject(response, "stassid", item);
        item = cJSON_CreateString(appConfig.staPass);
        cJSON_AddItemToObject(response, "stapass", item);

        theBody = cJSON_Print(response);
    }
    else if (strcmp(req->uri + 6, "info") == 0) {
        // informacje o urządzeniu - aktualizacja
        sprintf(tempStr, "%d.%d", HW_VERSION_MAJOR, HW_VERSION_MINOR);
        item = cJSON_CreateString(tempStr);
        cJSON_AddItemToObject(response, "hw_version", item);
        const esp_app_desc_t* adesc = esp_ota_get_app_description();
        item = cJSON_CreateString(adesc->version);
        cJSON_AddItemToObject(response, "sw_version", item);
        sprintf(tempStr, "%X08", Z21NET_GetSerialNum());
        item = cJSON_CreateString(tempStr);
        cJSON_AddItemToObject(response, "serialnum", item);

        theBody = cJSON_Print(response);
    }
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file name");
        cJSON_Delete(response);
        return ESP_FAIL;
    }

    if (theBody != NULL) {
        httpd_resp_send(req, (const char *)theBody, strlen(theBody));
        free(theBody);
        cJSON_Delete(response);
        return ESP_OK;
    }
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameter");
    }

    cJSON_Delete(response);
    return ESP_FAIL;

} // WEB_GetDataHandler

// obsługa pobierania plików webUI
// req:   dane żądania
// zwrot: kod wykonania operacji
esp_err_t WEB_GetFileHandler(httpd_req_t *req)
{
    if ((strlen(req->uri) <= 1) || (strcmp(req->uri, "/index.htm") == 0)) {
        extern const uint8_t index_htm_start[] asm("_binary_index_htm_start");
        extern const uint8_t index_htm_end[]   asm("_binary_index_htm_end");
        const size_t index_html_size = (index_htm_end - index_htm_start);

        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, (const char *)index_htm_start, index_html_size);
    }
    else {
        if (strcmp(req->uri, "/esp32.css") == 0) {
            extern const uint8_t esp32_css_start[] asm("_binary_esp32_css_start");
            extern const uint8_t esp32_css_end[]   asm("_binary_esp32_css_end");
            const size_t esp32_css_size = (esp32_css_end - esp32_css_start);

            httpd_resp_set_type(req, "text/css");
            httpd_resp_send(req, (const char *)esp32_css_start, esp32_css_size);
        }
        else if (strcmp(req->uri, "/favicon.ico") == 0) {
            extern const uint8_t ngsc32_ico_start[] asm("_binary_ngsc32_ico_start");
            extern const uint8_t ngsc32_ico_end[]   asm("_binary_ngsc32_ico_end");
            const size_t ngsc32_ico_size = (ngsc32_ico_end - ngsc32_ico_start);

            httpd_resp_set_type(req, "image/vnd.microsoft.icon");
            httpd_resp_send(req, (const char *)ngsc32_ico_start, ngsc32_ico_size);
        }
        else if (strcmp(req->uri, "/config.htm") == 0) {
            extern const uint8_t config_htm_start[] asm("_binary_config_htm_start");
            extern const uint8_t config_htm_end[]   asm("_binary_config_htm_end");
            const size_t config_html_size = (config_htm_end - config_htm_start);

            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, (const char *)config_htm_start, config_html_size);
        }
        else if (strcmp(req->uri, "/upgrade.htm") == 0) {
            extern const uint8_t upgrade_htm_start[] asm("_binary_upgrade_htm_start");
            extern const uint8_t upgrade_htm_end[]   asm("_binary_upgrade_htm_end");
            const size_t upgrade_html_size = (upgrade_htm_end - upgrade_htm_start);

            httpd_resp_set_type(req, "text/html");
            httpd_resp_send(req, (const char *)upgrade_htm_start, upgrade_html_size);
        }
        else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid file name");
            return ESP_FAIL;
        }
    }

    return ESP_OK;

} // WEB_GetFileHandler

// rejestracja obsługiwanych uri
// webHandle: uchwyt serwera
// zwrot:     brak
void WEB_RegisterWebHandlers(httpd_handle_t webHandle)
{
    // dane wysyłane przez panel webUI
    httpd_uri_t postConfig = {
        .uri       = "/cfg/*",
        .method    = HTTP_POST,
        .handler   = WEB_PostConfigHandler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(webHandle, &postConfig);

    // aktualizacja oprogramowania
    httpd_uri_t postUpgrade = {
        .uri       = "/upgrade",
        .method    = HTTP_POST,
        .handler   = WEB_PostUpgradeHandler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(webHandle, &postUpgrade);

    // pobieranie danych
    httpd_uri_t webDataFiles = {
        .uri       = "/json/*",
        .method    = HTTP_GET,
        .handler   = WEB_GetDataHandler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(webHandle, &webDataFiles);

    // pobieranie plików
    httpd_uri_t webuiFiles = {
        .uri       = "*",
        .method    = HTTP_GET,
        .handler   = WEB_GetFileHandler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(webHandle, &webuiFiles);

} // WEB_RegisterWebHandlers

// uruchomienie serwera http (webUI)
// param: brak
// zwrot: uchwyt serwera, NULL przy niepowodzeniu
httpd_handle_t WEB_ServerStart()
{
    httpd_handle_t webHandle;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.server_port = WIFI_PORT_WEBUI;
    config.max_open_sockets = (CONFIG_LWIP_MAX_SOCKETS - 3);

    if (httpd_start(&webHandle, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Started HTTP server on port: '%d'", config.server_port);
        WEB_RegisterWebHandlers(webHandle);
        return webHandle;
    }

    ESP_LOGE(TAG, "HTTP server failed");
    return NULL;

} // WEB_ServerStart

// zatrzymanie serwera http (webUI)
// webhandle: uchwyt serwera
// zwrot:     brak
void WEB_ServerStop(httpd_handle_t webhandle)
{
    ESP_LOGI(TAG, "Stopped HTTP server");
    httpd_stop(webhandle);
}

// EOF wics_webui.c
