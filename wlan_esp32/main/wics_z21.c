//
// Project: Wireless Command Station
// File:    wics_z21.c
// Author:  Nagus
// Version: 20200301
//

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "wics_config.h"
#include "wics_station.h"
#include "wics_qloco.h"
#include "wics_z21.h"

static const char *TAG = "WiCS.z21";

// komunikacja z siecią LAN
int 			lanSocket;		// socket LAN
uint8_t         *incLanBuf;		// bufor wejściowy LAN
struct sockaddr_in lanAddr;	    // adres docelowy dla datagramu

// rekord abonamentu rozgłaszania
typedef struct {
    in_addr_t ip;       // adres IP manipulatora
    uint32_t  flags;    // rodzaj abonamentu
} clientBroadcast_td;

static uint8_t 	z21Response[Z21NET_MAX_BUFFER];     // bufor odpowiedzi
static clientBroadcast_td *clientBroadcastList;     // lista abonamentów

extern stationStatus_td   stationStatus;            // stan centralki Z21
extern stationConfig_td   appConfig;                // konfiguracja aplikacji
extern EventGroupHandle_t appEventGroup;            // stan aplikacji

uint8_t Z21NET_BroadcastDatagram(uint32_t z21broadcast, in_addr_t addr);

// wysłanie odpowiedzi - bc_stopped
// param: brak
// zwrot: brak
void XBUS_SendStopped()
{
    *((uint16_t*)(&z21Response[i_Z21_HEADER])) = LAN_XBUS_MESSAGE;
    z21Response[i_X21_HEADER]  = LAN_X_BC_STOPPED;
    z21Response[i_X21_DATA0]   = 0x00;
    z21Response[i_X21_DATA0+1] = LAN_X_BC_STOPPED;
    *((uint16_t*)(&z21Response[i_Z21_DATALEN])) = 7;

    Z21NET_BroadcastDatagram(Z21_BC_OPER, IPADDR_ANY);

} // XBUS_SendStopped

// przetwarzanie pakietu X-Bus: CS Status
// cmd:   stan centralki
// zwrot: brak
void XBUS_StationConfig(uint8_t cmd)
{
    switch (cmd) {
    // wersja X-Bus
    case X_GET_VERSION:
    	z21Response[i_X21_HEADER]  = LAN_X_STATUS;
        z21Response[i_X21_DATA0]   = X_VERSION;
        z21Response[i_X21_DATA0+1] = XBUS_PROTOCOL;
        z21Response[i_X21_DATA0+2] = XBUS_STATIONTYPE;
        z21Response[i_Z21_DATALEN] = 9;
        break;
    // status centralki
    case X_GET_STATUS:
		z21Response[i_X21_HEADER]  = LAN_X_STATUS_CHANGED;
        z21Response[i_X21_DATA0]   = X_CENTRAL_STATE;
        z21Response[i_X21_DATA0+1] = stationStatus.centralState;
        z21Response[i_Z21_DATALEN] = 8;
        break;
    // wyłączenie napięcia na torach
    case X_SET_TRACK_POWER_OFF:
        STATION_TrackVoltage_Off();

        z21Response[i_X21_HEADER]  = LAN_X_STATION_STATUS;
        z21Response[i_X21_DATA0]   = X_BC_TRACK_POWER_OFF;
        z21Response[i_Z21_DATALEN] = 7;
    	break;
    // włączenie napięcia na torach
    case X_SET_TRACK_POWER_ON:
        STATION_TrackVoltage_On();

        z21Response[i_X21_HEADER]  = LAN_X_STATION_STATUS;
        z21Response[i_X21_DATA0]   = X_BC_TRACK_POWER_ON;
        z21Response[i_Z21_DATALEN] = 7;
    	break;
    // nie rozpoznane polecenie
    default:
        ESP_LOGI(TAG, "Unknown XBUS CSstatus: %02X", cmd);
        break;
    } // switch cmd

} // XBUS_StationConfig

// przetwarzanie datagramu X-Bus
// datagram:   treść datagramu
// fbroadcast: wskazanie flagi rozgłaszania odpowiedzi
// zwrot:    brak
void XBUS_DatagramProcess(uint8_t *datagram, uint32_t *fbroadcast)
{
    uint8_t xlen = datagram[i_Z21_DATALEN];
    uint8_t xsum = 0;
    uint8_t cnt;

    // sprawdzenie sumy kontrolnej
    for (cnt = i_X21_HEADER; cnt < xlen; cnt++)
        xsum ^= datagram[cnt];
    if (xsum != 0) {
        // suma kontrolna nie zgadza się - nieprawidłowy pakiet
        return;
    }

    switch (datagram[i_X21_HEADER]) {
    // zmiana stanu lokomotywy
    case LAN_X_SET_LOCO_OPER:
    	z21Response[i_X21_HEADER] = LAN_X_LOCO_INFO;
        xlen = QLOCO_UpdateOper(&(datagram[i_X21_DATA0]),
                                &(z21Response[i_X21_DATA0]));
        if (xlen != 0) {
            z21Response[i_Z21_DATALEN] = xlen + 6;
            *fbroadcast = Z21_BC_OPER;
        }
        break;
    // żądanie stanu lokomotywy
    case LAN_X_GET_LOCO_INFO:
		z21Response[i_X21_HEADER] = LAN_X_LOCO_INFO;
        xlen = QLOCO_GetInfo(&(datagram[i_X21_DATA0]),
                             &(z21Response[i_X21_DATA0]));
        if (xlen != 0) {
            z21Response[i_Z21_DATALEN] = xlen + 6;
            *fbroadcast = Z21_BC_OPER;
        }
        break;
	// awaryjne zatrzymanie
	case LAN_X_SET_STOP:
        stationStatus.centralState |= CS_EMERGENCY_STOP;
        STATION_EmergencyStop();

        z21Response[i_X21_HEADER]  = LAN_X_BC_STOPPED;
        z21Response[i_X21_DATA0]   = 0x00;
        z21Response[i_Z21_DATALEN] = 7;
        *fbroadcast = Z21_BC_OPER;
		break;
    // konfiguracja centralki
    case LAN_X_STATION_CONFIG:
        XBUS_StationConfig(datagram[i_X21_DATA0]);
        break;
    // konfiguracja modułu X-Bus
    case LAN_X_GET_XBUSCONFIG:
        z21Response[i_X21_HEADER]  = LAN_X_XBUSCONFIG;
        z21Response[i_X21_DATA0]   = X_FIRMWARE_VERSION;
        z21Response[i_X21_DATA0+1] = XBUS_VERSION_MAJOR;
        z21Response[i_X21_DATA0+2] = XBUS_VERSION_MINOR;
        z21Response[i_Z21_DATALEN] = 9;
        *fbroadcast = Z21_BC_OPER;
        break;
    // nie rozpoznany datagram
    default:
        ESP_LOGI(TAG, "Unknown XBUS datagram: %02X", datagram[i_X21_HEADER]);
        z21Response[i_X21_HEADER]  = LAN_X_LOCO_INFO;
        z21Response[i_X21_DATA0]   = X_UNKNOWN_COMMANMD;
        z21Response[i_Z21_DATALEN] = 7;
        break;
    } // switch xheader

    if (z21Response[i_Z21_DATALEN] != 0) {
		// uzupełnienie odpowiedzi
		*((uint16_t*)(&z21Response[i_Z21_HEADER])) = LAN_XBUS_MESSAGE;
        xsum = 0;
        xlen = z21Response[i_Z21_DATALEN] - 1;
        // suma kkontrolna
        for (cnt = i_X21_HEADER; cnt < xlen; cnt++)
            xsum ^= z21Response[cnt];
        z21Response[xlen] = xsum;
	}

} // XBUS_DatagramProcess

// wysłanie datagramu do terminala sieciowego
// ip:       adres docelowy
// response: pakiet danych
// zwrot:    brak
void Z21NET_SendDatagram(const in_addr_t ip, uint8_t *response)
{
    if (lanSocket < 0) {
		return;
	}

	lanAddr.sin_addr.s_addr = ip;
	int len = sizeof(lanAddr);

	int retVal = sendto(lanSocket, (const void*)response, *((uint16_t*)response),
                        0, (struct sockaddr *)&lanAddr, len);
	if (retVal < 0) {
		ESP_LOGE(TAG, "UDP datagram send failed");
	}

} // Z21NET_SendDatagram

// wysłanie datagramu Z21 do klientów z abonamentem
// z21broadcast: flaga rozgłaszania odpowiedzi
// addr:  adres pytającego
// zwrot: liczba wysłanych odpowiedzi
uint8_t Z21NET_BroadcastDatagram(uint32_t z21broadcast, in_addr_t addr)
{
    if (clientBroadcastList == NULL)
        return Z21NET_MAX_USER;

    uint8_t cnt = 0;
    uint8_t idx;
    for (idx = 0; idx < Z21NET_MAX_USER; idx++) {
        if ((clientBroadcastList[idx].flags & z21broadcast) != 0) {
            if (clientBroadcastList[idx].ip != addr) {
                cnt++;
                Z21NET_SendDatagram(clientBroadcastList[idx].ip, z21Response);
            }
        }
    }
    return cnt;

} // Z21NET_BroadcastDatagram

// zarządzanie flagami broadcast dla klienta
// addr:    adres manipulatora
// newflag: nowa flaga abonamentu, 0 oznacza usunięcie
// zwrot:   brak
void Z21NET_UpdateRecipient(in_addr_t addr, uint32_t newflag)
{
    if (clientBroadcastList == NULL)
        return;

    uint8_t imax = Z21NET_MAX_USER;
    uint8_t idx;
    // uaktualnienie flagi broadcast dla klienta
    for (idx = 0; idx < imax; idx++) {
        if ((clientBroadcastList[idx].flags != 0) &&
                (clientBroadcastList[idx].ip == addr)) {
            clientBroadcastList[idx].flags = newflag;
            return;
        }
    }

    if (newflag != 0) {
        // klient nie miał ustawionej flagi - dodanie do listy
        for (idx = 0; idx < imax; idx++) {
            if (clientBroadcastList[idx].flags == 0) {
                clientBroadcastList[idx].ip    = addr;
                clientBroadcastList[idx].flags = newflag;
            }
        }
    }

} // Z21NET_UpdateRecipient

// numer seryjny urządzenia
// param: brak
// zwrot: adres MAC jako numer seryjny urządzenia
uint32_t Z21NET_GetSerialNum()
{
    uint8_t theMAC[8];
    esp_base_mac_addr_get(theMAC);
    ESP_LOGI(TAG, "Base MAC %X", *((uint32_t*)(&(theMAC[2]))));

    return *((uint32_t*)(&(theMAC[2])));

} // Z21NET_GetSerialNum

// przetwarzanie datagramu Z21
// ip:       adres manipulatora
// datagram: treść datagramu
// zwrot:    brak
void Z21NET_DatagramProcess(const in_addr_t ip, uint8_t *datagram)
{
    uint32_t z21RespBroadcast = Z21_BC_NONE;
    *((uint16_t*)(&z21Response[i_Z21_DATALEN])) = 0;

    // kod protokołu Z21
    switch (*((uint16_t*)&(datagram[i_Z21_HEADER]))) {
    // pakiet X-Bus
    case LAN_XBUS_MESSAGE:
        XBUS_DatagramProcess(datagram, &z21RespBroadcast);
        break;
    // ustawienie flagi broadcast
    case LAN_SET_BROADCASTFLAGS:
        Z21NET_UpdateRecipient(ip, *((uint32_t*)(&datagram[i_Z21_DATA0])));
        break;
    // tryb pracy lokomotywy
    case LAN_GET_LOCOMODE:
        *((uint16_t*)(&z21Response[i_Z21_HEADER])) = LAN_LOCOMODE;
        z21Response[i_Z21_DATA0]   = datagram[i_Z21_DATA0]; // lokAddr
        z21Response[i_Z21_DATA0+1] = datagram[i_Z21_DATA0+1];
        z21Response[i_Z21_DATA0+2] = LOCOMODE_DCC; // obsługiwany tylko tryb DCC
        z21Response[i_Z21_DATALEN] = 7;
        break;
    // numer seryjny
    case LAN_GET_SERIAL_NUMBER:
        *((uint16_t*)(&z21Response[i_Z21_HEADER])) = LAN_SERIAL_NUMBER;
        *((uint32_t*)(&z21Response[i_Z21_DATA0]))  = Z21NET_GetSerialNum();
        z21Response[i_Z21_DATALEN] = 8;
        break;
    // wersja HW/FW
    case LAN_GET_HWINFO:
        *((uint16_t*)(&z21Response[i_Z21_HEADER]))  = LAN_HWINFO;
        *((uint32_t*)(&z21Response[i_Z21_DATA0]))   = Z21_HW_VERSION;
        *((uint32_t*)(&z21Response[i_Z21_DATA0+4])) = Z21_FW_VERSION;
        z21Response[i_Z21_DATALEN] = 12;
        break;
    // kody dostępu
    case LAN_GET_CODE:
        *((uint16_t*)(&z21Response[i_Z21_HEADER]))  = LAN_Z21CODE;
        z21Response[i_Z21_DATA0]   = Z21_CODE_NOLOCK;
        z21Response[i_Z21_DATALEN] = 5;
        break;
    // nierozpoznany datagram
    default:
        ESP_LOGI(TAG, "Unknown Z21 datagram: %04X",
                 *((uint16_t*)&(datagram[i_Z21_HEADER])));
        break;
    } // switch z21 header

    // wysłanie odpowiedzi
	if (*((uint16_t*)(&z21Response[i_Z21_DATALEN])) != 0) {
		Z21NET_SendDatagram(ip, z21Response);
		if (z21RespBroadcast != Z21_BC_NONE) {
			Z21NET_BroadcastDatagram(z21RespBroadcast, ip);
		}
	} // wysłanie odpowiedzi

} // Z21NET_DatagramProcess

// wątek komunikacji z terminalami sieciowymi
static void Z21NET_NetworkTask(void *pvParameters)
{
    struct sockaddr_in srvAddr;
	struct sockaddr_in fromAddr;
	socklen_t fromLen;

	// pętla łącza
	while (true) {
        xEventGroupWaitBits(appEventGroup,
                            WIFI_CONNECTED_BIT, true, true, portMAX_DELAY);
        bzero(&srvAddr, sizeof(srvAddr));

        lanSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (lanSocket < 0) {
            ESP_LOGE(TAG, "LAN socket error");
            vTaskDelay(Z21NET_RETRY_TOUT);
            continue;
        }

        srvAddr.sin_family = AF_INET;
		srvAddr.sin_port = htons(Z21NET_PORT_NUM);
		srvAddr.sin_addr.s_addr = INADDR_ANY;
		lanAddr.sin_family = AF_INET;
		lanAddr.sin_port = srvAddr.sin_port;

		if (0 != bind(lanSocket, (struct sockaddr *)&srvAddr, sizeof(srvAddr))) {
			ESP_LOGE(TAG, "Bind error! port: %d", Z21NET_PORT_NUM);
			vTaskDelay(Z21NET_RETRY_TOUT);
            continue;
		}

		// pętla transmisji
		while (true) {
            int retVal = 0;
            bzero(&fromAddr, sizeof(fromAddr));
            fromLen = sizeof(struct sockaddr_in);
            retVal = recvfrom(lanSocket, incLanBuf, Z21NET_MAX_BUFFER, 0,
                              (struct sockaddr*)&fromAddr, &fromLen);
            if (retVal > 0) {
                Z21NET_DatagramProcess(fromAddr.sin_addr.s_addr, incLanBuf);
            }
            else {
                ESP_LOGE(TAG, "UDP receive error");
                continue;
            }

		} // pętla transmisji

		close(lanSocket);
        lanSocket = -1;

	} // pętla łącza

	free(incLanBuf);
	incLanBuf = NULL;
	vTaskDelete(NULL);

} // Z21NET_NetworkTask

// uruchomienie aplikacji
// param: brak
// zwrot: brak
void Z21NET_Start()
{
    ESP_LOGI(TAG, "Z21NET starting: %dB", esp_get_free_heap_size());

    // bufor danych abonamentów odpowiedzi
    clientBroadcastList = (clientBroadcast_td*)malloc(
    							sizeof(clientBroadcast_td) * Z21NET_MAX_USER);
    if (clientBroadcastList == NULL) {
        ESP_LOGE(TAG, "CS buffer allocation error, free: %dB",
                 esp_get_free_heap_size());
    }
    else {
        bzero((void*)clientBroadcastList,
              sizeof(clientBroadcast_td) * Z21NET_MAX_USER);
    }

    // komunikacja z siecią LAN (manipulatory)
    lanSocket = -1;
	bzero((void*)&lanAddr, sizeof(lanAddr));
	incLanBuf = (uint8_t*)malloc(Z21NET_MAX_BUFFER + 1);
    if (incLanBuf != NULL) {
    	xTaskCreate(Z21NET_NetworkTask, "net_task", 4096, NULL, 5, NULL);
    }
    else {
    	ESP_LOGE(TAG, "LAN buffer allocation error, free %dB",
    			 esp_get_free_heap_size());
    }

} // Z21NET_Start

// EOF wics_z21.c
