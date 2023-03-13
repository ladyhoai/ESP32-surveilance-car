
#ifndef WIFI_INIT_H
#define WIFI_INIT_H

#include <esp_wifi.h>
#include <esp_log.h>
#include <freertos/event_groups.h>
#include <string.h>
#include <netdb.h>

#define SSID "Beverly Hills"
#define PASS "Baolinh2209@#$"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char* WIFI_TAG = "WIFI";

void connect_to_wifi(void (*event_handler) (void*, esp_event_base_t, int32_t, void*));
void set_static_ip(const char* ip, const char* netmask, const char* gateway, esp_netif_t* netif);
int get_wifi_setting_recon_num();

#endif