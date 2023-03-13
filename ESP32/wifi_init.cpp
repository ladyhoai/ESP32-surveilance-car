#include <wifi_init.h>

struct wifi_setting {
    uint8_t max_retry_reconnecting_num = 10;
} wifi_user_setting;

int get_wifi_setting_recon_num() {
    return wifi_user_setting.max_retry_reconnecting_num;
}

void set_static_ip(const char* ip, const char* netmask, const char* gateway, esp_netif_t* netif) {
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        return;
    }
    esp_netif_ip_info_t ip_if;
    memset(&ip_if, 0, sizeof(esp_netif_ip_info_t));
    ip_if.ip.addr = ipaddr_addr(ip);
    ip_if.netmask.addr = ipaddr_addr(netmask);
    ip_if.gw.addr = ipaddr_addr(gateway);
    if (esp_netif_set_ip_info(netif, &ip_if) != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "Failed to set IP info");
        return;
    }
}

void connect_to_wifi(void (*event_handler) (void*, esp_event_base_t, int32_t, void*)) {
    EventGroupHandle_t s_wifi_event_group;
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI("Wifi", "event loop successfully created");
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_disconnected;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_t instance_start_wifi;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, event_handler, sta_netif, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, event_handler, NULL, &instance_start_wifi));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, s_wifi_event_group, &instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, event_handler, s_wifi_event_group, &instance_disconnected));

    wifi_config_t wifi_conf = {
        .sta {
            .ssid = SSID,
            .password = PASS,
        }
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_conf));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Up till this point, the sta init has finished
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI("Wifi", "Wifi bits set");
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "Connected to SSID: %s", SSID);
    }
    else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to SSID: %s", SSID);
    }
    else {
        ESP_LOGI(WIFI_TAG, "Unexpected Wifi Event");
    }
    
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &instance_any_id));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_START, &instance_start_wifi));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &instance_disconnected));
    // vEventGroupDelete(s_wifi_event_group);
}