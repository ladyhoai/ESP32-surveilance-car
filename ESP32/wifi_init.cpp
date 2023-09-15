#include <wifi_init.h>
#include "esp_wpa2.h"
#include <string>

struct wifi_setting {
    uint8_t max_retry_reconnecting_num = 10;
} wifi_user_setting;

const char* USERNAME = "14363978";
const char* PASSWORD = "Ln31042003!";
const char* ID = "14363978";

int get_wifi_setting_recon_num() {
    return wifi_user_setting.max_retry_reconnecting_num;
}

void set_static_ip(const char* ip, const char* netmask, const char* gateway, esp_netif_t* netif) {
    // DHCPC client is the program that automatically assign IP address to devices on connecting to the network
    // We have to stop it so that we can set static IP
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

void connect_to_wifi(void (*event_handler) (void*, esp_event_base_t, int32_t, void*), nvs_handle_t handle_flash) {

    EventGroupHandle_t s_wifi_event_group;
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_LOGI("Wifi", "event loop successfully created");
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // netif mean network information
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

    // UTS Authentication
    // esp_wifi_sta_wpa2_ent_set_identity((uint8_t*) ID, strlen(ID));
    // esp_wifi_sta_wpa2_ent_set_username((uint8_t*) USERNAME, strlen(USERNAME));
    // esp_wifi_sta_wpa2_ent_set_password((uint8_t*) PASSWORD, strlen(PASSWORD));
    // esp_wifi_sta_wpa2_ent_enable();

    // Uncomment the line below to use Michael's network
    // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_conf));
    ESP_ERROR_CHECK(esp_wifi_start());

        // wifi_scan_config_t scan = {
        //     .ssid = 0,
        //     .bssid = 0,
        //     .channel = 0,
        //     .show_hidden = true
        //     };

        //     ESP_ERROR_CHECK(esp_wifi_scan_start(&scan, true));
        //     uint16_t number = 5;
        //     wifi_ap_record_t record[number];
        //     memset(record, 0, sizeof(record));
        //     uint16_t ap_count = 0;
        //     ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, record));  
        //     ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        //     ESP_LOGI("WiFi", "Total APs scanned = %u", ap_count);  
            
        //     for (int i = 0; i < ap_count; i++) {
        //         size_t required_size = 0;
        //         ESP_LOGI("Wifi","Signal strength is: %d \n", record[i].rssi);
        //         esp_err_t check = nvs_get_str(handle_flash, (const char*)record[i].ssid, NULL, &required_size);
                
        //         if (check == ESP_OK) {
        //             char* val = (char*) malloc(required_size);
        //             nvs_get_str(handle_flash, (const char*)record[i].ssid, val, &required_size);
        //             ESP_LOGI("Wifi","Password is: %s \n", val);
                    
        //             static wifi_config_t wifi_conf;

        //             strcpy((char*)wifi_conf.sta.ssid, (const char*) record[i].ssid);
        //             strcpy((char*)wifi_conf.sta.password , val);

        //             esp_wifi_set_config(WIFI_IF_STA, &wifi_conf);
                    
        //             free(val);

        //             esp_wifi_connect();

        //             break;
        //         }

        //         else if (check == ESP_ERR_NVS_NOT_FOUND) {
        //             ESP_LOGI("WiFi", "Password is not yet stored for this SSID %s", (const char*)record[i].ssid);
        //             continue;
        //         }
        //    }


    // Up till this point, the sta init has finished
    // Wait bits simply wait for either WIFI_CONNECTED_BIT or WIFI_FAIL_BIT to set, wait time depends on the last param of
    // the function. Here, portMAX_DELAY means the program will keep waiting til' one of the 2 bits is set

    // Maybe in the future we can try to make a timeout for this
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
