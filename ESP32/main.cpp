#include <esp_websocket.h>

#include <wifi_init.h>
#include <audio_handle.h>
#include <servo_handle.h>
#include <i2c_handler.h>
#include <lcd_handler.h>
#include <gpio_init.h>

#include <string>
#include <unordered_map>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <freertos/semphr.h>

#include <esp_event.h>
#include <lwip/sys.h>

#include "nvs_flash.h"

#define UNO 0x16
#define LCD 0x27
#define NANO 0x15
//#define AT_UTS

// Different set of WiFi config at UTS
#ifndef AT_UTS

#define IP "192.168.0.36"
#define NETMASK "255.255.255.0"
#define GATEWAY "192.168.0.1"

#else

#define IP "172.19.124.243"
#define NETMASK "255.255.254.0"
#define GATEWAY "172.19.124.1"

#endif

#define CAM_X_CHANNEL LEDC_CHANNEL_0
#define CAM_Y_CHANNEL LEDC_CHANNEL_1

unsigned int ip_address_display_time = 15000, lcd_update = 0;
static uint8_t wifi_reconnecting_num = 0;

float cam_x_cur_angle = 90, cam_y_cur_angle = 45;
int cam_axis_command = 0;
TaskHandle_t camera_task;
TaskHandle_t sonar_sensor_task;
TaskHandle_t audio_task_handle;
TaskHandle_t lcd_task_handle;

// SemaphoreHandle_t camera_axis_sem;
esp_websocket_client_handle_t audio_client;
esp_websocket_client_handle_t uno_command_client;
uint8_t uno_command_buffer[10];
char* temp_audio_buffer = "";

// Variables to drive LCD
bool lcd_activated = false;
std::string temp_lcd_string = "";
char line_1_lcd[16];
char line_2_lcd[16];

// This connection is for constant streaming of audio
const esp_websocket_client_config_t audio_client_conf = {
  .uri = "ws://192.168.0.51",
  .port = 4001, // This is the server port where our client is connected
  .disable_auto_reconnect = false,
 // .headers = "identifier: audio client",
  .buffer_size = 20000,
};
// This connection is for receiving command from network client
const esp_websocket_client_config_t uno_command_client_conf = {
    .uri = "ws://192.168.0.51",
    .port = 4000,
    .disable_auto_reconnect = false,
  //  .headers = "identifier: Command client",
};

// The number version of command
std::unordered_map<std::string, uint8_t> command = {
    {"w", 1}, {"s", 2}, {"wa", 3} , {"wd", 4}, {"sa", 5}, {"sd", 6}, {"cs", 7}, {"cw", 8}, {"ca", 9}, {"cd", 10},
    {"i", 11}, {"o", 12}, {"aw", 3}, {"dw", 4}, {"as", 5}, {"ds", 6}, {"stop", 0}, {"q", 20}, {"conau", 14}, {"stopau", 13}, {"consu", 15},
    {"stopsu", 16}
};  



void servo_task(void* param) {
    while (true) {
        if (cam_axis_command != 0) {
            switch (cam_axis_command) {

            case 7:
                if (cam_y_cur_angle < 90) {
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CAM_Y_CHANNEL, cam_y_cur_angle + 2);
                cam_y_cur_angle += 2;
                }
            break;

            case 8:
                if (cam_y_cur_angle > 0) {
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CAM_Y_CHANNEL, cam_y_cur_angle - 2);
                cam_y_cur_angle -= 2;
                }
            break;

            case 9:
                if (cam_x_cur_angle < 180) {
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CAM_X_CHANNEL, cam_x_cur_angle + 2);
                cam_x_cur_angle += 2;
                }
            break;

            case 10:
                if (cam_x_cur_angle > 0) {
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CAM_X_CHANNEL, cam_x_cur_angle - 2);
                cam_x_cur_angle -= 2;
                }
            break;
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void read_distance_task(void* param) {
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();
    uint8_t distance_buffer[2];
    while (true) {
        if (esp_websocket_client_is_connected(uno_command_client)) { 
        received_data_from_uno(distance_buffer, UNO, 2);
        std::string distance_string = "di";
        distance_string.append(std::to_string(distance_buffer[0]) + "b");
        distance_string.append(std::to_string(distance_buffer[1]) + "f");
        esp_websocket_client_send_text(uno_command_client, distance_string.c_str(), distance_string.size(), 10000 / portTICK_PERIOD_MS);
        }
        // this function will trigger in a 500ms interval
        
        xTaskDelayUntil(&last_wake_time, 500 / portTICK_PERIOD_MS);
    }
}

void audio_task(void* param) {
    
    while(true) {
        if (esp_websocket_client_is_connected(audio_client)) {
            stream_to_server(audio_client);
        }

    }
}

void lcd_task(void* param) {
    int current_max_position = 0;
    TickType_t last_wake_time;
    last_wake_time = xTaskGetTickCount();
    while (true) {
        if (lcd_activated) {
            int increase_line_1 = 0, increase_line_2 = 0;
            for (int i = 0; i < 16; i++) {
                line_1_lcd[i] = ' ';
                line_2_lcd[i] = ' ';

            }
            LCD_clearScreen();
            for (int i = current_max_position; i < current_max_position + 32 && i < temp_lcd_string.length(); i++) {
                    if (increase_line_1 < 16) {
                        line_1_lcd[increase_line_1] = temp_lcd_string[i];
                        increase_line_1++;
                    }
                    else {
                        line_2_lcd[increase_line_2] = temp_lcd_string[i];
                        increase_line_2++;
                    }
            }

            LCD_setCursor(0, 0);
            LCD_writeStr(line_1_lcd);
            LCD_setCursor(0, 1);
            LCD_writeStr(line_2_lcd);
            current_max_position += 32;
            if (current_max_position >= temp_lcd_string.length()) {
                current_max_position = 0;
                lcd_activated = false;
            }
            
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        else {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

// Event handler for this system
// Each received message will have a trailing character, sorting the command into its correct category
void event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
    // Arg is the parameter passed to the handler when we register a new event
    if (base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI("WIFI", "CONNECTING...");
        }

        else if (event_id == WIFI_EVENT_STA_CONNECTED) {
           // set_static_ip(IP, NETMASK, GATEWAY, (esp_netif_t*) arg);
            ESP_LOGI("WIFI", "STATIC IP SET");
        }    

        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            gpio_set_level(GPIO_NUM_2, 0);
            if (wifi_reconnecting_num < get_wifi_setting_recon_num()) {
                esp_wifi_connect();
                wifi_reconnecting_num++;
                ESP_LOGI(WIFI_TAG, "Reconnecting to AP");
            }

            else {
                xEventGroupSetBits((EventGroupHandle_t) arg, WIFI_FAIL_BIT);
            }
        }
    }
    // Green is Ground, Gray is 5V
    else if (base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* ip = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(WIFI_TAG, "Got IP:" IPSTR, IP2STR(&ip->ip_info.ip));
            wifi_reconnecting_num = 0;
            xEventGroupSetBits((EventGroupHandle_t) arg, WIFI_CONNECTED_BIT);
            gpio_set_level(GPIO_NUM_2, 1);
        }
    }

     else if (base == WEBSOCKET_EVENTS) {
        if (event_id == WEBSOCKET_EVENT_DATA && (char*)arg == (char*) 'c') {
            esp_websocket_event_data_t* received_command = (esp_websocket_event_data_t*) event_data;
            std::string temp (received_command->data_ptr, received_command->data_len);
            // The UNO command client handler will orchestrate the control flow of the chip
            switch (temp[temp.size() - 1]) {
                case 'c': {
                    temp.pop_back();
                    uint8_t a = command[temp] & 0xFF;
                    //ESP_LOGI("Data websocket", "Received message %d", a);
                    if (a != 7 && a != 8 && a != 9 && a != 10) {
                        if (a == 0) {
                            cam_axis_command = a;
                            gpio_set_level(STEPPER_DIR, 0);
                        }

                        if (a == 11) {
                            gpio_set_level(STEPPER_DIR, 0);
                        }
                        else if (a == 12) {
                            gpio_set_level(STEPPER_DIR, 1);
                        }

                    send_command_to_uno(a, UNO, 1); // Remember to change the address back to Uno
                    }

                    else if (a == 7 || a == 8 || a == 9 || a == 10) {
                        cam_axis_command = a;
                    }
            
                break;
                }
                case 'l': {
                    // Update LCD display
                    temp.pop_back();
                    temp_lcd_string = temp;
                    lcd_activated = true;
                break;
                }
                case 'u': {
                    if ((command[temp] & 0xFF) == 13) {
                        vTaskSuspend(audio_task_handle);
                    }
                    else if ((command[temp] & 0xFF) == 14) {
                        vTaskResume(audio_task_handle);
                    }
                    else if ((command[temp] & 0xFF) == 15) {
                        gpio_set_level(GPIO_NUM_23, 1);
                    }
                    else if ((command[temp] & 0xFF) == 16) {
                        gpio_set_level(GPIO_NUM_23, 0);
                    }
                break;    
                }
            }
        }

        else if (event_id == WEBSOCKET_EVENT_DISCONNECTED) {
            ESP_LOGI("Web", "client disconnected");
            send_command_to_uno((0 & 0xFF), UNO, 1);
        }

        else if (event_id == WEBSOCKET_EVENT_CONNECTED) {
            ESP_LOGI("Web", "client connected");
            esp_websocket_client_send_bin(uno_command_client, "esp", 3, 10000/portTICK_PERIOD_MS);
        }
     }
}

void audio_client_event(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
    if (base == WEBSOCKET_EVENTS) {
        if (event_id == WEBSOCKET_EVENT_DATA) {
            esp_websocket_event_data_t* received_buffer = (esp_websocket_event_data_t*) event_data;
            //ESP_LOGI("audio", "len: %d", received_buffer->data_len);
            output_to_speaker(received_buffer->data_ptr, received_buffer->data_len); 
            received_buffer->data_ptr = NULL;
            //sprintf(temp_audio_buffer, received_buffer->data_ptr);
            // for (int i = 0; i < 5; i++)
            // printf("%d", *(received_buffer->data_ptr + i));
        }
    }
}
// Init sequence: GPIO -> I2C -> Servo -> Mic -> Wifi -> Web Socket 

//#define WRITE_TO_FLASH
extern "C" void app_main() {

    esp_err_t ret = nvs_flash_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    nvs_handle_t handle_flash;
    nvs_open("storage", NVS_READWRITE, &handle_flash);

    #ifdef WRITE_TO_FLASH

    nvs_set_str(handle_flash, "OPTUS_4AFB46M", "aliya67945du");
    nvs_commit(handle_flash);
    nvs_set_str(handle_flash, "haha", "123456789");
    nvs_commit(handle_flash);
    nvs_set_str(handle_flash, "Guests network", "thisisaguestnetwork");
    nvs_commit(handle_flash);
    nvs_set_str(handle_flash, "Miyagi", "ln21157003");
    nvs_commit(handle_flash);
    nvs_set_str(handle_flash, "UTS-WiFi", "I14363978Pln31042003!");
    nvs_commit(handle_flash);
    nvs_set_str(handle_flash, "iPhone 13ProMax Ha", "helloeveryone");
    nvs_commit(handle_flash);

    #endif

    // This should be put at the beginning
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    size_t required_size = 0;
    // nvs_set_str(handle_flash, "test", "successful");
    // nvs_commit(handle_flash);

    // nvs_get_str(handle_flash, "OPTUS-4AFB46M", NULL, &required_size);
    // char* val = (char*) malloc(required_size);
    // nvs_get_str(handle_flash, "OPTUS-4AFB46M", val, &required_size);
    // ESP_LOGI("Test","Data read is: %s \n", val);
    // free(val);

    // Init GPIO
    init_gpio();
    ESP_LOGI("GPIO", "GPIO INIT DONE");

    // Init I2C
    I2C_clearBus(SDA, SCL);
    I2C_clearBus(SDA_LCD, SCL_LCD);
    ESP_LOGI("I2C", "Clear Bus Done");
    init_i2c();
    ESP_LOGI("I2C", " INIT DONE");
    
    //Init LCD.
    LCD_init(LCD, 16, 2);
    ESP_LOGI("LCD", "INIT DONE");
    LCD_home();
    LCD_clearScreen();
    LCD_writeStr("Hello humanity");
    LCD_setCursor(0, 1);
    LCD_writeStr("my name is Seph");

    // Init Servo. If no pulse is detected, try changing group id in servo config
    camera_axis_init(CAM_X, CAM_Y, CAM_X_CHANNEL, CAM_Y_CHANNEL);
    ESP_LOGI("Servo", "INIT DONE");

    //Init Wifi
    
    connect_to_wifi(event_handler, handle_flash);

    xTaskCreatePinnedToCore(read_distance_task, "Sonar sensor", 2048, NULL, 1, &sonar_sensor_task, 1);
    ESP_LOGI("Sonar sensor", "Task created");

    xTaskCreatePinnedToCore(servo_task, "Camera x y", 1024, NULL, 2, &camera_task, 1);
    ESP_LOGI("Servo", "Servo task created");

    xTaskCreatePinnedToCore(lcd_task, "LCD", 1024, NULL, 2, &lcd_task_handle, 1);

    // Init I2S
    init_i2s();
    ESP_LOGI("I2S", " INIT DONE");

    xTaskCreatePinnedToCore(audio_task, "audio task", 2048, NULL, 1, &audio_task_handle, 0);
    vTaskSuspend(audio_task_handle);

    // Establishing connection between esp32 and node js server
    audio_client = esp_websocket_client_init(&audio_client_conf);
    uno_command_client = esp_websocket_client_init(&uno_command_client_conf);
    
    // WARNING: I feel like we cannot differentiate between messages using the handler arg

    esp_websocket_register_events(audio_client, WEBSOCKET_EVENT_ANY, audio_client_event, (char*) 'a');
    esp_websocket_register_events(uno_command_client, WEBSOCKET_EVENT_ANY, event_handler, (char*) 'c');

    esp_websocket_client_start(uno_command_client);
    esp_websocket_client_start(audio_client);

}
