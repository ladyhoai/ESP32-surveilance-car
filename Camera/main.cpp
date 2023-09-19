#include <string>
// /*

//   This is a simple MJPEG streaming webserver implemented for AI-Thinker ESP32-CAM and
//   ESP32-EYE modules.
//   This is tested to work with VLC and Blynk video widget.

//   Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
//   (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

//   Board: AI-Thinker ESP32-CAM

// */
#define APP_CPU 1
#define PRO_CPU 0
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#include <OV2640.h>
#include <WiFi.h>
//#include <WebServer.h>
//#include <WiFiClient.h>

#include <esp_bt.h>
#include <esp_wifi.h>
#include "esp_wpa2.h"
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <driver/i2s.h>
#include <esp_websocket.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"


#define SSID "TP-Link_C7E2"
#define ID "14363978"
#define PASS "33247489"

esp_websocket_client_handle_t client;
const esp_websocket_client_config_t client_conf = {
  .uri = "ws://34.87.246.254",
  .port = 4003,
  .disable_auto_reconnect = false,
  .buffer_size = 5000
};

static uint8_t wifi_reconnecting_num = 0;

OV2640 cam;
// Set your Static IP address
IPAddress local_IP(172, 19, 119, 11);
// Set your Gateway IP address
IPAddress gateway(172, 19, 124, 1);

IPAddress subnet(255, 255, 254, 0);
//WebServer server(80);

// ===== rtos task handles =========================
// Streaming is implemented with 3 tasks:
TaskHandle_t tMjpeg;   // handles client connections to the webserver
TaskHandle_t tCam;     // handles getting picture frames from the camera and storing them locally
TaskHandle_t tStream;  // actually streaming frames to all connected clients

// frameSync semaphore is used to prevent streaming buffer as it is replaced with the next frame
SemaphoreHandle_t frameSync = NULL;

// Queue stores currently connected clients to whom we are streaming
QueueHandle_t streamingClients;

// We will try to achieve 25 FPS frame rate
const int FPS = 25;

// We will handle web client requests every 50 ms (20 Hz)
const int WSINTERVAL = 100;


// ======== Server Connection Handler Task ==========================


// Commonly used variables:
volatile size_t camSize;    // size of the current frame, byte
volatile char* camBuf;      // pointer to the current frame

char* allocateMemory(char* aPtr, size_t aSize) {

  //  Since current buffer is too smal, free it
  if (aPtr != NULL) free(aPtr);


  size_t freeHeap = ESP.getFreeHeap();
  char* ptr = NULL;

  // If memory requested is more than 2/3 of the currently free heap, try PSRAM immediately
  if ( aSize > freeHeap * 2 / 3 ) {
    if ( psramFound() && ESP.getFreePsram() > aSize ) {
      ptr = (char*) ps_malloc(aSize);
    }
  }
  else {
    //  Enough free heap - let's try allocating fast RAM as a buffer
    ptr = (char*) malloc(aSize);

    //  If allocation on the heap failed, let's give PSRAM one more chance:
    if ( ptr == NULL && psramFound() && ESP.getFreePsram() > aSize) {
      ptr = (char*) ps_malloc(aSize);
    }
  }

  // Finally, if the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL) {
    ESP.restart();
  }
  return ptr;
}

// ==== RTOS task to grab frames from the camera =========================
void camCB(void* pvParameters) {

  TickType_t xLastWakeTime;

  //  A running interval associated with currently desired frame rate
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);

  // Mutex for the critical section of swithing the active frames around
  portMUX_TYPE xSemaphore = portMUX_INITIALIZER_UNLOCKED;

  //  Pointers to the 2 frames, their respective sizes and index of the current frame
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    //  Grab a frame from the camera and query its size
    cam.run();
    size_t s = cam.getSize();

    //  If frame size is more that we have previously allocated - request  125% of the current frame space
    if (s > fSize[ifb]) {
      fSize[ifb] = s * 4 / 3;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }

    //  Copy current frame into local buffer
    char* b = (char*) cam.getfb();
    memcpy(fbs[ifb], b, s);

    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //  Only switch frames around if no frame is currently being streamed to a client
    //  Wait on a semaphore until client operation completes
    xSemaphoreTake( frameSync, portMAX_DELAY );

    //  Do not allow interrupts while switching the current frame
    portENTER_CRITICAL(&xSemaphore);
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;  // this should produce 1, 0, 1, 0, 1 ... sequence
    portEXIT_CRITICAL(&xSemaphore);

    //  Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive( frameSync );

    //  Technically only needed once: let the streaming task know that we have at least one frame
    //  and it could start sending frames to the clients, if any
    xTaskNotifyGive( tStream );

    //  Immediately let other (streaming) tasks run
    taskYIELD();

    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera. We can save some juice
    //  by suspedning the tasks
    if ( eTaskGetState( tStream ) == eSuspended ) {
      vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }
  }
}

// ==== Actually stream content to all connected clients ========================
void streamCB(void * pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  //  Wait until the first frame is captured and there is something to send
  //  to clients
  ulTaskNotifyTake( pdTRUE,          /* Clear the notification value before exiting. */
                    portMAX_DELAY ); /* Block indefinitely. */

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Default assumption we are running according to the FPS
    xFrequency = pdMS_TO_TICKS(1000 / FPS);

    //  Only bother to send anything if there is someone watching

        //  Ok. This is an actively connected client.
        //  Let's grab a semaphore to prevent frame changes while we
        //  are serving this frame
        xSemaphoreTake( frameSync, portMAX_DELAY );

        esp_websocket_client_send_bin(client, (char*) camBuf, (size_t) camSize, portMAX_DELAY);
        //Serial.println("send");

        //  The frame has been served. Release the semaphore and let other tasks run.
        //  If there is a frame switch ready, it will happen now in between frames
        xSemaphoreGive( frameSync );
        taskYIELD();
    
    //  Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, 40 / portTICK_PERIOD_MS);
  }
}

void handle_data(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
  if (base == WEBSOCKET_EVENTS) {
    if (event_id == WEBSOCKET_EVENT_CONNECTED) {
      esp_websocket_client_send_bin(client, "cam", 3, 1000/portTICK_PERIOD_MS);
      Serial.println("Connected to server");
    }

    else if (event_id == WEBSOCKET_EVENT_DISCONNECTED) {
      Serial.println("Disconnected from server");
      ESP.restart();
    }

    else if (event_id == WEBSOCKET_EVENT_DATA) {
      esp_websocket_event_data_t* received_command = (esp_websocket_event_data_t*) event_data;
      std::string temp (received_command->data_ptr, received_command->data_len);
      //Serial.println("hi");
      switch (temp[temp.size() - 1]) {
        case 'q': {
          temp.pop_back();
          int val = stoi(temp);
          sensor_t* config = esp_camera_sensor_get();
          config->set_quality(config, val);
          break;
        }     

        case 'f': {
          
          break;
        }
      }
    }
  }
}

void mjpegCB(void* pvParameters) {

  // for (;;) {
  //   cam.run();
  //   size_t s = cam.getSize();
  //   Serial.println(s);
  //   vTaskDelay(1000);
  // }
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creating frame synchronization semaphore and initializing it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive( frameSync );

  client = esp_websocket_client_init(&client_conf);
  esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, handle_data, NULL);
  esp_websocket_client_start(client);

  //=== setup section  ==================

  //  Creating RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
    camCB,        // callback
    "cam",        // name
    4096,         // stacj size
    NULL,         // parameters
    2,            // priority
    &tCam,        // RTOS task handle
    APP_CPU);     // core

  //  Creating task to push the stream to all connected clients
  xTaskCreatePinnedToCore(
    streamCB,
    "strmCB",
    4 * 1024,
    NULL, //(void*) handler,
    2,
    &tStream,
    APP_CPU);

  //  Registering webserver handling routines
  //server.on("/mjpeg/1", HTTP_GET, handleJPGSstream);
  //server.on("/jpg", HTTP_GET, handleJPG);
  //server.onNotFound(handleNotFound);

  //  Starting webserver
  //server.begin();

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // server.handleClient();

    //  After every server client handling request, we let other tasks run and then pause
    taskYIELD();
    // vTaskDelayUntil(&xLastWakeTime, 40 / portTICK_PERIOD_MS);
  }
}

void event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
  if (base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();          
            ESP_LOGI("WIFI", "CONNECTING...");
        }

        else if (event_id == WIFI_EVENT_STA_CONNECTED) {
           // set_static_ip(IP, NETMASK, GATEWAY, (esp_netif_t*) arg);
            ESP_LOGI("WIFI", "STATIC IP SET");
        }    

        else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            gpio_set_level(GPIO_NUM_2, 0);
            if (wifi_reconnecting_num < 10) {
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
    
    

    // UTS Authentication
    // esp_wifi_sta_wpa2_ent_set_identity((uint8_t*) ID, strlen(ID));
    // esp_wifi_sta_wpa2_ent_set_username((uint8_t*) USERNAME, strlen(USERNAME));
    // esp_wifi_sta_wpa2_ent_set_password((uint8_t*) PASSWORD, strlen(PASSWORD));
    // esp_wifi_sta_wpa2_ent_enable();
    wifi_config_t wifi_conf;
            // .sta {
            //     .ssid = SSID,
            //     .password = PASS,
            // }
        
    strcpy((char*)wifi_conf.sta.ssid, (const char*) SSID);
    strcpy((char*)wifi_conf.sta.password , PASS);
    // Uncomment the line below to use Michael's network
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_conf));
    ESP_ERROR_CHECK(esp_wifi_start());

    //     wifi_scan_config_t scan = {
    //         .ssid = 0,
    //         .bssid = 0,
    //         .channel = 0,
    //         .show_hidden = true
    //         };

    //         ESP_ERROR_CHECK(esp_wifi_scan_start(&scan, true));
    //         uint16_t number = 5;
    //         wifi_ap_record_t record[number];
    //         memset(record, 0, sizeof(record));
    //         uint16_t ap_count = 0;
    //         ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, record));  
    //         ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    //         ESP_LOGI("WiFi", "Total APs scanned = %u", ap_count);  
            
    //         for (int i = 0; i < ap_count; i++) {
    //             size_t required_size = 0;
    //             ESP_LOGI("Wifi","Signal strength is: %d \n", record[i].rssi);
    //             esp_err_t check = nvs_get_str(handle_flash, (const char*)record[i].ssid, NULL, &required_size);
                
    //             if (check == ESP_OK) {
    //                 char* val = (char*) malloc(required_size);
    //                 nvs_get_str(handle_flash, (const char*)record[i].ssid, val, &required_size);
    //                 ESP_LOGI("Wifi","Password is: %s \n", val);
                    
    //                 static wifi_config_t wifi_conf;

    //                 strcpy((char*)wifi_conf.sta.ssid, (const char*) record[i].ssid);
    //                 strcpy((char*)wifi_conf.sta.password , val);

    //                 esp_wifi_set_config(WIFI_IF_STA, &wifi_conf);
                    
    //                 free(val);

    //                 esp_wifi_connect();

    //                 break;
    //             }

    //             else if (check == ESP_ERR_NVS_NOT_FOUND) {
    //                 ESP_LOGI("WiFi", "Password is not yet stored for this SSID %s", (const char*)record[i].ssid);
    //                 continue;
    //             }
    //         }


    // // Up till this point, the sta init has finished
    // // Wait bits simply wait for either WIFI_CONNECTED_BIT or WIFI_FAIL_BIT to set, wait time depends on the last param of
    // // the function. Here, portMAX_DELAY means the program will keep waiting til' one of the 2 bits is set

    // // Maybe in the future we can try to make a timeout for this
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

// ==== SETUP method ==================================================================
//#define WRITE_TO_FLASH
void setup()
{

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
    
    nvs_set_str(handle_flash, "UTS-WiFi", "I14363978Pln31042003!");
    nvs_commit(handle_flash);
    
    nvs_set_str(handle_flash, "ZMI_C8D3", "64682812");
    nvs_commit(handle_flash);
#endif
  // Setup Serial connection:
  Serial.begin(115200);
  delay(25000); // wait for a second to let Serial connect


  // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters: pick one
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.frame_size = FRAMESIZE_SVGA;
  config.frame_size = FRAMESIZE_QVGA;
  //config.frame_size = FRAMESIZE_HVGA;
  config.jpeg_quality = 40;
  config.fb_count = 2;
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  pinMode(4, OUTPUT);
  if (cam.init(config) != ESP_OK) {
    Serial.println("Error initializing the camera");
    delay(10000);
    ESP.restart();
  }


  //  Configure and connect to WiFi
  IPAddress ip;
// if (!WiFi.config(local_IP, gateway, subnet)) {
//     Serial.println("STA Failed to configure");
//   }
  

  // esp_wifi_set_mode(WIFI_MODE_STA);
  // esp_wifi_sta_wpa2_ent_set_identity((uint8_t*) ID, strlen(ID));
  // esp_wifi_sta_wpa2_ent_set_username((uint8_t*) ID, strlen(ID));
  // esp_wifi_sta_wpa2_ent_set_password((uint8_t*) PASS, strlen(PASS));
  // esp_wifi_sta_wpa2_ent_enable();

WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  // connect_to_wifi(event_handler, handle_flash);
  // Serial.println("Connecting to WiFi");
  // delay(5000);
  // Serial.print("Stream Link: http://");
  // Serial.print(ip);
  // Serial.println("/mjpeg/1");


  // Start mainstreaming RTOS task
  xTaskCreatePinnedToCore(
    mjpegCB,
    "mjpeg",
    4 * 1024,
    NULL,
    2,
    &tMjpeg,
    APP_CPU);
}


void loop() {
  //Serial.println(WiFi.RSSI());
  // digitalWrite(4, HIGH);
  //vTaskDelay(500);
  
}

