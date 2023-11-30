#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include "esp_websocket_client.h"
#include "freertos/ringbuf.h"
#include "img.h"

#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

TFT_eSPI tft = TFT_eSPI(240, 320);
TFT_eSprite eye_look_left = TFT_eSprite(&tft);
TFT_eSprite eye_look_right = TFT_eSprite(&tft);

TFT_eSprite loading_bar = TFT_eSprite(&tft);
TFT_eSprite loading_bar_section1 = TFT_eSprite(&tft);
TFT_eSprite loading_bar_section2 = TFT_eSprite(&tft);
TFT_eSprite loading_bar_section3 = TFT_eSprite(&tft);
TFT_eSprite loading_bar_section4 = TFT_eSprite(&tft);

TFT_eSprite globe = TFT_eSprite(&tft);
TFT_eSprite cam = TFT_eSprite(&tft);
TFT_eSprite microphone = TFT_eSprite(&tft);

// const char* ssid = "UTS-WiFi";
// #define ID "14363978"
// #define USERNAME "14363978"
// #define PASSWORD "Ln31042003!"

const char* ssid = "TP-Link_C7E2";
const char* pass = "33247489";
int a = 0;
bool globe_on_screen = false;

RingbufHandle_t imgBuf;

esp_websocket_client_handle_t image_port;
const esp_websocket_client_config_t image_port_conf = {
  .uri = "ws://192.168.1.100",
  .port = 4004,
  .disable_auto_reconnect = false,
  .buffer_size = 4000,
};

void event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
  if (base == WEBSOCKET_EVENTS) {
    if (event_id == WEBSOCKET_EVENT_DATA) {
      esp_websocket_event_data_t* received_img_buf = (esp_websocket_event_data_t*) event_data;   
      xRingbufferSend(imgBuf, received_img_buf->data_ptr, received_img_buf->data_len, 1000 / portTICK_PERIOD_MS);
    }

    else if (event_id == WEBSOCKET_EVENT_CONNECTED) {
      if (!globe_on_screen) {
        globe_on_screen = true;
        globe.pushSprite(286, 7);
      } 
    }

    else if (event_id == WEBSOCKET_EVENT_DISCONNECTED) {
      if (globe_on_screen) {
        globe_on_screen = false;
        globe.fillSprite(TFT_BLACK);
      }
    }
  }
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // This might work instead if you adapt the sketch to use the Adafruit_GFX library
  // tft.drawRGBBitmap(x, y, bitmap, w, h);

  // Return 1 to decode next block
  return 1;
}

void arrange_init_screen() {
  loading_bar.createSprite(134, 20);
  loading_bar.setSwapBytes(true);
  loading_bar.pushImage(0, 0, 134, 20, bar);
  loading_bar.pushSprite(95, 117);

  cam.createSprite(41, 35);
  cam.setSwapBytes(true);
  cam.pushImage(0, 0, 41, 35, camera);
  cam.pushSprite(31, 185);

  microphone.createSprite(22, 34);
  microphone.setSwapBytes(true);
  microphone.pushImage(0, 0, 22, 34, mic);
  microphone.pushSprite(154, 65);

  globe.createSprite(29, 29);
  globe.setSwapBytes(true);
  globe.pushImage(0, 0, 29, 29, earth);
}

void eye_handler() {
  eye_look_left.createSprite(60, 36);
  eye_look_left.setSwapBytes(true);
  eye_look_left.pushImage(0, 0, 54, 33, left_eye_closed);
  eye_look_left.pushSprite(81, 17);
  
  eye_look_right.createSprite(60, 36);
  eye_look_right.setSwapBytes(true);
  eye_look_right.pushImage(0, 0,  60, 32, right_eye_closed);
  eye_look_right.pushSprite(184, 17);
  
  delay(16600);
  eye_look_left.fillSprite(TFT_BLACK);
  eye_look_right.fillSprite(TFT_BLACK);
  eye_look_left.pushImage(0, 0,  54, 33, left_eye_haft_open);
  eye_look_left.pushSprite(81, 17);
  eye_look_right.pushImage(0, 0,  54, 33, right_eye_half_open);
  eye_look_right.pushSprite(184, 17);

  delay(8300);
  eye_look_left.fillSprite(TFT_BLACK);
  eye_look_right.fillSprite(TFT_BLACK);
  eye_look_left.pushImage(0, 0,  59, 36, lefteye_look_upleft);
  eye_look_left.pushSprite(81, 17);
  eye_look_right.pushImage(0, 0,  59, 36, lefteye_look_upleft);
  eye_look_right.pushSprite(184, 17);

}

void loading_bar_task(void*) {
  while (true) {
    loading_bar_section1.createSprite(19, 12);
    loading_bar_section2.createSprite(19, 12);
    loading_bar_section3.createSprite(19, 12);
    loading_bar_section4.createSprite(19, 12);

    loading_bar_section1.setSwapBytes(true);
    loading_bar_section2.setSwapBytes(true);
    loading_bar_section3.setSwapBytes(true);
    loading_bar_section4.setSwapBytes(true);

    loading_bar_section1.pushImage(0, 0, 19, 12, loading_bar_section);
    loading_bar_section2.pushImage(0, 0, 19, 12, loading_bar_section);
    loading_bar_section3.pushImage(0, 0, 19, 12, loading_bar_section);
    loading_bar_section4.pushImage(0, 0, 19, 12, loading_bar_section);

    vTaskDelay(6225 / portTICK_PERIOD_MS);
    loading_bar_section1.pushSprite(120, 121);
    vTaskDelay(6225 / portTICK_PERIOD_MS);
    loading_bar_section2.pushSprite(141, 121);
    vTaskDelay(6225 / portTICK_PERIOD_MS);
    loading_bar_section3.pushSprite(162, 121);
    vTaskDelay(6225 / portTICK_PERIOD_MS);
    loading_bar_section4.pushSprite(183, 121);
    vTaskDelete(NULL);
  }
}

//====================================================================================
void setup(void)
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  arrange_init_screen();
  xTaskCreate(loading_bar_task, "Loading bar task", 2048, NULL, 1, NULL);
  eye_handler();

  delay(25000);
  Serial.begin(115200);
  imgBuf = xRingbufferCreateNoSplit(4500, 5);
    if (imgBuf == NULL) {
      Serial.println("ring buffer init failed");
    }
  pinMode(2, OUTPUT);
  Serial.println(ESP.getCpuFreqMHz());
  
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  // The decoder must be given the exact name of the rendering function above
  TJpgDec.setCallback(tft_output);

  WiFi.disconnect(true);

  WiFi.mode(WIFI_STA);

  // The code below is for UTS authentication
  // esp_wifi_set_mode(WIFI_MODE_STA);
  // esp_wifi_sta_wpa2_ent_set_identity((uint8_t*) ID, strlen(ID));
  // esp_wifi_sta_wpa2_ent_set_username((uint8_t*) USERNAME, strlen(USERNAME));
  // esp_wifi_sta_wpa2_ent_set_password((uint8_t*) PASSWORD, strlen(PASSWORD));
  // esp_wifi_sta_wpa2_ent_enable();
  // WiFi.begin(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println('.');
  }
  digitalWrite(2, HIGH);
  Serial.println(WiFi.localIP());

  image_port = esp_websocket_client_init(&image_port_conf);
  esp_websocket_register_events(image_port, WEBSOCKET_EVENT_ANY, event_handler, NULL);
  esp_websocket_client_start(image_port);

}
 
void loop() {

  size_t item_size;
  char* item = (char*) xRingbufferReceive(imgBuf, &item_size, 300 / portTICK_PERIOD_MS);
  if (item != NULL) {
    uint16_t w = 0, h = 0;
    TJpgDec.getJpgSize(&w, &h, (const uint8_t*)(item), item_size);
    TJpgDec.drawJpg(0, 163, (const uint8_t*)(item), item_size);
    
    vRingbufferReturnItem(imgBuf, (void*) item);
  }
  else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}