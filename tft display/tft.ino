#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include "esp_websocket_client.h"
#include "freertos/ringbuf.h"

#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

TFT_eSPI tft = TFT_eSPI(240, 320);

// const char* ssid = "UTS-WiFi";
// #define ID "14363978"
// #define USERNAME "14363978"
// #define PASSWORD "Ln31042003!"

const char* ssid = "TP-Link_C7E2";
const char* pass = "33247489";
int a = 0;

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

//====================================================================================
void setup(void)
{
  delay(25000);
  Serial.begin(115200);
  imgBuf = xRingbufferCreateNoSplit(4500, 5);
    if (imgBuf == NULL) {
      Serial.println("ring buffer init failed");
    }
  pinMode(2, OUTPUT);
  Serial.println(ESP.getCpuFreqMHz());
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  TJpgDec.setJpgScale(1);

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
    TJpgDec.drawJpg(0, 0, (const uint8_t*)(item), item_size);
    vRingbufferReturnItem(imgBuf, (void*) item);
  }
  else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
