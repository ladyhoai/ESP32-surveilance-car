#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>
#include "esp_wpa2.h"
#include "esp_wifi.h"
#include "esp_websocket_client.h"

#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

TFT_eSPI tft = TFT_eSPI(240, 320);

const char* ssid = "Beverly Hills";
const char* pass = "Baolinh2209@#$";
int a = 0;


esp_websocket_client_handle_t image_port;
const esp_websocket_client_config_t image_port_conf = {
  .uri = "ws://192.168.1.24",
  .port = 4000,
  .disable_auto_reconnect = false,
  .buffer_size = 10000,
};

void event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data) {
  if (base == WEBSOCKET_EVENTS) {
    if (event_id == WEBSOCKET_EVENT_DATA) {
      esp_websocket_event_data_t* received_img_buf = (esp_websocket_event_data_t*) event_data;      
      uint16_t w = 0, h = 0;
     // Serial.println(received_img_buf->payload_len);
       TJpgDec.getJpgSize(&w, &h, (const uint8_t*)(received_img_buf->data_ptr), received_img_buf->data_len);
       TJpgDec.drawJpg(0, 0, (const uint8_t*)(received_img_buf->data_ptr), received_img_buf->data_len);
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
  
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  Serial.println(ESP.getCpuFreqMHz());
  tft.init();
  tft.setRotation(3);
  tft.setSwapBytes(true);
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
  //vTaskDelay(1);
}
