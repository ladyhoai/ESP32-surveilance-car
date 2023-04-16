#include <TFT_eSPI.h>
#include <SPI.h>
#include <JPEGDecoder.h>
TFT_eSPI tft = TFT_eSPI();
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))
//====================================================================================
void setup(void)
{
  tft.init();
  tft.setRotation(1);
  //Arduino_ESP32SPI bus = Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO);
  //Arduino_ESP32PAR8 display = Arduino_ESP32PAR8(DC, CS, WR, RD, D0, D1, D2, D3, D4, D5, D6, D7);
 
  // display.begin();
  // display.fillScreen(BLACK);
  // display.setCursor(10, 10);
  // display.setTextSize(5);
  // display.setTextColor(BLUE);
  // display.print("Hello world");
}
 
void loop() {
  tft.fillScreen(TFT_WHITE);
 
for (int i = 0; i < 40; i++)
  {
    int rx = random(60);
    int ry = random(60);
    int x = rx + random(320 - rx - rx);
    int y = ry + random(240 - ry - ry);
    tft.fillEllipse(x, y, rx, ry, random(0xFFFF));
  }

  delay(2000);
  tft.fillScreen(TFT_BLACK);

  for (int i = 0; i < 40; i++)
  {
    int rx = random(60);
    int ry = random(60);
    int x = rx + random(320 - rx - rx);
    int y = ry + random(240 - ry - ry);
    tft.drawEllipse(x, y, rx, ry, random(0xFFFF));
  }

  delay(2000);
}
