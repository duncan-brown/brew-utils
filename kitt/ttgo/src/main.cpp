#include <Arduino.h>

#include <TFT_eSPI.h> 
#include <SPI.h>
#include "WiFi.h"
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define ADC_EN          14
#define ADC_PIN         34
#define BUTTON_1        35
#define BUTTON_2        0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms) //use-> espDelay(6000);
{   
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

void setup()
{
    tft.init();
    tft.setRotation(1);
    tft.setTextSize(2);


    tft.setTextDatum(MC_DATUM);
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_BLACK, TFT_RED);
    tft.drawString("POWER", tft.width()/2, tft.height()/2, 4);
    sleep(1);
    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    tft.drawString("MIN RPM", tft.width()/2, tft.height()/2, 4);
    sleep(1);
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString("FUEL ON", tft.width()/2, tft.height()/2, 4);
    sleep(1);
    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString("IGNITORS", tft.width()/2, tft.height()/2, 4);
    sleep(1);

    tft.setTextSize(1);
    tft.fontHeight(4);
    tft.setTextDatum(TL_DATUM);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString("AIR", 0, 0, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    tft.drawString("S1", 3.4*tft.width()/4, 0, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    sleep(1);
    tft.drawString("OIL", 0, tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    tft.drawString("S2", 3.4*tft.width()/4, tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    sleep(1);
    tft.setTextColor(TFT_BLACK, TFT_RED);
    tft.drawString("P1", 0, 2 * tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    tft.drawString("P3", 3.4*tft.width()/4, 2* tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    sleep(1);
    tft.drawString("P2", 0, 3 * tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
    tft.drawString("P4", 3.4*tft.width()/4, 3 * tft.height() / 4, 4);  //string,start x,start y, font weight {1;2;4;6;7;8}
}

void loop()
{

}