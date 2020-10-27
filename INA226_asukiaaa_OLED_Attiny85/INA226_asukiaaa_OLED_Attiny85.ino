#include <Tiny4kOLED.h>
#include "font8x16atari.h"
#include "homespun_font.h"
#include <INA226_asukiaaa.h>
const DCfont *largeFont = FONT8X16ATARI;
const DCfont *smallFont = FONTHOMESPUN;
INA226_asukiaaa voltCurrMeter(INA226_ASUKIAAA_ADDR_A0_GND_A1_GND, 51); //INA226_asukiaaa::calcCalibByResisterMilliOhm(100); //0.1 ohm = 5120/100 = 51
int16_t ma, mv, mw;
unsigned long starttime = 0;
boolean b;
int n =0;
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define Watt(ma, mv) (ma * mv)

void setup() {
  // put your setup code here, to run once:
  oled.begin();
  oled.on();
  oled.clear();
  oled.switchRenderFrame();
  oled.clear();
  n=voltCurrMeter.begin();
  while (n!=0)
  {
    n=voltCurrMeter.begin();
    oled.setFont(largeFont);
    oled.setCursor(32, 1);
    oled.print(F("INA FAULT:"));
    oled.print(n);
    oled.switchFrame();
    delay(3000);
  }
  
}

void loop() {
  voltCurrMeter.readMA(&ma);
  voltCurrMeter.readMV(&mv);
  voltCurrMeter.readMW(&mw);
  b = millis()/1000U%2==1;
  oled.clear();
  if (ma< 10)
  {
    oled.setFont(largeFont);
    oled.setCursor(32, 1);
    oled.setInverse(b?true:false);
    oled.print(F("NO LOAD"));
    starttime = millis();
  }
  else
  {
    oled.setInverse(false);
    oled.setFont(largeFont);
    oled.setCursor(0, 0);
    oled.print(mv/1000U);
    oled.print('.');
    oled.print((mv / 100U) % 10);
    oled.print(F("v  "));
    oled.print(ma/1000U);
    oled.print('.');
    oled.print((ma / 100U) % 10);
    oled.print((ma / 10U) % 10);
    oled.print(b?F("A"):F("A ."));
  
   
    oled.setCursor(0, 3);
    oled.setFont(smallFont);
    oled.print(mw/1000U);
    oled.print('.');
    oled.print((mw / 100U) % 10);
    oled.print(F("W     "));
    oled.print(numberOfHours((millis()-starttime)/1000));
    oled.print(numberOfMinutes((millis()-starttime)/1000)<10?F(":0"):F(":"));
    oled.print(numberOfMinutes((millis()-starttime)/1000));
    oled.print(numberOfSeconds((millis()-starttime)/1000)<10?F(":0"):F(":"));
    oled.print(numberOfSeconds((millis()-starttime)/1000));
  }
  oled.switchFrame();
  delay(150);
}
