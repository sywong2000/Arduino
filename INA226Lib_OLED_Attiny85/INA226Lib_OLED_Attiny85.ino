#include <Tiny4kOLED.h>
#include "font8x16atari.h"
#include "homespun_font.h"
#include <INA226.h>
const DCfont *largeFont = FONT8X16ATARI;
const DCfont *smallFont = FONTHOMESPUN;
INA226 ina;

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
  ina.begin();  
  ina.calibrate(0.1, 10);
}

void loop() {
  float c = ina.readShuntCurrent();
    oled.setFont(largeFont);
    oled.setCursor(0, 0);
    oled.print(c);
  oled.switchFrame();
  delay(150);
}
