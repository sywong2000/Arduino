#include <U8g2lib.h>
// #include <Wire.h>

#define ENABLE_PIN       A3    
#define STEP_PIN         A1
#define DIR_PIN          A0


//# IIC version
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0,6,5,U8X8_PIN_NONE); //ESP32C3 OLED开发板的屏幕接线：scl-->gpio6  sda-->gpio5 如果是Arduino则改为scl-->A5  sda-->A4
int last = millis();
int count = 0;
void setup(void) {
 u8g2.setContrast(300);
 u8g2.begin();
 Serial0.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
}



void loop(void) {
//  u8g2.firstPage();
//  do {
//   u8g2.setFont(u8g2_font_ncenB10_tr);
//   u8g2.drawStr(30,24,"TEST");
//  } while ( u8g2.nextPage() );

  if (millis()-last > 500)
  { 
    count++;
    last = millis();
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  String s = "TEST " + String(count);
  u8g2.drawStr(30,24,s.c_str());
  u8g2.sendBuffer();
}
