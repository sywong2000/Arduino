#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

boolean noload = true;

void setup(void) {
  Serial.begin(9600);
  Serial.println("Initializing..");
  ina219.begin();
  Serial.println("INA219 (Ammeter) OK");
  u8g2.begin();
  Serial.println("U8G2 (OLED) OK");
  setSSD1306VcomDeselect(0);
  setSSD1306PreChargePeriod(15,1);
}

// value from 0 to 7, higher values more brighter
void setSSD1306VcomDeselect(uint8_t v)
{  
  u8x8_cad_StartTransfer(u8g2.getU8x8());
  u8x8_cad_SendCmd(u8g2.getU8x8(), 0x0db);
  u8x8_cad_SendArg(u8g2.getU8x8(), v << 4);
  u8x8_cad_EndTransfer(u8g2.getU8x8());
}

// p1: 1..15, higher values, more darker, however almost no difference with 3 or more
// p2: 1..15, higher values, more brighter
void setSSD1306PreChargePeriod(uint8_t p1, uint8_t p2)
{ 
  u8x8_cad_StartTransfer(u8g2.getU8x8());
  u8x8_cad_SendCmd(u8g2.getU8x8(), 0x0d9);
  u8x8_cad_SendArg(u8g2.getU8x8(), (p2 << 4) | p1 );
  u8x8_cad_EndTransfer(u8g2.getU8x8());
}

void loop(void) {
  
  float current_mA = 0, shuntvoltage=0, busvoltage=0, power_mW=0, loadvoltage= 0;
  u8g2.clearBuffer();          // clear the internal menory
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  Serial.print(shuntvoltage);
  Serial.print(" ");
  Serial.print(busvoltage);
  Serial.print(" ");
  Serial.print(current_mA);
  Serial.print(" ");
  Serial.print(power_mW);
  Serial.print(" ");

  if (current_mA < 10 && power_mW <10)
  {
    // likely no load
    u8g2.setFont(u8g2_font_logisoso16_tf);  // choose a suitable font
    
    if ((millis()/1000)%2==1)
    {
      u8g2.drawBox(0,0,128,32);
      u8g2.setColorIndex(0);
      u8g2.drawStr(30 ,24, "NO LOAD");
      u8g2.setColorIndex(1);
    }
    else
    {
      u8g2.setCursor (30, 24);
      u8g2.print("NO LOAD");
    }
  }
  else
  {
    char ampstr[5] = "";
    char volstr[5] = "";
    char wattstr[6] = "";
    char printstr[20] = "";
    char symbol = '.';
    dtostrf(abs(current_mA/1000),2,2,ampstr);
    dtostrf(abs(loadvoltage),3,1,volstr);
    dtostrf(abs(power_mW/1000),4,1,wattstr);
  
    
  
    u8g2.setFont(u8g2_font_logisoso16_tf);  // choose a suitable font
    u8g2.setCursor (0, 16);
    sprintf(printstr,"%sv %sA",volstr, ampstr);
    u8g2.print(printstr);
    u8g2.setFont(u8g2_font_t0_12_tf);  // choose a suitable font
    u8g2.setCursor (0, 32);
    u8g2.print((millis()/1000)%2==1?'\xB7':' ');
    u8g2.print(wattstr);
    u8g2.print(" watt");
  }
  u8g2.sendBuffer();
  Serial.println(current_mA);
  delay(200);
}
