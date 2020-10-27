#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <INA226_WE.h>

#define I2C_ADDRESS 0x40
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);
INA226_WE ina226(I2C_ADDRESS);
boolean noload = true;
//float current = 0, shuntvoltage=0, busvoltage=0, power=0, loadvoltage= 0;

void setup(void) {
  Serial.begin(115200);
  Serial.println("Initializing..");
  Wire.begin();
  ina226.init();
  Serial.println("INA226 (Ammeter) OK");
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
  
  
  u8g2.clearBuffer();          // clear the internal menory
  ina226.readAndClearFlags();
  float shuntVoltage_mV = ina226.getShuntVoltage_mV();
  float busVoltage_V = ina226.getBusVoltage_V();
  float current_mA = ina226.getCurrent_mA();
  float power_mW = ina226.getBusPower();
  float loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  
  //loadvoltage = busvoltage + (shuntvoltage / 1000);
  Serial.print(shuntVoltage_mV);
  Serial.print("mV ");
  Serial.print(busVoltage_V);
  Serial.print("V ");
  Serial.print(current_mA);
  Serial.print("mA ");
  Serial.print(power_mW);
  Serial.print("mW ");
  Serial.print(loadVoltage_V);
  Serial.print("V ");
  Serial.println();

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
    dtostrf(abs(current_mA/100.0),2,2,ampstr);
    dtostrf(abs(loadVoltage_V),3,1,volstr);
    dtostrf(abs(power_mW/100.0),4,1,wattstr);
  
    //u8g2.setFont(u8g2_font_fub30_tn);
    u8g2.setFont(u8g2_font_10x20_mn);
    
    //u8g2.setFont(u8g2_font_t0_12_tf);  // choose a suitable font
    //u8g2.setFont(u8g2_font_logisoso16_tf);  // choose a suitable font
    u8g2.setCursor (0, 16);
    sprintf(printstr,"%sv %sA",volstr, ampstr);
    u8g2.print(printstr);
    
    u8g2.setCursor (0, 32);
    u8g2.print((millis()/1000)%2==1?'\xB7':' ');
    u8g2.print(wattstr);
    u8g2.print(" watt");
  }
  u8g2.sendBuffer();
  delay(100);
}
