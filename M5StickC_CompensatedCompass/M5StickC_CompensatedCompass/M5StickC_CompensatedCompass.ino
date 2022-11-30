/*
    note: need add library Adafruit_BMP280 from library manage
    Github: https://github.com/adafruit/Adafruit_BMP280_Library
*/
#include <EEPROM.h>
#include <M5StickC.h>
//#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
//#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;
//Adafruit_BMP280 bme;

float accX = 0;
float accY = 0;
float accZ = 0;

float roll = 0;
float pitch = 0;

double alpha = 0.2;
double theta, last_theta = 0;
double phi, last_phi     = 0;


float heading = 0;
double X_h , Y_h = 0;
uint8_t setup_flag = 1;


bmm150_mag_data value;
bmm150_mag_data last_value;

void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    //    Serial.print(".");
    delay(1);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;

  writeIntIntoEEPROM(0,value_offset.x);
  writeIntIntoEEPROM(2,value_offset.y);
  writeIntIntoEEPROM(4,value_offset.z);
  EEPROM.commit();

  Serial.printf("Calibration Offset written to EEPROM:");
  Serial.printf("X: %d   Y: %d   Z: %d   ", value_offset.x,value_offset.y,value_offset.z);
  Serial.println("");

}

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0, 26);
  M5.MPU6886.Init();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.println("Compass");
  pinMode(M5_BUTTON_HOME, INPUT);
  Serial.begin(115200);

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Compass Chip ID can not read!");
    while (1);
  } else {
    Serial.println("Initialize done!");
  }

  EEPROM.begin(6); //3 x int16_t

  value_offset.x = readIntFromEEPROM(0);
  value_offset.y = readIntFromEEPROM(2);
  value_offset.z = readIntFromEEPROM(4);

  Serial.printf("Calibration Offset loaded from EEPROM:");
  Serial.printf("X: %d   Y: %d   Z: %d   ", value_offset.x,value_offset.y,value_offset.z);
  Serial.println("");

}


void writeIntIntoEEPROM(int address, int16_t number)
{ 
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

int16_t readIntFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}
void loop() {

  M5.MPU6886.getAccelData(&accX, &accY, &accZ);

  if ((accX < 1) && (accX > -1)) {
    theta = asin(-accX);// * 57.295; // roll
  }
  if (accZ != 0) {
    phi = atan(accY / accZ);// * 57.295; //pitch
  }

  theta = alpha * theta + (1 - alpha) * last_theta;
  phi   = alpha * phi + (1 - alpha) * last_phi;


  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;


  value.x = alpha * value.x + (1 - alpha) * last_value.x;
  value.y = alpha * value.y + (1 - alpha) * last_value.y;
  value.z = alpha * value.z + (1 - alpha) * last_value.z;




  // heading 1 is the correct tilt compensated values

  X_h = (double)value.y * cos(phi) + (double)value.x * sin(theta) * sin(phi) + (double)value.z * cos(theta) * sin(phi);
  Y_h = (double)value.x * cos(theta) + (double)value.z * sin(theta);

  heading = atan2( Y_h, X_h);
  if (heading < 0) { /* Convert Azimuth in the range (0, 2pi) */
    heading = 2 * M_PI + heading;
  }

  Serial.printf("Heading: %.2f   Pitch: %.2f    Roll: %.2f   ", heading * 57.295, phi * 57.295, theta * 57.295);
  Serial.println("");

  M5.Lcd.setCursor(0, 40, 2);

  //  float pressure = bme.readPressure();
  //  M5.Lcd.setCursor(0, 60, 2);
  //  M5.Lcd.printf("pressure: %2.1f", pressure);
  delay(100);

  if (!setup_flag) {
    setup_flag = 1;

    if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
      Serial.println("Chip ID can not read!");
      while (1);
    } else {
      Serial.println("Initialize done!");
    }

    M5.MPU6886.Init();
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 10, 2);
    Serial.println("Calibrating compass now. Please flip and move the device around..");
    M5.Lcd.println("Calibrating compass now.");
    M5.Lcd.setCursor(0, 25, 2);
    M5.Lcd.println("Please flip & move the device around.");
    calibrate(10000);
    Serial.println("Calibrate done..");
    delay(5);
    //  M5.Speaker.beep();
    M5.Lcd.fillScreen(BLACK);

  }

  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.println("Compass");

  M5.Lcd.setCursor(0, 20, 2);
  M5.Lcd.printf("Heading: %2.1f    ", heading * 57.2957795);

  M5.Lcd.setCursor(0, 40, 2);
  M5.Lcd.printf("Pitch: %2.1f    ", phi * 57.2957795);

  M5.Lcd.setCursor(0, 60, 2);
  M5.Lcd.printf("Roll: %2.1f    ", theta * 57.2957795);

  last_theta = theta;
  last_phi   = phi;
  last_value.x = value.x;
  last_value.y = value.y;
  last_value.z = value.z;

  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    setup_flag = 0;
    while (digitalRead(M5_BUTTON_HOME) == LOW);
  }


}
