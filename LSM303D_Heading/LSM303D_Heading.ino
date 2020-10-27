#include <Wire.h>
#include <LSM303.h>
#define RAD_TO_DEG 57.2957795

LSM303 compass;
float roll, pitch;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  Serial.print("Device Type = ");
  Serial.print(compass.getDeviceType());
  Serial.println();
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}

void loop() {
  compass.read();
  Serial.print("Heading = ");
  Serial.print(compass.heading());
  Serial.println();
  
  delay(100);
}
