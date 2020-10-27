#include "SSD1306.h"
#include "src\MPU9250.h"
#include <math.h>
#include <float.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define OLED_SDA2 4
#define OLED_SCL2 15
#define I2C_OLED_ADDRESS    0x3C


#define MPU9250_SDA2 21
#define MPU9250_SCL2 22
#define I2C_MPU9250_ADDRESS  0x68
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
const float Pi = 3.141592653589793238462643383279502884f;


#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value




#define OLED_RESET_GPIO 16

TwoWire I2Ctwo = TwoWire(1);
SSD1306  display(I2C_OLED_ADDRESS, OLED_SDA2, OLED_SCL2);

void reset_oled()
{
  pinMode(OLED_RESET_GPIO, OUTPUT);
  digitalWrite(OLED_RESET_GPIO, LOW);
  delay(50);
  digitalWrite(OLED_RESET_GPIO, HIGH);
}

int button1 = 0;
long buttonTimer = 0;
long longPressTime  = 1000;
boolean buttonActive = false;
boolean longPressActive = false;


int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   SelfTest[6];
float   magCalibration[3];

void setup()
{
  Serial.begin(115200);

  //delay(3000);
  Wire.begin();

  I2Ctwo.begin(MPU9250_SDA2, MPU9250_SCL2, 400000);
  I2Ctwo.beginTransmission(I2C_MPU9250_ADDRESS);

  //  mpu.setWire(&I2Ctwo);
  //  mpu.beginMag();

  reset_oled();
  pinMode(button1, INPUT);
  display.init();
  display.setLogBuffer(4, 30);
  display.setContrast(100);
  display.clear();
  display.println("Waiting...");
  display.println("");
  display.println("");
  display.println("");
  display.drawLogBuffer(0, 0);
  display.display();

  byte c = readByte(I2C_MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  if (c == 0x71)
  {
    MPU9250SelfTest(SelfTest);
    display.clear();
    display.println("Done Self Test...");
    display.println("");
    display.println("");
    display.println("");
    display.drawLogBuffer(0, 0);
    display.display();
    initMPU9250();
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963

    initAK8963(magCalibration);

  }
  else
  {
    display.clear();
    display.println("Cannot connect MPU9250 " + String(c));
    display.println("");
    display.println("");
    display.println("");
    display.drawLogBuffer(0, 0);
    display.display();
    while (1);
  }




}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(button1) == LOW)
  {
    if (longPressActive == false)
    {
      display.clear();
      display.println("Button pressed...");
      display.println("");
      display.println("");
      display.println("");

      display.drawLogBuffer(0, 0);
      display.display();
    }
    if (buttonActive == false)
    {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false))
    {
      longPressActive = true;

      display.println("Button Hold...");
      display.println("");
      display.println("");
      display.println("");
      display.clear();
      display.drawLogBuffer(0, 0);
      display.display();
    }
  }
  else
  {
    if (buttonActive == true) {

      if (longPressActive == true) {

        longPressActive = false;
      }
      buttonActive = false;
    }
  }

  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];



  }

}
