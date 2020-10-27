#include "SSD1306.h"
#include "src\MPU9250.h"
#include <math.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define OLED_SDA2 4
#define OLED_SCL2 15
#define I2C_OLED_ADDRESS    0x3C


#define MPU9250_SDA2 21
#define MPU9250_SCL2 22
#define I2C_MPU9250_ADDRESS  0x68
const float Pi = 3.141592653589793238462643383279502884f;

#define OLED_RESET_GPIO 16

TwoWire I2Ctwo = TwoWire(1);
SSD1306  display(I2C_OLED_ADDRESS, OLED_SDA2, OLED_SCL2);
MPU9250FIFO mpu(I2Ctwo,I2C_MPU9250_ADDRESS);

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

float mX, mY, mZ, heading, pitch, roll, hX, hY, tc_heading,aXnorm, aYnorm, aX, aY, aZ, factor, gX, gY, gZ, gPitch, gRoll, gYaw;

void setup()
{
  Serial.begin(115200);
  
  //delay(3000);
  Wire.begin();


  I2Ctwo.begin(MPU9250_SDA2, MPU9250_SCL2, 400000);
  I2Ctwo.beginTransmission(I2C_MPU9250_ADDRESS);
  mpu.begin();

  //  mpu.setWire(&I2Ctwo);
  //  mpu.beginMag();

  reset_oled();
  pinMode(button1, INPUT);
  display.init();
  display.setLogBuffer(4, 30);
  display.setContrast(100);
  display.clear();
  display.println("Waiting...");
  display.drawLogBuffer(0, 0);
  display.display();


}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(button1) == LOW)
  {
    if (longPressActive == false)
    {
      display.clear();
      display.println("Button pressed...");
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

      display.println("Button hold...");
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
    mpu.readSensor();
    //mpu.print();
    
    aX = mpu.getAccelX_mss();
    aY = mpu.getAccelY_mss();
    aZ = mpu.getAccelZ_mss();
    
    mX = mpu.getMagX_uT();
    mY = mpu.getMagY_uT();
    mZ = mpu.getMagZ_uT();

    gX = mpu.getGyroX_rads();
    gY = mpu.getGyroY_rads(); // pitch
    gZ = mpu.getGyroZ_rads();

    factor = sqrtf( aX*aX +  aY*aY + aZ*aZ);
    aXnorm = aX/factor;
    aYnorm = -aY/factor;
    
    pitch = asinf(aXnorm); // pitch in radian
    roll = asinf(aYnorm/cosf(pitch));
//    roll = asinf(aX/factor); //gyropitch, where gyropitch = accelpitch, accelpitch = asin(a_x/factor)
//    pitch = asinf(-aY/factor); //-gyroroll, where gyroroll = accelroll, accelrool = asin(a_y/factor)

    hX = mX * cosf(pitch) + mZ * sinf(pitch);
    hY = mX * sinf(roll)  * sinf(pitch) + mY * cosf(roll) - mZ * sinf(roll) * cosf(pitch);
    
//    hY = mX * cosf(pitch) + mY * sinf(roll) * sinf(pitch) - mZ * cosf(roll) * sinf(pitch);
//    hX = mY * cosf(roll) + mZ * sinf(roll);
    
    tc_heading = atan2f(hY, hX) * 180 / Pi;
    heading = atan2f(mY, mX) * 180 / Pi;

    if (tc_heading<0) tc_heading += 360;
    if (tc_heading>360) tc_heading -= 360;
    if (heading<0) heading += 360;
    if (heading>360) heading -= 360;

    //Serial.println(String(gX)+"\t"+String(gY)+"\t"+String(gZ)+"\t"+String(aX)+"\t"+String(aY)+"\t"+String(aZ)+"\t"+String(mX)+"\t"+String(mY)+"\t"+String(mZ)+"\t"+String(pitch)+"\t"+String(roll)+"\t"+String(heading)+"\t"+String(tc_heading));
    Serial.print(roll);
    Serial.print('\t');
    Serial.print(pitch);
    Serial.print('\t');
    Serial.print(tc_heading);
    Serial.println();
    
    display.clear();
    display.println("heading (raw)=" + String(heading));
    //display.println("pitch=" + String(pitch)+".roll=" + String(roll));
    //display.println("mX=" + String(mX));
    //display.println("mY=" + String(mY));
    //display.println("mZ=" + String(mZ));
    display.println("pitch=" + String(pitch* 180 / Pi));
    display.println("roll=" + String(roll* 180 / Pi));
    display.println("heading =" + String(tc_heading));
    display.drawLogBuffer(0, 0);
    display.display();
  }


}
