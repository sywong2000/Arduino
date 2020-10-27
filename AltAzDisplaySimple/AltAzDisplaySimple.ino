#include "SSD1306.h"
#include "src\MPU9250.h"
#include "math.h"

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
MPU9250 mpu;

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

float mX, mY, mZ, heading, pitch, roll, hX, hY, tc_heading, aXnorm, aYnorm, aX, aY, aZ, factor, cpitch, croll;

void setup()
{
  Serial.begin(115200);
  delay(2000);


  I2Ctwo.begin(MPU9250_SDA2, MPU9250_SCL2, 400000);
  I2Ctwo.beginTransmission(I2C_MPU9250_ADDRESS);

  Wire.begin();
  reset_oled();

  mpu.setup(I2Ctwo);
  //  mpu.setWire(&I2Ctwo);
  //  mpu.beginMag();


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
      display.println("Start calibration...");
      display.clear();
      display.drawLogBuffer(0, 0);
      display.display();
      delay(500);
      mpu.calibrateAccelGyro();
      mpu.calibrateMag();
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
    mpu.update();
    //mpu.print();

    aX = mpu.getAcc(0);
    aY = mpu.getAcc(1);
    aZ = mpu.getAcc(2);
    mX = mpu.getMag(0);
    mY = mpu.getMag(1);
    mZ = mpu.getMag(2);

    factor = sqrtf( aX * aX +  aY * aY + aZ * aZ);
    aXnorm = aX / factor;
    aYnorm = aY / factor;

    pitch = mpu.getPitch();
    roll = mpu.getRoll();
    cpitch = asinf(aXnorm); // pitch in radian
    croll = -asinf(aYnorm / cosf(pitch));


    hX = mX * cosf(pitch) + mZ * sinf(pitch);
    hY = mX * sinf(roll)  * sinf(pitch) + mY * cosf(roll) - mZ * sinf(roll) * cosf(pitch);

    tc_heading = atan2f(hY, hX) * 180 / Pi;
    heading = atan2f(mY, mX) * 180 / Pi;

    heading = heading < 0 ? heading + 360 : heading;
    tc_heading = hY < 0 ? tc_heading + 360 : tc_heading;


    //Serial.println(String(aX) + "\t" + String(aY) + "\t" + String(aZ) + "\t" + String(mX) + "\t" + String(mY) + "\t" + String(mZ) + "\t" + String(pitch) + "\t" + String(roll) + "\t" + String(heading) + "\t" + String(tc_heading));
    display.clear();
    display.println("heading (raw)=" + String(heading));
    //display.println("pitch=" + String(pitch)+".roll=" + String(roll));
    //display.println("mX=" + String(mX));
    //display.println("mY=" + String(mY));
    //display.println("mZ=" + String(mZ));
    display.println("pitch=" + String(pitch * 180 / Pi) + "/" + String(cpitch * 180 / Pi));
    display.println("roll=" + String(roll * 180 / Pi) + "/" + String(croll * 180 / Pi));
    display.println("heading =" + String(tc_heading));
    display.drawLogBuffer(0, 0);
    display.display();
  }


}
