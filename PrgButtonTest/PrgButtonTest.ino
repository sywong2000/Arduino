#include "SSD1306.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define I2C_EXAMPLE_MASTER_SDA_IO 4 /*!< gpio number for I2C master data *//////////////
#define OLED_RESET_GPIO 16

#define SDA2 4
#define SCL2 15

SSD1306  display(0x3c, 4, 15);

void reset_oled()
{
  gpio_set_direction((gpio_num_t)OLED_RESET_GPIO, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT );
  gpio_set_level((gpio_num_t)OLED_RESET_GPIO, 1);
  vTaskDelay(50 / portTICK_RATE_MS);
  gpio_set_level((gpio_num_t)OLED_RESET_GPIO, 0);
  vTaskDelay(50 / portTICK_RATE_MS);
  gpio_set_level((gpio_num_t)OLED_RESET_GPIO, 1);
  vTaskDelay(50 / portTICK_RATE_MS);
}

int button1 = 0;
long buttonTimer = 0;
long longPressTime  = 1000;
boolean buttonActive = false;
boolean longPressActive = false;

void setup()
{
  reset_oled();
  pinMode(button1, INPUT);
  display.init();
  display.setLogBuffer(1, 30);
  display.setContrast(255);
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

    display.println("Waiting for button...");
    display.clear();
    display.drawLogBuffer(0, 0);
    display.display();
  }


}
