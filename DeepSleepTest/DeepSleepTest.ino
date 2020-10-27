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

void setup() 
{
  reset_oled();
  display.init();
  display.setLogBuffer(5, 30);
  display.setContrast(255);
  display.clear();
  display.println("I am up");
  display.drawLogBuffer(0, 0);
  display.display();
  delay(1000);
  display.println("Preparing to deepsleep....");
  display.drawLogBuffer(0, 0);
  display.display();
  delay(1500);

  // put your setup code here, to run once:
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {
  // put your main code here, to run repeatedly:

}
