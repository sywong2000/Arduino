// #include <EEPROM.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "EEPROM.h"



// remember to comment 	-> if (_usbLineInfo.lineState > 0)	{
// in serial:write function as workaround for Sparkfun Pro Micro board
// because the RTS and DTR needs to be disabled
// not need for ESP32-C3 & UNO

// PORT not exists
// https://www.reddit.com/r/esp32/comments/109b5gp/how_do_i_solve_port_does_not_exist_error/
// press and hold BOOT
// press reset
// release BOOT

// Arduino compile settings for ESP32-C3
// USB CDC On Boot: Enabled
// JTAG Adapter: Integrated USB JTAG

// TMC2209 pinout (c:\Users\sywong.manager\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.8.6\cores\arduino\CDC.cppleft side  - target pin)
// EN     - GPIO_3_A3
// MS1    - N/A
// MS2    - N/A
// TX     - RX GPIO_20
// RX     - TX GPIO_21
// CLK    - A2 GPIO_2
// STEP   - A1 GPIO_1
// DIR    - A0 GPIO_0

// ESP-C3 ABRobot 

// 5V           |       |   GPIO_10
// GND          |       |   GPIO_9_SCL (GPIO_6_OLED)
// 3.3V         |       |   GPIO_8_SDA (GPIO_5_OLED)
// RX GPIO20    |       |   GPIO_7_SS
// TX GPIO21    |       |   GPIO_6_MOSI
// A2 GPIO_2    |       |   GPIO_5_A5_MISO
// A1 GPIO_1    |       |   GPIO_4_A4
// A0 GPIO_0    |       |   GPIO_3_A3



// #define SW_SCK           5      // Software Slave Clock (SCK) 
// #define SW_TX            6      // SoftwareSerial receive pin 
// #define SW_RX            7      // SoftwareSerial transmit pin 
// #define ENABLE_PIN       12    
// #define STEP_PIN         4
// #define DIR_PIN          3

// #define SW_SCK           A2
// #define SW_TX            A5
// #define SW_RX            A4
#define ENABLE_PIN        A3    
#define STEP_PIN          A1
#define DIR_PIN           A0
#define HALF_STEP         32
#define FULL_STEP         16

#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE           0.11f   // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)
#define RMS_CURRENT       1700    // (RMS*sqrt(2) ~ max current consumption)
#define STALL_VALUE       100     // 0 - 255, higher value more sensitive


// const int ms1Pin = 8;
// const int ms2Pin = 9;

#define WIRE Wire
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;


// SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
// TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
TMC2209Stepper driver(&Serial0, R_SENSE, DRIVER_ADDRESS); // ESP32 try uses Serial0
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

unsigned long currentPosition = 10000;
unsigned long targetPosition = 10000;
unsigned long lastSavedPosition = 0;

constexpr uint8_t semin = 6;
constexpr uint8_t semax = 2;
String OledDisplay = "";


bool eoc = false;
bool bHalfStep = false;
int LineLen= 0;
String cmd, param, line;
const long nSpeedConstant = 100;
long nSpeedFactor = 10;
long nSpeed = nSpeedConstant* nSpeedFactor;

float tCoeff = 0;
bool bSetToMove = false;
long millisLastMove = 0;
const long millisDisableDelay = 10000; // 10 sec


long hexstr2long(String hexstr)
{
  char buf[hexstr.length() + 1];
  hexstr.toCharArray(buf, hexstr.length() + 1);
  return strtol(buf, NULL, 16);
}


void setup() {

  // init OLED with SSD1306 ASCII lib
  Wire.begin(5,6);
  Wire.setClock(400000);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.clear();
  oled.ssd1306WriteCmd(SSD1306_SEGREMAP);
  oled.ssd1306WriteCmd(SSD1306_COMSCANINC);
  oled.setContrast(255);
  oled.setFont(Callibri15);

  // SoftSerial.begin(115200);           // initialize software serial for UART motor control
  Serial0.begin(115200);
  //driver.beginSerial(115200);      // Initialize UART
  driver.GSTAT(0b111);
  driver.begin();
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(RMS_CURRENT); // mA
  driver.microsteps(FULL_STEP);         // Set microsteps
  driver.TCOOLTHRS(0x3FF); // 20bit max
  // driver.seimin(1);                // minimum current for smart current control 0: 1/2 of IRUN 1: 1/4 of IRUN
  driver.semin(semin);                // [0... 15] If the StallGuard4 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
  driver.semax(semax);                // [0... 15]  If the StallGuard4 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
  driver.sedn(0b01);      // Set current reduction rate
  driver.seup(0b01);      // Set current increase rate
  driver.irun(31);
  driver.ihold(0);               // hold current  0=1/32 … 31=32/32
  driver.iholddelay(2);            // 0 - 15 smooth current drop
  // driver.TPWMTHRS(0);          // 0: Disabled, 0xFFFFF = 1048575 MAX TSTEP.
  //                                // StealthChop PWM mode is enabled, if configured. When the velocity exceeds
  //                                // the limit set by TPWMTHRS, the driver switches to SpreadCycle.
  // driver.TCOOLTHRS(0);             // 0-7 TSTEP
  //                                // 0: TPWM_THRS= 0
  //                                // 1: TPWM_THRS= 200
  //                                // 2: TPWM_THRS= 300
  //                                // 3: TPWM_THRS= 400
  //                                // 4: TPWM_THRS= 500
  //                                // 5: TPWM_THRS= 800
  //                                // 6: TPWM_THRS= 1200
  //                                // 7: TPWM_THRS= 4000
  driver.pwm_autoscale(true);      // Needed for stealthChop
  driver.en_spreadCycle(false);    // false = StealthChop / true = SpreadCycle
  driver.intpol(true);             // interpolate to 256 microsteps

  // driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  stepper.setEnablePin(ENABLE_PIN);
  stepper.disableOutputs();
  stepper.setMaxSpeed(nSpeed);
  stepper.setSpeed(nSpeed);

  // init position from EEPROM
  // default position is 10000
  EEPROM.get(0, currentPosition);
  if (isnan(currentPosition) || currentPosition<0 || currentPosition>65535)
  {
    EEPROM.put(0, (long)10000);
  }
  stepper.setCurrentPosition  (currentPosition);
  lastSavedPosition = currentPosition;
  targetPosition = currentPosition;

  Serial.begin(9600);
}


int oled_last_refresh = millis();

void loop() {

  OledDisplay = "READY    ";

  // process when end of command
  if (eoc)
  {
    if (line.startsWith("2"))
    {
      //debugSerial.println("Got Dual focuser command(?) starting with 2. Send values of first motor");
      // remove first character and parse command
      line = line.substring(1);
    }

    LineLen = line.length();
    if (LineLen >= 2)
    {
      cmd = line.substring(0, 2);
    }
    else
    {
      cmd = line.substring(0, 1);
    }

    if (LineLen > 2)
    {
      param = line.substring(2);
    }

    cmd.trim();
    param.trim();


    // GB - get back light value
    if (cmd.equalsIgnoreCase("GB"))
    {
      Serial.print("00#");
    }

    // PH - Find Motor Home
    if (cmd.equalsIgnoreCase("PH"))
    {
      stepper.setCurrentPosition(10000);
      stepper.moveTo(0);
    }

    // GV - firmware version
    if (cmd.equalsIgnoreCase("GV"))
    {
      Serial.print("10#");
    }

    // C - temperature conversion
    if (cmd.equalsIgnoreCase("C"))
    {
      // Serial.print("10#");
    }

    // GP - get current position
    if (cmd.equalsIgnoreCase("GP"))
    {
      currentPosition = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04lX", currentPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // GN - Get New Position
    if (cmd.equalsIgnoreCase("GN"))
    {
      targetPosition = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04lX", targetPosition);
      Serial.print(tempString);
      Serial.print("#");
    }

    // GT - get temperature
    if (cmd.equalsIgnoreCase("GT"))
    {
    //      sensors.requestTemperatures();
      float temperature = 25.0;//sensors.getTempCByIndex(0);
      if (temperature > 100 || temperature < -50)
      {
        // error
        temperature = 0;
      }
      byte t_int = (byte)temperature << 1;
      t_int += round(temperature - (byte)temperature);
      Serial.print(t_int, HEX);
      Serial.print('#');
    }

    // GC - Get temperature coefficient
    if (cmd.equalsIgnoreCase("GC"))
    {
      // Serial.print("02#");
      Serial.print((byte)tCoeff, HEX);
      Serial.print('#');
    }

    // SC - Set temperature coefficient
    if (cmd.equalsIgnoreCase("SC"))
    {
      if (param.length() > 4)
      {
        param = param.substring(param.length() - 4);
      }
      if (param.startsWith("F"))
      {
        tCoeff = ((0xFFFF - strtol(param.c_str(), NULL, 16)) / -2.0f) - 0.5f;
      }
      else
      {
        tCoeff = strtol(param.c_str(), NULL, 16) / 2.0f;
      }
    }

    // GD - Get Motor Speed
    // 20 - 15pps
    // 10 - 30pps
    // 08 - 60pps
    // 04 - 125pps
    // 02 - 250pps

    if (cmd.equalsIgnoreCase("GD"))
    {
      char tempString[6];
      sprintf(tempString, "%02X", nSpeedFactor*32);
      Serial.print(tempString);
      Serial.print("#");
    }

    // SD - Set Motor Speed
    // 20 - 15pps
    // 10 - 30pps
    // 08 - 60pps
    // 04 - 125pps
    // 02 - 250pps

    if (cmd.equalsIgnoreCase("SD"))
    {
      nSpeedFactor = 32/hexstr2long(param);
      // SpeedFactor: smaller value means faster
      nSpeed = nSpeedConstant * nSpeedFactor;
      float s = targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed;
      stepper.setMaxSpeed(s);
      stepper.setSpeed(s);
      stepper.setAcceleration(50000);

      // stepper.setMaxSpeed(nSpeed);
      // stepper.setSpeed(nSpeed);
    }


    // get half step    
    if (cmd.equalsIgnoreCase("GH"))
    {
      if (bHalfStep)
      {
        Serial.print("FF#");
      }
      else
      {
        Serial.print("00#");
      }
    }

    // full step
    if (cmd.equalsIgnoreCase("SF"))
    {
      bHalfStep = false;
      driver.microsteps(FULL_STEP);
    }

    // half step
    if (cmd.equalsIgnoreCase("SH"))
    {
      bHalfStep = true;
      driver.microsteps(HALF_STEP);
    }

    // GI - is Motor Moving
    if (cmd.equalsIgnoreCase("GI"))
    {
      if (stepper.distanceToGo() != 0 && bSetToMove)
      {
        Serial.print("01#");
      }
      else
      {
        Serial.print("00#");
      }
    }
    // set current motor position
    if (cmd.equalsIgnoreCase("SP"))
    {
      currentPosition = hexstr2long(param);
      stepper.setCurrentPosition(currentPosition);
    }

    // set new motor position
    if (cmd.equalsIgnoreCase("SN"))
    {
      targetPosition = hexstr2long(param);
      stepper.moveTo(targetPosition);
    }

    // initiate a move
    if (cmd.equalsIgnoreCase("FG"))
    {
      OledDisplay = "MOVING";
      oled.setRow(2);
      oled.setCol(36);
      oled.print(OledDisplay);

      stepper.disableOutputs();
      float s = targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed;
      stepper.setMaxSpeed(s);
      stepper.setSpeed(s);
      stepper.setAcceleration(50000);
      bSetToMove = true;
    }

    // stop a move
    if (cmd.equalsIgnoreCase("FQ"))
    {
      bSetToMove = false;
      stepper.stop();
    
    }

    // non-standard commands

    if (cmd.equalsIgnoreCase("SI"))
    {
      // set RMS current
      // e.g. :SI0500$ - set to 500mA
      int nCurrent = param.toInt();
      driver.rms_current(nCurrent);
    }


    line = "";
    eoc = false;
  }  // eoc == true

  if (stepper.distanceToGo()!=0 && bSetToMove)
  {
    // stepper.setSpeed(targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed);
    // stepper.setAcceleration(targetPosition< stepper.currentPosition()?nSpeed/-20:nSpeed/20);
    // stepper.runSpeed();
    OledDisplay = "MOVING";
    stepper.run();
    millisLastMove = millis();
  }

  
  if (millis()- oled_last_refresh > 50 && (!bSetToMove|| stepper.distanceToGo()==0))
  {
    oled.setRow(2);
    oled.setCol(36);
    oled.print(OledDisplay);
    oled_last_refresh = millis();
  }

  if (millis() - millisLastMove > millisDisableDelay)
  {
    // Save current location in EEPROM
    if (lastSavedPosition != currentPosition)
    {
      EEPROM.put(0, currentPosition);
      lastSavedPosition = currentPosition;
    }
  }
}


// For sparkfun pro micro
void serialEventRun(void) {
  if (Serial.available()) serialEvent();
}

void serialEvent() {
  while (Serial.available()>0 && !eoc)
  {
    char c = Serial.read();
    if (c != '#' && c != ':')
    {
      line = line + c;
    }
    else
    {
      if (c == '#')
      {
        eoc = true;
        // break and handle the full command first
        break;
      }
      if (c == ':')
      {
        line = "";
      }
    }
  }
}
