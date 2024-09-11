#include <EEPROM.h>
#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>


// TMC2209 pinout (left side  - target pin)
// EN     - GPIO_3_A3
// MS1    - N/A
// MS2    - N/A
// TX     - A4 GPIO_7
// RX     - A3 GPIO_4
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



#define SW_SCK           5      // Software Slave Clock (SCK) 
#define SW_TX            6      // SoftwareSerial receive pin 
#define SW_RX            7      // SoftwareSerial transmit pin 
#define ENABLE_PIN       12    
#define STEP_PIN         4
#define DIR_PIN          3

#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)
#define RMS_CURRENT       50
#define STALL_VALUE       100   // 0 - 255, higher value more sensitive


// const int ms1Pin = 8;
// const int ms2Pin = 9;

// SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

unsigned long currentPosition = 10000;
unsigned long targetPosition = 10000;
unsigned long lastSavedPosition = 0;

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

  // SoftSerial.begin(115200);           // initialize software serial for UART motor control
  driver.beginSerial(115200);      // Initialize UART
  driver.begin();
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(RMS_CURRENT); // mA
  driver.microsteps(8);         // Set microsteps
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.seimin(1);                // minimum current for smart current control 0: 1/2 of IRUN 1: 1/4 of IRUN
  driver.semin(15);                // [0... 15] If the StallGuard4 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
  driver.semax(15);                // [0... 15]  If the StallGuard4 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
  driver.sedn(4);                  // current down step speed 0-11%
  driver.iholddelay(3);            // 0 - 15 smooth current drop
  driver.TPWMTHRS(0);        // 0: Disabled, 0xFFFFF = 1048575 MAX TSTEP.
                                 // StealthChop PWM mode is enabled, if configured. When the velocity exceeds
                                 // the limit set by TPWMTHRS, the driver switches to SpreadCycle.
  driver.TCOOLTHRS(0);             // 0-7 TSTEP
                                 // 0: TPWM_THRS= 0
                                 // 1: TPWM_THRS= 200
                                 // 2: TPWM_THRS= 300
                                 // 3: TPWM_THRS= 400
                                 // 4: TPWM_THRS= 500
                                 // 5: TPWM_THRS= 800
                                 // 6: TPWM_THRS= 1200
                                 // 7: TPWM_THRS= 4000
  driver.pwm_autoscale(true);      // Needed for stealthChop
  driver.en_spreadCycle(false);    // false = StealthChop / true = SpreadCycle
  //driver.intpol(true);             // interpolate to 256 microsteps

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
  stepper.setCurrentPosition(currentPosition);
  lastSavedPosition = currentPosition;
  targetPosition = currentPosition;

  Serial.begin(9600);
}


void loop() {

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
      stepper.setAcceleration(s);

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
      driver.microsteps(8);         // Set microsteps
    }

    // half step
    if (cmd.equalsIgnoreCase("SH"))
    {
      bHalfStep = true;
      driver.microsteps(16);
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
      //driver.rms_current(RMS_CURRENT);
      stepper.disableOutputs();
      float s = targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed;
      stepper.setMaxSpeed(s);
      stepper.setSpeed(s);
      stepper.setAcceleration(s);

      bSetToMove = true;
    }

    // stop a move
    if (cmd.equalsIgnoreCase("FQ"))
    {
      bSetToMove = false;
      stepper.stop();
    
    }
    line = "";
    eoc = false;
  }  // eoc == true

  if (stepper.distanceToGo()!=0 && bSetToMove)
  {
    // stepper.setSpeed(targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed);
    // stepper.setAcceleration(targetPosition< stepper.currentPosition()?nSpeed/-20:nSpeed/20);
    // stepper.runSpeed();
    stepper.run();
    millisLastMove = millis();
  }

  // if (millis() - millisLastMove > millisDisableDelay)
  // {
  //   // Save current location in EEPROM
  //   if (lastSavedPosition != currentPosition)
  //   {
  //     EEPROM.put(0, currentPosition);
  //     lastSavedPosition = currentPosition;
  //   }
  //   // set motor to sleep state
  //   driver.rms_current(500);
  //   stepper.enableOutputs();
  // }
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
