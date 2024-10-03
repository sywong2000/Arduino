#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "EEPROM.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"



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

// TMC2209 pinout 
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
#define DIAG_PIN          A4

#define HALF_STEP         64
#define FULL_STEP         32

#define DRIVER_ADDRESS    0b00    // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE           0.11f   // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)
#define RMS_CURRENT       2100    // (RMS*sqrt(2) ~ max current consumption)
#define IDLE_RMS_CURRENT  200     // Idle Current
#define STALL_VALUE       100     // 0 - 255, higher value more sensitive


// const int ms1Pin = 8;
// const int ms2Pin = 9;

#define WIRE Wire
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// BLE services and characteristics UUID
#define SERVICE_UUID                            "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CURRENT_POS_CHARACTERISTIC_UUID         "19b10001-e8f2-537e-4f6c-d104768a1214"
#define TARGET_POS_CHARACTERISTIC_UUID          "19b10002-e8f2-537e-4f6c-d104768a1214"
#define IS_MOVING_CHARACTERISTIC_UUID           "19b10003-e8f2-537e-4f6c-d104768a1214"
#define HALT_REQUEST_CHARACTERISTIC_UUID        "19b10004-e8f2-537e-4f6c-d104768a1214"

#define SG_RESULT_CHARACTERISTIC_UUID           "19b10005-e8f2-537e-4f6c-d104768a1214"
#define ACTUAL_CURRENT_CHARACTERISTIC_UUID      "19b10006-e8f2-537e-4f6c-d104768a1214"
#define DIAG_VALUE_CHARACTERISTIC_UUID          "19b10007-e8f2-537e-4f6c-d104768a1214"
#define SEMIN_CHARACTERISTIC_UUID               "19b10008-e8f2-537e-4f6c-d104768a1214"
#define SEMAX_CHARACTERISTIC_UUID               "19b10009-e8f2-537e-4f6c-d104768a1214"
#define SGTHRS_CHARACTERISTIC_UUID              "19b10010-e8f2-537e-4f6c-d104768a1214"

#define SPEED_CHARACTERISTIC_UUID               "19b10011-e8f2-537e-4f6c-d104768a1214"

bool bHaltRequestFromBLE = false;

BLEServer* pServer = NULL;
BLECharacteristic* pCurrentPosCharacteristic = NULL;    // read, notification
BLECharacteristic* pIsMovingCharacteristic = NULL;      // read, notification
BLECharacteristic* pSGResultCharacteristic = NULL;    // read, notification
BLECharacteristic* pActualCurrentCharacteristic = NULL;      // read, notification
BLECharacteristic* pDiagValueCharacteristic = NULL;      // read, notification
BLECharacteristic* pSeMinCharacteristic = NULL;    // read, notification
BLECharacteristic* pSeMaxCharacteristic = NULL;      // read, notification
BLECharacteristic* pSgThrsCharacteristic = NULL;    // read, notification



BLECharacteristic* pSpeedCharacteristic = NULL;    // read, write
BLECharacteristic* pTargetPosCharacteristic = NULL;     // read, write
BLECharacteristic* pHaltRequestCharacteristic = NULL; // read, write 


bool BLE_DeviceConnected = false;
bool BLE_DeviceConnectedPrev = false;


class FocuserServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLE_DeviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    BLE_DeviceConnected = false;
  }
};


class FocuserTargetPosCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pTargetPosCharacteristic) {

    String value = pTargetPosCharacteristic->getValue();
    
    if (value.length() > 0) {
      int nRequestedTargetPositionFromBLE = value.toInt();
      if (nRequestedTargetPositionFromBLE >=0) {
        targetPosition = nRequestedTargetPositionFromBLE;
        stepper.moveTo(targetPosition);
        driver.rms_current(nCurrent);
        driver.ihold(0);
        driver.iholddelay(2);
        bSetIdle = false;
        stepper.disableOutputs();
        updateStepperSpeed(nSpeed);
      }
    }
  }
};

class FocuserSpeedCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pSpeedCharacteristic) {

    String value = pSpeedCharacteristic->getValue();
    
    if (value.length() > 0) {
      int nPPSFromBLE = value.toInt();
      if (nPPSFromBLE >=0) {
        nSpeedFactor = 32/nPPSFromBLE;
        nSpeed = nSpeedConstant * nSpeedFactor;
        updateStepperSpeed(nSpeed);
      }
    }
  }
};

class FocuserHaltRequestCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pHaltRequestedCharacteristic) {

    String value = pHaltRequestedCharacteristic->getValue();
    if (value.length() > 0) {
      // Serial.print("Characteristic event, written: ");
      // Serial.println(static_cast<int>(value[0])); // Print the integer value

      bHaltRequestFromBLE = static_cast<bool>(value[0]);
    }
    // pLedCharacteristic->setValue("Hello World!");
    // pLedCharacteristic->notify();
  }
};

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
String OledDisplayed = "";


bool eoc = false;
bool bHalfStep = false;
int LineLen= 0;
String cmd, param, line;
long nSpeedConstant = 100;
long nSpeedFactor = 16; // default is 250pps (32/02)
long nSpeed = nSpeedConstant* nSpeedFactor;

float tCoeff = 0;
bool bSetToMove = false;
bool bSetIdle = false;
long millisLastMove = 0;
long lastCharacteristicsUpdate = 0;
long nextAdvertisementMillis = -1;

const long millisIdleDelay = 5000; // 10 sec
const long nBLECharacteristicsRefreshMs = 50;


int nCurrent = RMS_CURRENT;
int nLastPollByClient = 0;


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
  driver.rms_current(RMS_CURRENT);      // set the initial mA
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


  // prepare the BLE server
  BLEDevice::init("MOONLITE BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new FocuserServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTargetPosCharacteristic = pService->createCharacteristic(
    TARGET_POS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE
  );
  
  pTargetPosCharacteristic->setCallbacks(new FocuserTargetPosCharacteristicCallbacks());

  pHaltRequestCharacteristic = pService->createCharacteristic(
    HALT_REQUEST_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE
    );

  pHaltRequestCharacteristic->setCallbacks(new FocuserHaltRequestCharacteristicCallbacks());

  pSpeedCharacteristic = pService->createCharacteristic(
    SPEED_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE
    );

  pSpeedCharacteristic->setCallbacks(new FocuserSpeedCharacteristicCallbacks());

  pCurrentPosCharacteristic =  pService->createCharacteristic(
    CURRENT_POS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pIsMovingCharacteristic =  pService->createCharacteristic(
    IS_MOVING_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pSGResultCharacteristic =  pService->createCharacteristic(
    SG_RESULT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pActualCurrentCharacteristic =  pService->createCharacteristic(
    ACTUAL_CURRENT_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pDiagValueCharacteristic =  pService->createCharacteristic(
    DIAG_VALUE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pSeMinCharacteristic =  pService->createCharacteristic(
    SEMIN_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pSeMaxCharacteristic =  pService->createCharacteristic(
    SEMAX_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY 
  );

  pSgThrsCharacteristic =  pService->createCharacteristic(
    SGTHRS_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY 
  );


  pTargetPosCharacteristic->addDescriptor(new BLE2902());
  pHaltRequestCharacteristic->addDescriptor(new BLE2902());
  pCurrentPosCharacteristic->addDescriptor(new BLE2902());
  pIsMovingCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  pAdvertising->setAdvertisementType(ADV_TYPE_IND);
  // BLEAdvertisementData oAdvData = BLEAdvertisementData();
  // oAdvData.setName("MOONLITE BLE");
  // oAdvData.setShortName("MOONLITEBLE");

  // pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();


  OledDisplay = "   IDLE   ";
  updateOLED();
}

int oled_last_refresh = millis();

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
      nLastPollByClient = millis();
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
      updateStepperSpeed(nSpeed);
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
      bSetIdle = false;
      updateOLED();

      driver.rms_current(nCurrent);
      driver.ihold(0);
      driver.iholddelay(2);

      stepper.disableOutputs();
      updateStepperSpeed(nSpeed);
      bSetToMove = true;
    }

    // stop a move
    if (cmd.equalsIgnoreCase("FQ"))
    {
      bSetToMove = false;
      stepper.stop();
    }

    // non-standard commands

    // set RMS current
    // e.g. :SI0500$ - set to 500mA
    if (cmd.equalsIgnoreCase("SI"))
    {
      nCurrent = param.toInt();
      driver.rms_current(nCurrent);
      // driver.irun(31);
      driver.ihold(0);
      driver.iholddelay(2);
    }

    // set Speed Rate Constant
    // e.g. :RC10$ - set to nSpeedConstant to 10
    if (cmd.equalsIgnoreCase("RC"))
    {
      nSpeedConstant = param.toInt();
      nSpeed = nSpeedConstant * nSpeedFactor;
      float s = targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed;
      stepper.setMaxSpeed(s);
      stepper.setSpeed(s);
      stepper.setAcceleration(50000);
    }


    line = "";
    eoc = false;
  }  // eoc == true

  if (bHaltRequestFromBLE)
  {
      bSetToMove = false;
      stepper.stop();
      bHaltRequestFromBLE = false;
  }

  if (BLE_DeviceConnected && (millis() - lastCharacteristicsUpdate > nBLECharacteristicsRefreshMs))
  {
    // update the characteristics
    if (String(stepper.currentPosition())!= pCurrentPosCharacteristic->getValue())
    {
      pCurrentPosCharacteristic->setValue(String(stepper.currentPosition()));
      pCurrentPosCharacteristic->notify();
    }
    if (String(stepper.distanceToGo() != 0 && bSetToMove)!=pIsMovingCharacteristic->getValue())
    {
      pIsMovingCharacteristic->setValue(String(stepper.distanceToGo() != 0 && bSetToMove));
      pIsMovingCharacteristic->notify();
    }

    if (String(driver.SG_RESULT())!=pSGResultCharacteristic->getValue())
    {
      pSGResultCharacteristic->setValue(String(driver.SG_RESULT()));
      pSGResultCharacteristic->notify();
    }

    if (String(driver.cs2rms(driver.cs_actual()))!=pActualCurrentCharacteristic->getValue())
    {
      pActualCurrentCharacteristic->setValue(String(driver.cs2rms(driver.cs_actual())));
      pActualCurrentCharacteristic->notify();
    }

    if (String(digitalRead(DIAG_PIN))!=pDiagValueCharacteristic->getValue())
    {
      pDiagValueCharacteristic->setValue(String(digitalRead(DIAG_PIN)));
      pDiagValueCharacteristic->notify();
    }

    if (String(driver.semin()*32)!=pSeMinCharacteristic->getValue())
    {
      pSeMinCharacteristic->setValue(String(driver.semin()*32));
      pSeMinCharacteristic->notify();
    }

    if (String((driver.semin()+driver.semax()+1)*32)!=pSeMaxCharacteristic->getValue())
    {
      pSeMaxCharacteristic->setValue(String((driver.semin()+driver.semax()+1)*32));
      pSeMaxCharacteristic->notify();
    }

    if (String(driver.SGTHRS()*2)!=pSgThrsCharacteristic->getValue())
    {
      pSgThrsCharacteristic->setValue(String(driver.SGTHRS()*2));
      pSgThrsCharacteristic->notify();
    }

    lastCharacteristicsUpdate = millis();
  }

  // disconnecting
  if (!BLE_DeviceConnected && BLE_DeviceConnectedPrev) {
    nextAdvertisementMillis = millis() + 500; // start the advertisement again 500ms later, non blocking
    BLE_DeviceConnectedPrev = BLE_DeviceConnected;
  }
  // connecting
  if ( BLE_DeviceConnected && !BLE_DeviceConnectedPrev) {
    BLE_DeviceConnectedPrev = BLE_DeviceConnected;
  }


  if (nextAdvertisementMillis>0 && millis()> nextAdvertisementMillis)
  {
    nextAdvertisementMillis = -1;
    pServer->startAdvertising(); // restart advertising
  }



  if (stepper.distanceToGo()!=0 && bSetToMove)
  {
    // stepper.setSpeed(targetPosition< stepper.currentPosition()?nSpeed*-1:nSpeed);
    // stepper.setAcceleration(targetPosition< stepper.currentPosition()?nSpeed/-20:nSpeed/20);
    // stepper.runSpeed();
    stepper.run();
    millisLastMove = millis();
    OledDisplay = "  MOVING  ";
    updateOLED();
  }

  if (stepper.distanceToGo()==0)
  {
    if (millis()-nLastPollByClient>800)
    {
      OledDisplay = "    IDLE    ";
    }
    else
    {
      OledDisplay = "   READY   ";
    }
    updateOLED();
  }

  
  // if (millis()- oled_last_refresh > 50 && (!bSetToMove|| stepper.distanceToGo()==0))
  // {
  //   oled.setRow(2);
  //   oled.setCol(36);
  //   oled.print(OledDisplay);
  //   oled_last_refresh = millis();
  // }

  if (millis() - millisLastMove > millisIdleDelay)
  {
    // Save current location in EEPROM
    if (lastSavedPosition != currentPosition)
    {
      EEPROM.put(0, currentPosition);
      lastSavedPosition = currentPosition;
    }
    
    if (!bSetIdle)
    {
      driver.rms_current(IDLE_RMS_CURRENT);
      driver.ihold(0);
      driver.iholddelay(2);
      bSetIdle = true;
    }
  }
}

void updateStepperSpeed(int nS)
{
  float s = targetPosition< stepper.currentPosition()?nS*-1:nS;
  stepper.setMaxSpeed(s);
  stepper.setSpeed(s);
  stepper.setAcceleration(50000);
}

void updateOLED()
{
    if (OledDisplay != OledDisplayed)
    {
      oled.clear();
      oled.setRow(2);
      oled.setCol(36);
      oled.print(OledDisplay);
      OledDisplayed = OledDisplay;
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
