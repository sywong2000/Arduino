//2024 Aug 29 works okay on TMC2209 white board v2.1

#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial



int enablePin = 12;
int stepPin = 4;
int dirPin = 3;

int ms1Pin = 8;
int ms2Pin = 9;

int btnA = 10;
int btnB = 9;
int btnC = 8;
int btnD = 11;
long btnDelay = 40;
long DbtnDelay = 400;
long nPressed = 0;

//AccelStepper stepper(1, stepPin, dirPin);
int position = 0;
int nStepSize = 40;
int nSpeed = 4000;
int nStallCnt = 0;

#define SW_SCK           5      // Software Slave Clock (SCK) - BLUE
#define SW_TX            7      // SoftwareSerial receive pin - BROWN
#define SW_RX            6      // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)
#define STALL_VALUE     10 // [0..255]
#define DIAG_PIN        2
//int DIAG_PIN  = A0;
constexpr uint8_t sgthrs = STALL_VALUE;
constexpr uint8_t semin = 6;
constexpr uint8_t semax = 2;

// Create a new instance of the AccelStepper class:
// SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
// AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin);
 
void setup() {
  // Set the maximum speed in steps per second:
  // SoftSerial.begin(19200);           // initialize software serial for UART motor control
  Serial.begin(115200);
  driver.beginSerial(19200);      // Initialize UART

  driver.begin();
  driver.GSTAT(0b111);
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(120); // mA
  driver.microsteps(32);         // Set microsteps
  driver.TCOOLTHRS(0x3FF); // 20bit max
  // driver.seimin(1);                // minimum current for smart current control 0: 1/2 of IRUN 1: 1/4 of IRUN
  driver.semin(semin);                // [0... 15] If the StallGuard4 result falls below SEMIN*32, the motor current becomes increased to reduce motor load angle.
  driver.semax(semax);                // [0... 15]  If the StallGuard4 result is equal to or above (SEMIN+SEMAX+1)*32, the motor current becomes decreased to save energy.
  // driver.sedn(4);                  // current down step speed 0-11%
  // driver.iholddelay(3);            // 0 - 15 smooth current drop

  driver.sedn(0b00);     // Set current reduction rate
  driver.seup(0b01);     // Set current increase rate
  // driver.TPWMTHRS(0);        // 0: Disabled, 0xFFFFF = 1048575 MAX TSTEP.
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
  // driver.pwm_autoscale(true);      // Needed for stealthChop
  // driver.en_spreadCycle(false);    // false = StealthChop / true = SpreadCycle
  driver.intpol(true);             // interpolate to 256 microsteps
  driver.SGTHRS(STALL_VALUE);
  //attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterruptFunc, RISING);
  pinMode(DIAG_PIN, INPUT);


  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  // pinMode(ms1Pin, OUTPUT);
  // pinMode(ms2Pin, OUTPUT);

  // stepper.setMaxSpeed(nSpeed);
  // stepper.setSpeed(nSpeed);        
  // stepper.setEnablePin(enablePin);
  // stepper.setAcceleration(40000);


  // // digitalWrite(ms1Pin, HIGH);
  // // digitalWrite(ms2Pin, HIGH);
  // stepper.disableOutputs();

  // stepper.setCurrentPosition(0);
  // Serial.print("driver.SGTHRS = ");
  // Serial.println(driver.SGTHRS());
  driver.VACTUAL(4000);

  // digitalWrite(enablePin, HIGH);
  // stepper.disableOutputs();
  Serial.println("StallGuard_value Actual_current Diag_pin semin semax sgthrs");
}

// 100 steps half circle full step
// 1600 steps half circle 1/16
long int t1, t2, d;


bool stalled = false;
bool resetting = false;

void loop()
{
  static uint32_t last_time=0;
  uint32_t ms = millis();

  // if (stepper.distanceToGo()==0)
  // {
  //   stepper.moveTo(stepper.currentPosition()+40000);
  //   // Serial.println("Next Target");
  // }
  // stepper.run();
  
  if (ms - last_time > 50)
  {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%u %u %u %u %u %u",driver.SG_RESULT(),driver.cs2rms(driver.cs_actual()),100 * digitalRead(DIAG_PIN),semin*32, (semin+semax+1)*32, sgthrs*2);
  Serial.println(buffer);
  last_time = millis();
  }
}

void stallInterruptFunc(){ // flag set for motor A when motor stalls
    //Serial.println("Stalled..." + String(nStallCnt++));
    stalled = true;
}
