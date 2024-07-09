#include <TMCStepper.h>  // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>  // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
//#include <MultiStepper.h>

int enablePin = 2;
int stepPin = 4;
int dirPin = 3;

int ms1Pin = 8;
int ms2Pin = 9;

int btn11 = 11;
int btn12 = 12;
int btn13 = 13;
int btn14 = 14;
long btnDelay = 40;
long DbtnDelay = 400;
long nPressed = 0;

int ledPin = 10;

//AccelStepper stepper(1, stepPin, dirPin);
int position = 0;
int nStepSize = 40;
int nSpeed = 800;
int brightness = 0;

#define SW_SCK 5             // Software Slave Clock (SCK) - BLUE
#define SW_TX 6              // SoftwareSerial receive pin - BROWN
#define SW_RX 7              // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f        // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

// Create a new instance of the AccelStepper class:
SoftwareSerial SoftSerial(SW_RX, SW_TX);                      // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);  // Create TMC driver
AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin);

void setup() {
  // Set the maximum speed in steps per second:
  SoftSerial.begin(11520);    // initialize software serial for UART motor control
  driver.beginSerial(11520);  // Initialize UART

  driver.begin();            // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);            // Enables driver in software
  driver.rms_current(2500);  // Set motor RMS current
  driver.microsteps(256);    // Set microsteps

  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);  // Needed for stealthChop

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // pinMode(ms1Pin, OUTPUT);
  // pinMode(ms2Pin, OUTPUT);

  stepper.setMaxSpeed(8000);
  stepper.setSpeed(1000);
  stepper.setEnablePin(enablePin);
  // digitalWrite(ms1Pin, HIGH);
  // digitalWrite(ms2Pin, HIGH);
  stepper.disableOutputs();

  Serial.begin(115200);

  //default brightness
  brightness = 10;
  brightness = map(brightness, 0, 255, 0, 255);
}

// 100 steps half circle full step
// 1600 steps half circle 1/16
long int t1, t2, d;
bool bSimStarted = false;


void loop() {

  analogWrite(ledPin,brightness);
  nSpeed = 1200;
  while (digitalRead(btn11) == LOW) {
    bSimStarted = false;
    stepper.setSpeed(nSpeed);
    stepper.runSpeed();
  }

  // d = (millis() - t1);
  // Serial.print("Duration: ");
  // Serial.print(d);
  // Serial.println(" ms.");

  // delay(1000);

  if (digitalRead(btn12) == LOW && !bSimStarted) {
    bSimStarted = true;
    t1 = millis();
    nSpeed = -6.6;
    Serial.print("Running at speed:");
    Serial.println(nSpeed);
    t1 = millis();

    // 150 degress / 1.8 degree per step * 256 microsteps = 21333

    stepper.setCurrentPosition(0);
  }

  while (bSimStarted && stepper.currentPosition() < 21300) {
    stepper.setSpeed(nSpeed);
    stepper.runSpeed();
  }
}
