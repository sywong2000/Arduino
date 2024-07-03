#include <TMCStepper.h>         // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <AccelStepper.h>
#include <SoftwareSerial.h>     // Software serial for the UART to TMC2209 - https://www.arduino.cc/en/Reference/softwareSerial
//#include <MultiStepper.h>

int enablePin = 2;
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
int nSpeed = 800;


#define SW_SCK           5      // Software Slave Clock (SCK) - BLUE
#define SW_TX            6      // SoftwareSerial receive pin - BROWN
#define SW_RX            7      // SoftwareSerial transmit pin - YELLOW
#define DRIVER_ADDRESS   0b00   // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f           // SilentStepStick series use 0.11 ...and so does my fysetc TMC2209 (?)

// Create a new instance of the AccelStepper class:
SoftwareSerial SoftSerial(SW_RX, SW_TX);                          // Be sure to connect RX to TX and TX to RX between both devices
TMC2209Stepper driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);   // Create TMC driver
AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin);
 
void setup() {
  // Set the maximum speed in steps per second:
  SoftSerial.begin(11520);           // initialize software serial for UART motor control
  driver.beginSerial(11520);      // Initialize UART

  driver.begin();                                                                                                                                                                                                                                                                                                                            // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(2500);        // Set motor RMS current
  driver.microsteps(256);         // Set microsteps

  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);     // Needed for stealthChop

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  // pinMode(ms1Pin, OUTPUT);
  // pinMode(ms2Pin, OUTPUT);

  stepper.setMaxSpeed(8000);
  stepper.setSpeed(1000);        
  stepper.setEnablePin(enablePin);
  // digitalWrite(ms1Pin, HIGH);
  // digitalWrite(ms2Pin, HIGH);
  stepper.disableOutputs();

  Serial.begin(115200);

}

// 100 steps half circle full step
// 1600 steps half circle 1/16
long int t1, t2, d;

 
void loop() 
{ 
  // Set the current position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor forward at 200 steps/second until the motor reaches 400 steps (2 revolutions):
  nSpeed = 600;
  Serial.print("Running at speed:");
  Serial.println(nSpeed);
  t1 = millis();
  while(stepper.currentPosition() != 1600)
  {
    stepper.setSpeed(nSpeed);
    stepper.runSpeed();
  }
  d = (millis() - t1);
  Serial.print("Duration: ");
  Serial.print(d);
  Serial.println(" ms.");
 
  delay(1000);
 
  // Reset the position to 0:
  stepper.setCurrentPosition(0);
 
  // Run the motor backwards at 600 steps/second until the motor reaches -200 steps (1 revolution):
  nSpeed = -6.6;
  Serial.print("Running at speed:");
  Serial.println(nSpeed);
  t1 = millis();

  while(stepper.currentPosition() != -1600) 
  {
    stepper.setSpeed(nSpeed);
    stepper.runSpeed();
  }
  d = millis() - t1;
  Serial.print("Duration: ");
  Serial.print(d);
  Serial.println(" ms.");
 
  // delay(1000);
 
  // // Reset the position to 0:
  // stepper.setCurrentPosition(0);
 
  // // Run the motor forward at 400 steps/second until the motor reaches 600 steps (3 revolutions):
  // while(stepper.currentPosition() != 600)
  // {
  //   stepper.setSpeed(400);
  //   stepper.runSpeed();
  // }
 
  // delay(3000);
}
