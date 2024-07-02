#include <AccelStepper.h>
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

AccelStepper stepper(1, stepPin, dirPin);
int position = 0;
int nStepSize = 40;
int nSpeed = 800;

// Create a new instance of the AccelStepper class:
// AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
 
void setup() {
  // Set the maximum speed in steps per second:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

  stepper.setMaxSpeed(8000);
  stepper.setSpeed(1000);        
  stepper.setEnablePin(enablePin);
  digitalWrite(ms1Pin, HIGH);
  digitalWrite(ms2Pin, HIGH);
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
  nSpeed = 200;
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
