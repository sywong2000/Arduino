#include <AccelStepper.h>
// The X Stepper pins
#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2
// The Y stepper pins
#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 6

#define SPEED_PIN A4
#define  MAX_SPEED 6000
#define  MIN_SPEED 0.1

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
//AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
void setup()
{  
  Serial.begin(115200);
    stepper1.setMaxSpeed(MAX_SPEED);
    stepper1.setAcceleration(12000.0);
    //stepper1.moveTo(100);
    
//    stepper2.setMaxSpeed(100.0);
//    stepper2.setAcceleration(100.0);
//    stepper2.moveTo(100);
}
void loop()
{
    static float current_speed = 0.0;         // Holds current motor speed in steps/second
  static int analog_read_counter = 1000;    // Counts down to 0 to fire analog read
  static char sign = 1;                     // Holds -1, 1 or 0 to turn the motor on/off and control direction
  static int analog_value = 0;              // Holds raw analog value.
  
  analog_value = analogRead(SPEED_PIN);
    current_speed = sign * ((analog_value/1023.0) * (MAX_SPEED - MIN_SPEED)) + MIN_SPEED;
    Serial.print(analog_value);
    Serial.print("-");
    Serial.print(current_speed);
    Serial.println();
    stepper1.setSpeed(current_speed);
    stepper1.runSpeed();
    
    // Change direction at the limits
//    if (stepper1.distanceToGo() == 0)
//        stepper1.moveTo(-stepper1.currentPosition());
//    if (stepper2.distanceToGo() == 0)
//        stepper2.moveTo(-stepper2.currentPosition());
//    stepper1.run();
//    stepper2.run();
}
