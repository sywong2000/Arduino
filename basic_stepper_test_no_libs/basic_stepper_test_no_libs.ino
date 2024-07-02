int enablePin = 2;
const int stepPin = 4;
const int dirPin = 3;
const int stepsPerRevolution = 300;
 
void setup()
{
  // Declare pins as Outputs
  pinMode(enablePin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(enablePin, LOW);
}
void loop()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);
 
  // Spin motor slowly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  delay(1000); // Wait a second
  
  // Set motor direction counterclockwise;
  digitalWrite(dirPin, LOW);
 
  // Spin motor quickly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(8000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(8000);
  }
  delay(1000); // Wait a second
}