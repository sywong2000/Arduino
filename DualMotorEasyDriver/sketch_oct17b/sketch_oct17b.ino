
int ENPin = 8;
int dirPin1 = 5;
//int dirPin1 = 3;
int stepperPin1 = 2;

int dirPin2 = 6;
//int dirPin2 = 7;
int stepperPin2 = 3;
//int stepperPin2 = 6;

int msx1pin = 4;
int msx2pin = 5;
int msy1pin = 8;
int msy2pin = 9;

int analogPin = A4;
uint32_t LastStepTime = 0;
uint32_t CurrentTime = 0;

#define  MAX_SPEED 500
#define  MIN_SPEED 0.1

#define MIN_RPMS                10.0
#define MAX_RPMS                200.0
#define STEPS_PER_REV         200
#define MICROSTEPS_PER_STEP     8
static float MICROSECONDS_PER_MICROSTEP   (1000000/(STEPS_PER_REV * MICROSTEPS_PER_STEP)/(MIN_RPMS / 60));

void setup() {
  Serial.begin(9600);
  pinMode(ENPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepperPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepperPin2, OUTPUT);
  digitalWrite(ENPin,LOW);
}

void step(boolean dir,int steps){
  digitalWrite(dirPin1,dir);
  digitalWrite(dirPin2,dir);
  delay(50);
  for(int i=0;i<steps;i++){
    digitalWrite(stepperPin1, HIGH);
    digitalWrite(stepperPin2, HIGH);
    delayMicroseconds(100);
    //delay(1);
    digitalWrite(stepperPin1, LOW);
    digitalWrite(stepperPin2, LOW);
    delayMicroseconds(100);
    //delay(1);
  }
}

void loop(){
  
  step(true,1600*4);
  delay(100);
  step(false,1600*4);
  delay(100);
}
