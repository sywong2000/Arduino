int powerPin = 7; //STDBY
int PWMPin = 4; // PWM
int AIN1Pin = 6; //AIN1
int AIN2Pin = 5; //AIN2

boolean useSleep = true; // true= use sleep pin, false = use enable pin

// maximum speed is 160pps which should be OK for most
// tin can steppers
#define MAXSPEED 160
#define SPEEDMULT 3

#define MAXCOMMAND 8

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
long CurrentPos, TargetPos;
int eoc = 0;
int idx = 0;
boolean isMoving = false;
boolean bNeedMove = false;
char tempString[10];
long stepInterval = 1000; //micros

    typedef enum
    {
        DIRECTION_CCW = 0,
        DIRECTION_CW = 1
    } Direction;

Direction MotorDirection= DIRECTION_CW;

void setup()
{
  Serial.begin(9600);
	pinMode(powerPin,OUTPUT);
	pinMode(AIN1Pin,OUTPUT);
	pinMode(AIN2Pin,OUTPUT);
  pinMode(PWMPin,OUTPUT);
 
  digitalWrite(powerPin,HIGH);
  digitalWrite(PWMPin,HIGH);
  digitalWrite(AIN1Pin,LOW);
  digitalWrite(AIN2Pin,LOW);
	memset(line, 0, MAXCOMMAND);
}


//

//
unsigned long time, lastStepTime;
boolean ledon = false;

void loop()
{
  if (isMoving)
  {
    if (micros() - lastStepTime>=stepInterval)
    {
      long stepsTaken = (micros() - lastStepTime)/stepInterval;
      if (MotorDirection==DIRECTION_CW)
      {
        CurrentPos += stepsTaken;
        if (CurrentPos >= TargetPos)
        {
          digitalWrite(AIN1Pin, LOW);
          digitalWrite(AIN2Pin, LOW);
          isMoving = false;
        }
      }
      else
      {
        CurrentPos -= stepsTaken;
        if (CurrentPos <= TargetPos)
        {
          digitalWrite(AIN1Pin, LOW);
          digitalWrite(AIN2Pin, LOW);
          isMoving = false;
        }
      }
      lastStepTime = micros();
    }
  }
  
	while (Serial.available() && !eoc) 
	{
		inChar = Serial.read();
		if (inChar != '#' && inChar != ':') {
			line[idx++] = inChar;
			if (idx >= MAXCOMMAND) {
				idx = MAXCOMMAND - 1;
			}
		}
		else {
			if (inChar == '#') {
				eoc = 1;
			}
		}
	} // end while Serial.available()

  
  
  
	// we may not have a complete command yet but there is no character coming in for now and might as well loop in case stepper needs updating
	// eoc will flag if a full command is there to act upon

	// process the command we got
	if (eoc) {
		memset(cmd, 0, MAXCOMMAND);
		memset(param, 0, MAXCOMMAND);

		int len = strlen(line);
		if (len >= 2) {
			strncpy(cmd, line, 2);
		}

		if (len > 2) {
			strncpy(param, line + 2, len - 2);
		}

		memset(line, 0, MAXCOMMAND);  //WvB sets the string line to 0
		eoc = 0;
		idx = 0;

		//now execute the command
		//Immediately stop any focus motor movement. returns nothing
		//code from Quickstop example. This is blocking
   
		if (!strcasecmp(cmd, "FQ"))    // WvB: "FQ" = Halt motor, position is retained
		{
      digitalWrite(AIN1Pin, LOW);
      digitalWrite(AIN2Pin, LOW);
      isMoving = false;
		}

		//Go to the new position as set by the ":SNYYYY#" command. returns nothing    // initiate a move
		//turn stepper on and flag it is running
		// is this the only command that should actually make the stepper run ?
		if (!strcasecmp(cmd, "FG")) {  //WvB: start motor, to NewPosition
      if (TargetPos!=CurrentPos)
      {
        MotorDirection= (TargetPos > CurrentPos)?DIRECTION_CW:DIRECTION_CCW;
        // start the motor
        lastStepTime = micros();
        digitalWrite(MotorDirection==DIRECTION_CW?AIN1Pin:AIN2Pin,HIGH);
        digitalWrite(MotorDirection==DIRECTION_CW?AIN2Pin:AIN1Pin,LOW);
        isMoving = true;
      }
		}

		//Returns the temperature coefficient where XX is a two-digit signed (2�s complement) hex number.
		//hardcoded
		if (!strcasecmp(cmd, "GC")) {
			Serial.print("02#");
		}

		//Returns the current stepping delay where XX is a two-digit unsigned hex number. See the :SD# command for a list of possible return values.
		//hardcoded for now
		// might turn this into AccelStepper acceleration at some point
		if (!strcasecmp(cmd, "GD")) {
			Serial.print("02#");
		}

		//Returns "FF#" if the focus motor is half-stepped otherwise return "00#"
		//hardcoded
		if (!strcasecmp(cmd, "GH")) {
			Serial.print("00#");
		}

		//Returns "00#" if the focus motor is not moving, otherwise return "01#",
		//AccelStepper returns Positive as clockwise
		if (!strcasecmp(cmd, "GI")) {
			if (!isMoving) {
				Serial.print("00#");
			}
			else {
				Serial.print("01#");
			}
		}

		//Returns the new position previously set by a ":SNYYYY" command where YYYY is a four-digit unsigned hex number.
		if (!strcasecmp(cmd, "GN")) {
			
			sprintf(tempString, "%04X", TargetPos);
			Serial.print(tempString);
			Serial.print("#");
		}

		//Returns the current position where YYYY is a four-digit unsigned hex number.
		if (!strcasecmp(cmd, "GP")) {
			sprintf(tempString, "%04X", CurrentPos);
			Serial.print(tempString);
			Serial.print("#");
		}

		//Returns the current temperature where YYYY is a four-digit signed (2�s complement) hex number.
		if (!strcasecmp(cmd, "GT")) {
			Serial.print("0020#");
		}

		//Get the version of the firmware as a two-digit decimal number where the first digit is the major version number, and the second digit is the minor version number.
		//hardcoded
		if (!strcasecmp(cmd, "GV")) {
			Serial.print("10#");
		}

		//Set the new temperature coefficient where XX is a two-digit, signed (2�s complement) hex number.
		if (!strcasecmp(cmd, "SC")) {
			//do nothing yet
		}

		//Set the new stepping delay where XX is a two-digit,unsigned hex number.
		if (!strcasecmp(cmd, "SD")) {
			//do nothing yet
		}

		//Set full-step mode.
		if (!strcasecmp(cmd, "SF")) {
			//do nothing yet
		}

		//Set half-step mode.
		if (!strcasecmp(cmd, "SH")) {
			//do nothing yet
		}

		//Set the new position where YYYY is a four-digit
		if (!strcasecmp(cmd, "SN")) {
			TargetPos = hexstr2long(param);
			// stepper.enableOutputs(); // turn the motor on here ??
			//WvB: this seems strange, motor is turned on here. There should be another command to do this
			//turnOn();
			//motor.moveTo(pos);

      if (TargetPos!=CurrentPos)
      {
        MotorDirection= (TargetPos > CurrentPos)?DIRECTION_CW:DIRECTION_CCW;
        // start the motor
        lastStepTime = micros();
        digitalWrite(MotorDirection==DIRECTION_CW?AIN1Pin:AIN2Pin,HIGH);
        digitalWrite(MotorDirection==DIRECTION_CW?AIN2Pin:AIN1Pin,LOW);
        isMoving = true;
      }

		}

		//Set the current position where YYYY is a four-digit unsigned hex number.
		if (!strcasecmp(cmd, "SP")) {
			CurrentPos = hexstr2long(param);
			//motor.setCurrentPosition(pos);
		}

	}// end if(eoc)


} // end loop

long hexstr2long(char *line) {
	long ret = 0;

	ret = strtol(line, NULL, 16);
	return (ret);
}
