#include <Servo.h>
#include "Bounce2.h"

int BatteryS = 3;
#define MotorKV 2300 
int MaxMotorSpeed = 2000; 

#define PIN_REVTRIGGER 3 
#define PIN_MAINMOTOR1 9  
#define PIN_MAINMOTOR2 10
#define PIN_SAFETY 2
#define PIN_SPEEDPOT A3

// Servo Objects
Servo MainMotor1; 
Servo MainMotor2;


// Acceleration Time (Not used - go max)
long AccelerateTime = 0; //ms. Start with 200ms for 4s /// Not used

// Deceleration Time
long DecelerateTime = 3000; //ms IF you have a bLHeli_32 OR S set this to 1000 otherwise 6000

// Servo Floor Speed
long MinMotorSpeed = 1000;

// Current Motor Speed
long CurrentMotorSpeed = MinMotorSpeed;

// Use this as a placeholder when motor direction changes before reaching end travel, and when lowering max speed 
long InterruptedMotorSpeed = 0;

// Speed Adjustment
long MaxMotorSpeedCeiling; //use this to remember the absolute max motor speed.
byte SetMaxSpeed = 100; // in percent.

// Track changes to Rev Trigger State
unsigned long TimeLastTriggerChanged = 0;
unsigned long TimeLastTriggerPressed = 0;
unsigned long TimeLastTriggerReleased = 0;

// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool SafetyPressed = false; // Mag Release is Depressed

// Main Motor Command
bool CommandRev = false;
bool PrevCommandRev = false; // So we can keep track of changes

// Debounce Variables
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce RevTriggerBounce = Bounce();
Bounce SafetySwitchBounce = Bounce();


// System Modes
#define MODE_SAFETY 1
#define MODE_NORMAL 4
int CurrentSystemMode = MODE_NORMAL;


void setup() 
{
  float MotorRPM;

  // Set up comms
  Serial.begin(57600);
  Serial.println( F("Booting.. ") );   

  Serial.println( F("Configuring Debouncing") );
  pinMode(PIN_REVTRIGGER, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_REVTRIGGER );
  RevTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_SAFETY, INPUT_PULLUP);
  SafetySwitchBounce.attach( PIN_SAFETY );
  SafetySwitchBounce.interval( DebounceWindow );

  Serial.println( F("Debouncing Configured") );

  // Set up the pot
  pinMode( PIN_SPEEDPOT, INPUT );

  Serial.println( F("Initialising ESC") );
  // Set up motors
  MainMotor1.attach(PIN_MAINMOTOR1);
  MainMotor2.attach(PIN_MAINMOTOR2);
  // Arm ESC's
  MainMotor1.writeMicroseconds(MinMotorSpeed);
  MainMotor2.writeMicroseconds(MinMotorSpeed);
  delay(3000);   // Wait for ESC to initialise (7 seconds)
  Serial.println( F("ESC Initialised") );

  MaxMotorSpeedCeiling = MaxMotorSpeed;
  MotorRPM = BatteryS * MotorKV * 3.7;
  DecelerateTime = (float)(DecelerateTime) * (MotorRPM / 33300);

  Serial.println( F("Booted.") );
}


/*
 * Read the sensor / button state for all buttons, including software debounce.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  SafetySwitchBounce.update(); // Update the pin bounce state
  SafetyPressed = (SafetySwitchBounce.read());

}

/*
 * Slow the main motors to 0
 */
void RevDown()// RevDown 
{
  // Don't do anything if the motor is already stopped.
  if( CurrentMotorSpeed == MinMotorSpeed )
  {
    return;
  }

  if( DecelerateTime > 0 )
  {
    unsigned long CurrentTime = millis(); // Need a base time to calcualte from
    long SpeedRange = (MaxMotorSpeedCeiling - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
    long RampDownPerMS = SpeedRange / DecelerateTime;  // This is the number of units per S that we need to subtract
    unsigned long ActualDecelerationMS = CurrentTime - TimeLastTriggerReleased; // The number of MS in deceleration
    int NewMotorSpeed = MaxMotorSpeed - (ActualDecelerationMS * RampDownPerMS / 1000); // Calclate the new motor speed..

    // Now we need to take into account acceleration while braking
    if( NewMotorSpeed > CurrentMotorSpeed ) 
    {
      if( InterruptedMotorSpeed == 0 ) // Save the current motor speed for later comparison
      {
        InterruptedMotorSpeed = CurrentMotorSpeed;
        NewMotorSpeed = CurrentMotorSpeed;
      }
      else
      {
        NewMotorSpeed = InterruptedMotorSpeed - (MaxMotorSpeed - NewMotorSpeed);
      }      
    }

    if( NewMotorSpeed < MinMotorSpeed ) NewMotorSpeed = MinMotorSpeed; // Just in case we overshoot...

    if( (NewMotorSpeed - MinMotorSpeed) < (int)((float)MinMotorSpeed / 100 * 5) )
    {
      NewMotorSpeed = MinMotorSpeed; // We are within 5% of total stop, so just shut it down.
    }
    
    CurrentMotorSpeed = NewMotorSpeed;
  }
  else
  {
    // Immediately stop the motor
    CurrentMotorSpeed = MinMotorSpeed;
  }
}

/*
 * Run the main motors.
 */
void RevUp()
{

  // Don't do anything if the motor is already running.
  if( CurrentMotorSpeed == MaxMotorSpeed )
  {
    return;
  }

  if( AccelerateTime > 0 )
  {
    unsigned long CurrentTime = millis(); // Need a base time to calcualte from
    long SpeedRange = (MaxMotorSpeedCeiling - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
    long RampUpPerMS = SpeedRange / AccelerateTime;  // This is the number of units per S that we need to subtract
    unsigned long ActualAccelerationMS = CurrentTime - TimeLastTriggerPressed; // The number of MS in acceleration
    int NewMotorSpeed = MinMotorSpeed + (ActualAccelerationMS * RampUpPerMS / 1000); // Calclate the new motor speed..

    // Now we need to take into account acceleration while braking
    if( NewMotorSpeed < CurrentMotorSpeed ) 
    {
      if( InterruptedMotorSpeed == 0 ) // Save the current motor speed for later comparison
      {
        InterruptedMotorSpeed = CurrentMotorSpeed;
        NewMotorSpeed = CurrentMotorSpeed;
      }
      else
      {
        NewMotorSpeed = NewMotorSpeed - MinMotorSpeed + InterruptedMotorSpeed;
      }
    }

    if( NewMotorSpeed > MaxMotorSpeed ) NewMotorSpeed = MaxMotorSpeed; // Just in case we overshot...

    if( (NewMotorSpeed - MaxMotorSpeed) > (int)((float)MaxMotorSpeed / 100 * 95) )
    {
      NewMotorSpeed = MaxMotorSpeed; // We are within 5% of total full, so just wind it open.
    }
    
    CurrentMotorSpeed = NewMotorSpeed;
  }
  else
  {
    // Immediately hit the motor
    CurrentMotorSpeed = MaxMotorSpeed;
  }
}

/*
 * Account for changes to the rotary encoder and feed in a new max motor speed.
 */
void ProcessManualSpeedControl()
{
  static byte LastSetMaxSpeed = 100;
  int CurrentPotInput = 0;

  if( CommandRev )
  {
    CurrentPotInput = analogRead( PIN_SPEEDPOT );
    SetMaxSpeed = map( CurrentPotInput, 0, 1023, 35, 110 ); // Pull out of bounds so that the ends of the pot are zones.   
  }

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  SetMaxSpeed = constrain( SetMaxSpeed, 45, 100 ); // Constrain between 10% and 100%

  MaxMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeedCeiling );  // Keep processing against the max theoretical motor speed.

  LastSetMaxSpeed = SetMaxSpeed;

  // Need to simulate pressing the trigger to get the smooth ramping
  if( CommandRev ) 
  {
    TimeLastTriggerPressed = millis();   
    InterruptedMotorSpeed = 0;
  }

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

}

void ProcessRevCommand()
{


  CommandRev = RevTriggerPressed; // Accept the trigger input 
}

void loop() 
{

  ProcessButtons();
  ProcessSystemMode();

  // Only process firing input commands when in MODFE_NORMAL.. Otherwise allow for current cycle to finish.
  if( CurrentSystemMode == MODE_NORMAL )
  {
    // Perform command logic
    ProcessRevCommand();   
  } 

  // Calculate new timing variables
  if( CommandRev != PrevCommandRev )
  {
    InterruptedMotorSpeed = 0; // Reset the interrupted motor speed
    TimeLastTriggerChanged = millis();
    PrevCommandRev = CommandRev;
    if( CommandRev )  // Was depressed, now pressed
    {
      TimeLastTriggerPressed = TimeLastTriggerChanged;
    }
    else // Was pressed, now depressed
    {
      TimeLastTriggerReleased = TimeLastTriggerChanged;
    }
  }

  ProcessManualSpeedControl();

  // Command main motors
  if( CommandRev ) // Commanding the motors to spin
  {
    RevUp();
  }
  else // Commanding the motors to decelerate
  {
    RevDown();
  }  

  ProcessMainMotors();
}

/*
 * Communicate with the main motors
 */
void ProcessMainMotors()
{
  static long PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    MainMotor1.writeMicroseconds( CurrentMotorSpeed );
    MainMotor2.writeMicroseconds( CurrentMotorSpeed );
    // Debugging output
    Serial.println(CurrentMotorSpeed);

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

void ProcessSystemMode()
{

  if( SafetyPressed )
  {
    CurrentSystemMode = MODE_SAFETY;
    CommandRev = false;
    return;
  }
  
  CurrentSystemMode = MODE_NORMAL;
}
