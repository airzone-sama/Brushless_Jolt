#include <Bounce2.h>
#include <EEPROM.h>

// Pin Definitions
#define PIN_TRIGGER_FULL 4
#define PIN_TRIGGER_REV 3
#define PIN_PUSHER_FET 6
#define PIN_ROFPOT A1
#define PIN_SPEEDPOT A0
#define PIN_MOTOR_A 9
#define PIN_MOTOR_B 10


// Configuration Options
byte BatteryS = 3;
byte BurstSize = 2;
byte TargetDPSBurst = 10;
byte TargetDPSAuto = 10;
byte MotorSpeedFull = 50; // For full-pull
byte MotorSpeedHalf = 30; // For half-pull
#define POT_READ_INTERVAL 100

// Modes
#define MODE_LOW_BATT 0
#define MODE_NORMAL 1
byte SystemMode = MODE_NORMAL;


// Pusher Controls
// Pusher 3S
#define PULSE_ON_TIME_3S 35
#define PULSE_RETRACT_TIME_3S 85
// Pusher 4S
#define PULSE_ON_TIME_4S 25   //25
#define PULSE_RETRACT_TIME_4S 85   //85
int PulseOnTime;
int PulseRetractTime;
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
byte CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;


// Firing Controls
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_IDLE 4
byte CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
byte ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
long ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
byte TargetDPS = 10; // This is what the solenoid will operate at. 
bool RequestShot = false; // Set to true to request the firing sequence to commence
bool RequestAutoStop = false; // Set to true to stop Full Auto


// Motor Controls
#define MOTOR_SPINUP_LAG 500 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINDOWN_3S 3000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
#define MOTOR_MAX_SPEED 2000
int MaxMotorSpeed = MOTOR_MAX_SPEED;
int DecelerateTime = 0;
int AccelerateTime = 0;
long MotorRampUpPerMS = 0;
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireFullTriggerPressed = false; // Fire Trigger is Depressed


// Serial Comms
#define SERIAL_INPUT_BUFFER_MAX 25
char SerialInputBuffer[SERIAL_INPUT_BUFFER_MAX];
byte SavedMode = FIRE_MODE_SINGLE;
byte SavedBurstSize = 0;
bool HasSavedMode = false;
bool AutoFire = false;
int AutoFireMotorSpeed = 0;


void setup() {
  // Setial startup
  Serial.begin( 57600 ); // Debugging
  Serial.println( F("Booting") );

  // Set up debouncing
  Serial.println( F("Configuring Debouncing") );  
  
  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  Serial.println( F("Debouncing Configured") );

  // Set up the Pots
  pinMode( PIN_SPEEDPOT, INPUT );  
  pinMode( PIN_ROFPOT, INPUT );  

  // Setup Motor Outputs
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_MOTOR_A, OUTPUT );
  pinMode( PIN_MOTOR_B, OUTPUT );
  digitalWrite( PIN_MOTOR_A, LOW );
  digitalWrite( PIN_MOTOR_B, LOW );
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );

  // Setup Pusher Outputs
  Serial.println( F("Configuring Pusher FET") );
  pinMode( PIN_PUSHER_FET, OUTPUT );
  digitalWrite( PIN_PUSHER_FET, LOW );  


  Serial.println( F("Detecting Battery") );
  // Setup Battery
  if( BatteryS == 3 )
  {
    Serial.println( F("Configuring for 3S Battery") );
    PulseOnTime = PULSE_ON_TIME_3S;
    PulseRetractTime = PULSE_RETRACT_TIME_3S;

    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;
  }
  else
  {
    Serial.println( F("Configuring for 4S Battery") );
    PulseOnTime = PULSE_ON_TIME_4S;
    PulseRetractTime = PULSE_RETRACT_TIME_4S;

    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;
  }

  SystemMode = MODE_NORMAL;
  CalculateRampRates(); 

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );
  RevTriggerBounce.update();
  while( RevTriggerBounce.read() == LOW )
  {
    delay(10);
    RevTriggerBounce.update();
  }
  delay(10);

  Serial.println( F("Booted") );
}

/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  


  Serial.print( F("Ramp Up per MS = ") );
  Serial.println( MotorRampUpPerMS );

  Serial.print( F("Ramp Down per MS = ") );
  Serial.println( MotorRampDownPerMS );
}

// Updates the PWM Timers
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2) + 2; // Adjust for the prescalar
  OCR1A = NewSpeed;
  OCR1B = NewSpeed;
}


void loop() {
  ProcessButtons(); // Get User and Sensor input
  ProcessSystemMode(); // Find out what the system should be doing

  // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }
  
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
    
  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

  // Process Firing Controls
  ProcessFiring();
  ProcessSolenoid();
}

bool ProcessSerialInput()
{
  bool SerialDataAvailable = false;
  if( Serial.available() != 0 )
    SerialDataAvailable = true;
    
  if( !SerialDataAvailable ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( (Serial.available() > 0) )
  {
    char NextByte = 0;
    if( Serial.available() != 0 )
      NextByte = Serial.read();
    else
      NextByte = 0; //WTF is this happening??

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      case '?': // Presume help - Simulate DS
        SerialInputBuffer[0] = 'D';
        SerialInputBuffer[1] = 'S';
        return true;
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= SERIAL_INPUT_BUFFER_MAX ) CurrentBufferPosition = (SERIAL_INPUT_BUFFER_MAX - 1);  // Capture Overflows.
    }
  }

  return false;
}

void ProcessSerialCommand()
{
  char CommandHeader[3]; // Place the header into this buffer
  // Copy it using a lazy way
  CommandHeader[0] = SerialInputBuffer[0];
  CommandHeader[1] = SerialInputBuffer[1];
  CommandHeader[2] = 0;

  // Run Motor Full Command - FM
  if( (strcmp( CommandHeader, "FM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_FULL;
  }

  // Run Motor Half Command - HM
  if( (strcmp( CommandHeader, "HM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_HALF;
  }

  // Stop Motor Command - SM
  if( (strcmp( CommandHeader, "SM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_NONE;
  }

  // Single Fire Full Command - SF
  if( (strcmp( CommandHeader, "SF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }

  // Single Fire Half Command - SH
  if( (strcmp( CommandHeader, "SH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
  }

  // Burst Fire Full Command - BF
  if( (strcmp( CommandHeader, "BF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }  

  // Burst Fire Full Command - BH
  if( (strcmp( CommandHeader, "BH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
  }  

  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    Serial.println( F("#JS-OK$") );
  } 

  // Display Settings - DS
  if( strcmp( CommandHeader, "DS" ) == 0 )
  {
    Serial.println( F("--------------------") );
    Serial.println( F("Blaster Settings:") );
    Serial.println( F("--------------------") );

    Serial.print( F("Full Trigger Power = ") );
    Serial.println( MotorSpeedFull );

    Serial.print( F("Burst Size = ") );
    Serial.println( BurstSize );
    Serial.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );

    Serial.print( F("ROF Auto = ") );
    Serial.println( TargetDPSAuto );
    Serial.println( F("--------------------\n") );

    Serial.println( F("--------------------") );
    Serial.println( F("Blaster Status:") );
    Serial.println( F("--------------------") );

    Serial.print( F("Motor Ramp Up Rate = ") );
    Serial.println( MotorRampUpPerMS );
    
    Serial.print( F("Motor Ramp Down Rate = ") );
    Serial.println( MotorRampDownPerMS );

    Serial.print( F("Battery S = ") );
    Serial.println( BatteryS );

    Serial.print( F("System Mode = ") );
    Serial.println( SystemMode );
    
    Serial.print( F("Full Trigger State = ") );
    Serial.println( FireFullTriggerPressed );

    Serial.print( F("Rev Trigger State = ") );
    Serial.println( RevTriggerPressed );

    Serial.println( F("--------------------\n") );
  } 
      
}

/*
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{
  
  static bool PreviousRevTriggerPressed = false; // Keep track of the human input
  static unsigned long RevStartTime = 0;

  if( !(SystemMode == MODE_NORMAL) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( (PreviousRevTriggerPressed != RevTriggerPressed) && (SystemMode == MODE_NORMAL) )
  {
    // Human has taken control - disengage autopilot but only when not in config mode
    PreviousRevTriggerPressed = RevTriggerPressed;
    RevStartTime = millis();
    AutoRev = false;
  }

  // Rev can occur for 30 seconds only
  #define MAX_REV_TIME 10000
  
  if( !AutoRev )
  {
    if( RevTriggerPressed )
    {
      if( ( millis() - RevStartTime ) > MAX_REV_TIME )
      {
        CommandRev = COMMAND_REV_NONE;
      }
      else
      {
        CommandRev = COMMAND_REV_FULL;  
      }
    }
    else
    {
      //CommandRev = COMMAND_REV_NONE;  // This will be handled elsewhere
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

// Update the motors with the new speed
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    Serial.println(CurrentMotorSpeed);

    // Use this for Servo Library
    if( CurrentMotorSpeed > MOTOR_MAX_SPEED )
      UpdatePWM( MOTOR_MAX_SPEED );
    else
      UpdatePWM( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Calculate the desired motor speed
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..  

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    //Serial.println( MSElapsed );
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

  Serial.print( F("New target speed = ") );
  Serial.println( TargetMotorSpeed );
}

// Process the firing request and queue up some darts to fire.
void ProcessFiring()
{
  if( !(SystemMode == MODE_NORMAL) ) // Finish off the stroke unless in running or in ROF config mode
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_IDLE;
    return;
  }

  static unsigned long InitiatedAutoFire = 0;
  if( AutoFire )
  {
    if( InitiatedAutoFire == 0 ) // Started auto fire process. Start spinning the motors
    {
      InitiatedAutoFire = millis();
      AutoRev = true;
      CommandRev = AutoFireMotorSpeed;
      return;
    }
    if( (millis() - InitiatedAutoFire) < MOTOR_SPINUP_LAG ) // Wait for the lag
    {
      return;
    }
    RequestShot = true;
  }
  else
  {
    InitiatedAutoFire = 0;
  }
    

  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) ) // Don't try and push a dart into stationary flywheels..
  {
    if( ProcessingFireMode == FIRE_MODE_AUTO )
    {
      ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
      LastSolenoidCycleStarted = millis();
      ShotsToFire = 0;
      ExecuteFiring = true;
    }
    return;
  }

  // Requesting Shot while we were doing nothing special
  if( RequestShot && (ProcessingFireMode == FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = 99; // Single fire mode is always flat out
        Serial.println( F( "Start Single" ) );
        break;
      case FIRE_MODE_BURST:
        ShotsToFire = BurstSize; // Set the burst size
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = TargetDPSBurst;
        Serial.println( F( "Start Burst" ) );
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        TargetDPS = TargetDPSAuto;
        Serial.println( F( "Start Full Auto" ) );
        break;        
    }
  }
  else if( RequestAutoStop && (ProcessingFireMode == FIRE_MODE_AUTO) ) // Requesting Stop while firing in Full Auto 
  {
    Serial.println( F( "Stop Full Auto" ) );
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    LastSolenoidCycleStarted = millis();
    ExecuteFiring = true;

    if( CurrentSolenoidCyclePosition == SOLENOID_CYCLE_PULSE )
    {
      ShotsToFire = 1;
    }
    else
    {
      ShotsToFire = 0;
    }
  }
}

// Make the solenoid do the things it's supposed to do
void ProcessSolenoid()
{
  if( !ExecuteFiring ) // Just skip if there is no firing to execute
  {
    return;
  }

  // Calculate duty cycle whenever the target changes.
  static byte PrevTargetDPS = 0;
  if( (PrevTargetDPS != TargetDPS) && ((CurrentSolenoidCyclePosition == SOLENOID_CYCLE_IDLE) || ( CurrentSolenoidCyclePosition == SOLENOID_CYCLE_COOLDOWN) ) )
  {
    PrevTargetDPS = TargetDPS;
    if( TargetDPS == 99 ) // Full rate
    {
      TimeBetweenShots = 0;
    }
    else
    {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = 1000 - TotalPulseOverhead;
      if( FreeMS <= 0 )
      {
        TimeBetweenShots = 0; // Pusher won't achieve this rate
      }
      else
      {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  // We actually have nothing to do
  if( ProcessingFireMode == FIRE_MODE_IDLE )
  {
    return; // Solenoid is idling.
  }

  // We are apparently supposed to fire 0 darts... Typically for end-of-firing scenarios
  if( (ShotsToFire == 0) && (ProcessingFireMode != FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    Serial.println( F("Finished shooting") );
    ExecuteFiring = false;
    if( AutoFire )
    {
      AutoFire = false;
      if( HasSavedMode )
      {
        BurstSize = SavedBurstSize;
        CurrentFireMode = SavedMode;
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        HasSavedMode = false;
      }
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
      RequestShot = false;
      AutoRev = false;
      HasSavedMode = false;
    }
    return;    
  }

  // Last check to ensure the motors are running before we send a dart into them
  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    Serial.println( F("Shooting Aborted - Motors not running") );
    ExecuteFiring = false;
    return;        
  }

  // Pulse solenoid on high
  if( (millis() - LastSolenoidCycleStarted) < PulseOnTime )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE )
    {
      //Serial.println( F("Start Pulse") );
      /*
      if( (SystemMode != MODE_NORMAL) ) // Don't fire unless the system m ode is normal
      {
        ShotsToFire = 0;
        Serial.println( F("Mag Out!!") );
        return;
      }
      */
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    digitalWrite( PIN_PUSHER_FET, HIGH );
    return;
  }

  // Release solenoid for retraction
  if( (millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime) )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_RETRACT )
    {
      //Serial.println( F("End Pulse") );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    digitalWrite( PIN_PUSHER_FET, LOW );
    return;      
  }  

  // Wait for the Global Cool Down... i.e. ROF adjustment
  if((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots))
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_COOLDOWN )
    {
      //Serial.println( F("Cooling Down") );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    digitalWrite( PIN_PUSHER_FET, LOW );
    return;      
  }

  // We have completed a single solenoid cycle. Return to idle, ready for the next shot.
  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
  ShotsToFire -= 1;
  LastShot = millis();
  LastSolenoidCycleStarted = millis();
  Serial.println( F("Bang!!") );  
}


/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read()); 

  // Determine the current firing mode
  if( !AutoFire )
  {
    CurrentFireMode = FIRE_MODE_AUTO;
  }

  // Handle the Pots

  //MotorSpeedFull
  //TargetDPSAuto
  static unsigned long LastPotRead = 0;
  if( millis() - LastPotRead > POT_READ_INTERVAL )
  {
    int CurrentPotInput = 0;
    CurrentPotInput = analogRead( PIN_SPEEDPOT );
    CurrentPotInput = map( CurrentPotInput, 0, 1023, 25, 110 ); // Pull out of bounds so that the ends of the pot are zones.   
    if( (CurrentPotInput < (MotorSpeedFull + 2)) && (CurrentPotInput > (MotorSpeedFull - 2) ) )
    {
      CurrentPotInput = MotorSpeedFull;
    }
    CurrentPotInput = constrain( CurrentPotInput, 30, 100 );
    if( CurrentPotInput != MotorSpeedFull )
    {
      MotorSpeedFull = CurrentPotInput;
      TimeLastMotorSpeedChanged = millis();
      Serial.print( F( "New Speed Pot Value = " ) );
      Serial.println( MotorSpeedFull );
    }

    CurrentPotInput = analogRead( PIN_ROFPOT );
    CurrentPotInput = map( CurrentPotInput, 0, 1023, 1, 13 ); // Pull out of bounds so that the ends of the pot are zones.   
    CurrentPotInput = constrain( CurrentPotInput, 3, 10 );
    if( CurrentPotInput != TargetDPSAuto )
    {
      TargetDPSAuto = CurrentPotInput;
      Serial.print( F( "New ROF Pot Value = " ) );
      Serial.println( TargetDPSAuto );
      if( (CommandRev == COMMAND_REV_FULL) && !AutoFire )
      {
        TargetDPS = TargetDPSAuto;
      }
    }
    
    LastPotRead = millis();
  }


  // Firing controls here

  static unsigned long TriggerLastPulled = 0;
  if( (FireFullTriggerBounce.fell()) && (CommandRev == COMMAND_REV_FULL) )
  {
    RequestShot = true;
    RequestAutoStop = false;
    Serial.println( "START FIRING" ); 
  }
  if( (FireFullTriggerBounce.rose()) && (CommandRev == COMMAND_REV_FULL) )
  {
    RequestAutoStop = true;
    Serial.println( "STOP FIRING" );
  }

  //RequestShot = FireFullTriggerBounce.fell(); // Programatically keep track of the request for a shot
  //RequestAutoStop = FireFullTriggerBounce.rose();
  
}

// We are toggline between different system states here..
// Also handle the blasted configuration controls here... Because there's no special configuration screen
void ProcessSystemMode()
{
  static byte LastSystemMode = MODE_NORMAL;

  SystemMode = MODE_NORMAL;


  if( LastSystemMode != SystemMode )
  {
    Serial.print( F("New System Mode = ") );
    Serial.println( SystemMode );
    LastSystemMode = SystemMode;
  }
}
