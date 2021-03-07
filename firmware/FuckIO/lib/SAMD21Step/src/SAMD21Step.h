#ifndef SAMD21Stepper_h
#define SAMD21Stepper_h

#include <Arduino.h>

#define STEPPER_VERBOSE

// Struct to store all stepper relevant informations
typedef struct { 
  const int pulsePin;                   // Arduino pin number for the pulse pin. Must be a pin with a TCC timer output (see PinLookup). One timer can exclusivly serve only one pin.
  const int dirPin;                     // Arduino pin number of the direction signal. May be any pin.
  const int enablePin;                  // Arduino pin number of the driver enable signal. May be any pin. Set to -1 if not used.
  const bool enableActiveLow;           // Set to true if the enable signal is active low. For active high set to false.
  const int alarmPin;                   // Arduino pin number of the driver alarm signal. Set to -1 if not used. Expects an external pull-up, pull-down or tri-state input
  const bool alarmActiveHigh;           // Set to true if the alarm signal is active high. For active low set to false.
  const int homePin;                    // Arduino pin number of the home switch. Homing is mandatory. Expects an external pull-up, pull-down or tri-state input
  const bool homeActiveHigh;            // Set to true if the home signal is active high. For active low set to false.
  const bool invertPulse;               // Set to true if you need an active low pulse signal
  const float minPulseWidth;              // Minimum pulse width in µs
  const int maxVelocity;                // Maximum velocity in steps/sec
  const int maxAcceleration;            // Maximum accelceration & decceleratin in steps/sec^2
  const int minPosition;                // Minimum position of the axis in steps
  const int maxPosition;                // Maximum position of the axis in steps
  const int homePosition;               // The position of home. When the home switch is triggered _position will be set to this value. May be outside the minPosition and maxPosition boundary
  const bool invertDirection;           // Set to true to invert the movement direction
  const bool invertHoming;              // Set to true to invert the direction for searching the home switch
} StepperConfig;

// Enum to store the steppers current state
typedef enum {
  STEPPER_IDLE,                         // Stepper is not active and enable is false
  STEPPER_RUNNING,                      // Stepper is running. Motor is energized, but not necessarily moving
  STEPPER_ALARM,                        // Unexpected boundary violation or stepper driver is in fault mode. Consult you stepper driver manual to see why
  STEPPER_NOT_INITIALIZED,               // Config failed to initialize the stepper drive. Propably a wrong pulseDir-pin.
  STEPPER_FAILED
} StepperState;

// Enum to store current movement direction
typedef enum {
  DIRECTION_CCW = 0,  // Counter-Clockwise is counting down
  DIRECTION_CW  = 1   // Clockwise is counting up
} Direction;

// Table for access in timer-ISR
typedef struct {
  long int position;                    // Current position of the stepper
  Direction direction;                  // Rotation direction of motor (count position up or down)
  long int minSafePosition;             // Minimum boundary: If violated timer will be halted immideately
  long int maxSafePosition;             // Maximum boundary: If violated timer will be halted immideately
  bool homed;                           // flag for beeing homed
  bool fault;                           // Fault Flag for boundary violation
} PositionParameter;

// Table for looking up and storing values for TCCx
typedef struct {
  const Tcc* TCCx;                      // Pointer to timer
  const RwReg* REG_TCCx_CTRLA;          // Pointer to timer's CTRLA register
  const RwReg* REG_TCCx_WAVE;           // Pointer to timer's WAVE register
  const RwReg* REG_TCCx_PERB;           // Pointer to timer's PERB register
  const RwReg* REG_TCCx_INTENSET;       // Pointer to timer's INTERNSET register
  const IRQn_Type TCCx_IRQn;            // Pointer to IRQ register for timer
  const unsigned long int counterSize;  // Timer's counter size: 24 bits for TCC0 and TCC1, 16 bits for TCC2
  unsigned int TCCDiv;                  // Timer's clock divider: 1, 2, 4, 8, 16, 64, 256, or 1024
  unsigned long long int steps;         // Timer's periode (resolution): 2 to counterSize
  unsigned int dutyCycle;               // Timer's CC value: 2 to steps
  bool invertPulse;                     // fast aka normal aka single-slope PWM, True if pulse should be inverted
  bool enabled;                         // Shows if TCCx should be enabled
} TimerLookup;

static TimerLookup timerTable[] = {
  {TCC0, &REG_TCC0_CTRLA, &REG_TCC0_WAVE, &REG_TCC0_PERB, &REG_TCC0_INTENSET, TCC0_IRQn, 0xFFFFFF, 1, 500000, 250000, false, false},
  {TCC1, &REG_TCC1_CTRLA, &REG_TCC1_WAVE, &REG_TCC1_PERB, &REG_TCC1_INTENSET, TCC1_IRQn, 0xFFFFFF, 1, 500000, 250000, false, false},
  {TCC2, &REG_TCC2_CTRLA, &REG_TCC2_WAVE, &REG_TCC2_PERB, &REG_TCC2_INTENSET, TCC2_IRQn,   0xFFFF, 1,  50000,  25000, false, false}
};
static const unsigned int timerTableSize = sizeof(timerTable) / sizeof(timerTable[0]);

// Tables for looking up pin mappings etc. for different boards
typedef struct { 
  const int arduinoPin;                 // Arduino pin number
  const unsigned int port;              // Port of the SAMD21 pin
  const unsigned int samd21Pin;         // SAMD21 pin
  const unsigned int timer;             // Timer used for this pin
  const RwReg* REG_TCCx_CCBy;           // Pointer to count register used for this pin
  const unsigned long int pMux;         // Pin multiplexer for this pin
} PinLookup;

static const PinLookup pinTable[] = {
#if defined (ARDUINO_SAMD_NANO_33_IOT)
//Table begin
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0}, 
{-1, 0, 0, 0, 0, 0}, 
{-1, 0, 0, 0, 0, 0},
{ 4, PORTA,  7, 1, &REG_TCC1_CCB1, PORT_PMUX_PMUXO_E},
{ 5, PORTA,  5, 0, &REG_TCC0_CCB1, PORT_PMUX_PMUXO_E},    // PWM 2
{ 6, PORTA,  4, 0, &REG_TCC0_CCB0, PORT_PMUX_PMUXE_E},    // PWM 1
{ 7, PORTA,  6, 1, &REG_TCC1_CCB0, PORT_PMUX_PMUXE_E},    // Pulse
{ 8, PORTA, 18, 0, &REG_TCC0_CCB2, PORT_PMUX_PMUXE_F},
{-1, 0, 0, 0, 0, 0}, 
{-1, 0, 0, 0, 0, 0}, 
{11, PORTA, 16, 2, &REG_TCC2_CCB0, PORT_PMUX_PMUXE_E}, 
{12, PORTA, 19, 0, &REG_TCC0_CCB3, PORT_PMUX_PMUXO_F},
{13, PORTA, 17, 2, &REG_TCC2_CCB1, PORT_PMUX_PMUXO_E}
//Table end

#elif defined (ARDUINO_SAMD_ZERO) || \
      defined (ARDUINO_SAMD_FEATHER_M0)
//Table begin
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0},
{ 3, PORTA,  9, 0, &REG_TCC0_CCB1, PORT_PMUX_PMUXO_E},
{ 4, PORTA,  8, 0, &REG_TCC0_CCB0, PORT_PMUX_PMUXE_E},
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0},
{ 8, PORTA,  6, 1, &REG_TCC1_CCB0, PORT_PMUX_PMUXE_E},
{ 9, PORTA,  7, 1, &REG_TCC1_CCB1, PORT_PMUX_PMUXO_E},
{10, PORTA, 18, 0, &REG_TCC0_CCB2, PORT_PMUX_PMUXE_F},
{11, PORTA, 16, 2, &REG_TCC2_CCB0, PORT_PMUX_PMUXE_E},
{12, PORTA, 19, 0, &REG_TCC0_CCB3, PORT_PMUX_PMUXO_F},
{13, PORTA, 17, 2, &REG_TCC2_CCB1, PORT_PMUX_PMUXO_E}
//Table end

#elif defined (ARDUINO_SAMD_MKRZERO) || \
      defined (ARDUINO_SAMD_MKR1000) || \
      defined (ARDUINO_SAMD_MKRWIFI1010) || \
      defined (ARDUINO_SAMD_MKRFox1200) || \
      defined (ARDUINO_SAMD_MKRWAN1300) || \
      defined (ARDUINO_SAMD_MKRWAN1310) || \
      defined (ARDUINO_SAMD_MKRGSM1400) || \
      defined (ARDUINO_SAMD_MKRNB1500) || \
      defined (ARDUINO_SAMD_MKRVIDOR4000)
//Table begin
{-1, 0, 0, 0, 0, 0},
{-1, 0, 0, 0, 0, 0},
{ 2, PORTA, 10, 1, &REG_TCC1_CCB0, PORT_PMUX_PMUXE_E}, 
{ 3, PORTA, 11, 1, &REG_TCC1_CCB1, PORT_PMUX_PMUXO_E},
{ 4, PORTB, 10, 0, &REG_TCC0_CCB0, PORT_PMUX_PMUXE_F},
{ 5, PORTB, 11, 0, &REG_TCC0_CCB1, PORT_PMUX_PMUXO_F},
{ 6, PORTA, 20, 0, &REG_TCC0_CCB2, PORT_PMUX_PMUXE_F},
{ 7, PORTA, 21, 0, &REG_TCC0_CCB3, PORT_PMUX_PMUXO_F},
{ 8, PORTA, 16, 2, &REG_TCC2_CCB0, PORT_PMUX_PMUXE_E},
{ 9, PORTA, 17, 2, &REG_TCC2_CCB1, PORT_PMUX_PMUXO_E}
//Table end

#else
  #error Board not supported by SAMD21Stepper Library
#endif
};
static const unsigned int pinTableSize = sizeof(pinTable) / sizeof(pinTable[0]);

class Stepper {
  public:
    int begin(StepperConfig* stepconfig);
    int enableDriver(bool enabled);                     // 
    float getSpeed();                                   // returns the current speed of the motor in steps/sec
    int getPosition();                                  // returns the current position
    int status();                                       // returns the status of the stepper
    void setHome();                                     // sets the current position as home
    bool isHomed();                                     // returs true if drive has been succesfully homed
    bool doMove(int speed, Direction direction);        // moves the stepper with commanded steps/sec. False if state is not STEPPER_RUNNING
    int initTimer(float motionPeriode);                 // inits the timer perpherial with values derived from config parameteres and motion periode [ms]. Typically called when initializing the motion class.
    bool hasFault();                                    // checks the fault flag and acts accordingly
    StepperConfig* config;                              // Store the stepper config
  private:
    void _setClockDivider(unsigned int GCLKDiv);
    void _updateTimer(unsigned long long int steps);
    void _enableTimer(bool enabled);
    unsigned int _GCLKDiv = 1;                          // Main clock divider: 1 to 255 for TCC0 to TC3
    unsigned int _timerNumber = 0;                      // Timer number associated with this stepper instance
    StepperState _state = STEPPER_NOT_INITIALIZED;            // Current state of the stepper
};

#endif

