#include "SAMD21Step.h"

// volatile global variable array for access in timer-ISR
volatile PositionParameter positionTable[] {
    {0, DIRECTION_CW, 0, 0, false, false},
    {0, DIRECTION_CW, 0, 0, false, false},
    {0, DIRECTION_CW, 0, 0, false, false}
}; 

/**************************************************************************/
/*!
  @brief  Load the stepper configuration and checker whether a valid pulse 
  was used. Selects the correct timer peripherial and configures all stepper
  related pins as inputs & outputs. Enable the clocks for timer TCC0 to TC3
  with GCLKDiv = 1.
  @param stepconfig Struct of type StepperConfig which containes all relevant
  parameters to get the stepper moving.
  @returns State STEPPER_FAILED for invalid pulsePin, otherwise 
  STEPPER_NOT_INITIALIZED
*/
/**************************************************************************/
int Stepper::begin(StepperConfig* stepconfig) {
  //Copy pointer to config
  config = stepconfig;

  // Check if an acceptable pulse pin is used and determine which timer to use
  unsigned int i;
  for (i = 0; i < pinTableSize; i++) {
    if (pinTable[i].arduinoPin == config->pulsePin) {
      break;
    }
  }
  if (i >= pinTableSize || config->pulsePin < 0) {
    _state = STEPPER_FAILED;
    return _state;
  }

  // Select timer according to pinTable
  _timerNumber = pinTable[config->pulsePin].timer;
  timerTable[_timerNumber].invertPulse = config->invertPulse;

#ifdef STEPPER_VERBOSE
  Serial.print("Timer init: "); Serial.println(_timerNumber);
  Serial.print("Pin: "); Serial.println(config->pulsePin);
#endif

  // Copy position limits
  positionTable[_timerNumber].minSafePosition = config->minPosition;
  positionTable[_timerNumber].maxSafePosition = config->maxPosition;

  // Configure all further stepper pins
  pinMode(config->dirPin, OUTPUT);
  digitalWrite(config->dirPin, positionTable[_timerNumber].direction ^ config->invertDirection);  // stepper direction and inv config relate bitwise XOR

  // Check if the external driver must be enabled (disabled with -1)
  if (config->enablePin >= 0) {
    pinMode(config->enablePin, OUTPUT);
    digitalWrite(config->enablePin, config->enableActiveLow);            // Disarm stepper driver
  }
#ifdef STEPPER_VERBOSE
    Serial.print("EnablePin: "); Serial.println(config->enablePin);
    Serial.print("InvDir: "); Serial.println(config->invertDirection);
    Serial.print("InvHome: "); Serial.println(config->invertDirection);
    Serial.print("InvPulse: "); Serial.println(config->invertPulse);
#endif

  // Check if driver can issue an alarm state (disabled with -1)
  if (config->alarmPin >= 0) {
    pinMode(config->alarmPin, INPUT);
  }

  // Config home sensor as input
  pinMode(config->homePin, INPUT);

  // Setup the main clock
  _setClockDivider(_GCLKDiv);

  // Disable timer peripherial
  _enableTimer(false);

  // Enable a SAMD21 pin as multiplexed and connect it to a pin using the port multiplexer
  PORT->Group[pinTable[config->pulsePin].port].PINCFG[pinTable[config->pulsePin].samd21Pin].bit.PMUXEN = 1;
  PORT->Group[pinTable[config->pulsePin].port].PMUX[pinTable[config->pulsePin].samd21Pin >> 1].reg |= pinTable[config->pulsePin].pMux;

  _state = STEPPER_NOT_INITIALIZED;
  return _state; 
}

/**************************************************************************/
/*!
  @brief  Calculates the timer values like TCCDIv & CC from the motion 
  periode and minimum pulse width. Prepares the timer peripherial and 
  enables the interrupt. Typically called by the motion controller.
  @param motionPeriode Time sequence of the perdiodic call of doMove() by 
  the motion controller. Time in [ms]
  @returns State STEPPER_NOT_INITIALIZED for failed TCCDiv calculation, 
  otherwise STEPPER_IDLE
*/
/**************************************************************************/
int Stepper::initTimer(float motionPeriode) {
  // Check whether there is sufficient dynamic range
  if (motionPeriode < config->minPulseWidth * 4.0e-3) {
    _state = STEPPER_NOT_INITIALIZED;
    return _state;
  }

  // Calculate TCCDiv and CC0 based on motionPeriode and minPulseWidth
  int TCCDiv = 1;  
  float idealTCCDiv = 1.0;
  idealTCCDiv = (VARIANT_MCK * (motionPeriode * 1.0e-3)) / (_GCLKDiv * timerTable[_timerNumber].counterSize);

#ifdef STEPPER_VERBOSE
    Serial.print("IdealTCCDiv: "); Serial.println(idealTCCDiv);
#endif

  // Derive TCC prescaler from parameter idealTCCDiv; default to 1 if a wrong number was entered
  unsigned int my_TCC_CTRLA_PRESCALER_DIV;
  if (idealTCCDiv < 1.0) {
    TCCDiv = 1;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV1;
  } else if (idealTCCDiv < 2.0) {
    TCCDiv = 2;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV2;
  } else if (idealTCCDiv < 4.0) {
    TCCDiv = 4;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV4;
  } else if (idealTCCDiv < 8.0) {
    TCCDiv = 8;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV8;
  } else if (idealTCCDiv < 16.0) {
    TCCDiv = 16;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV16;
  } else if (idealTCCDiv < 64.0) {
    TCCDiv = 64;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV64;
  } else if (idealTCCDiv < 256.0) {
    TCCDiv = 256;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV256;
  } else if (idealTCCDiv < 1024.0) {
    TCCDiv = 1024;
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV1024;
  } else {
    my_TCC_CTRLA_PRESCALER_DIV = TCC_CTRLA_PRESCALER_DIV1;
    TCCDiv = 1;
    _state = STEPPER_NOT_INITIALIZED;
    return _state;
  }

  // Set prescaler TCCDiv for TCCx
  timerTable[_timerNumber].TCCDiv = TCCDiv;
  *(RwReg*)timerTable[_timerNumber].REG_TCCx_CTRLA |= my_TCC_CTRLA_PRESCALER_DIV;

#ifdef STEPPER_VERBOSE
  Serial.print("TCCDiv: "); Serial.println(TCCDiv);
#endif
  
  // Select nromal PWM and invert output if requested
  if (timerTable[_timerNumber].invertPulse) {
    *(RwReg*)timerTable[_timerNumber].REG_TCCx_WAVE |= TCC_WAVE_WAVEGEN_NPWM | TCC_WAVE_POL(0xF);
  } else {
    *(RwReg*)timerTable[_timerNumber].REG_TCCx_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  }
  while (timerTable[_timerNumber].TCCx->SYNCBUSY.bit.WAVE);
  
  // Enable interrupts
  *(RwReg*)timerTable[_timerNumber].REG_TCCx_INTENSET = TCC_INTENSET_OVF; //Set up interrupt at TOP of each PWM cycle
  NVIC_SetPriority(timerTable[_timerNumber].TCCx_IRQn, 0);     // Set the Nested Vector Interrupt Controller (NVIC) priority
  NVIC_EnableIRQ(timerTable[_timerNumber].TCCx_IRQn);

#ifdef STEPPER_VERBOSE
  Serial.print("TCCx_IRQn: "); Serial.println(_timerNumber);
#endif
  
  // Calculate duty cycle from timer parameters and minPulseWidth
  unsigned int dutyCycle = 2;
  dutyCycle = (int)((VARIANT_MCK * config->minPulseWidth * 1.0e-6) / (_GCLKDiv * timerTable[_timerNumber].TCCDiv));

#ifdef STEPPER_VERBOSE
  Serial.print("TCCx_CCBy: "); Serial.println(dutyCycle);
#endif

  // Clamp resolution to TCCx's counter size and 2
  if (dutyCycle < 2) {
    dutyCycle = 2;
  }
  if (dutyCycle > timerTable[_timerNumber].counterSize) {
    dutyCycle = timerTable[_timerNumber].counterSize;
  } 

  // Set duty cycle
  timerTable[_timerNumber].dutyCycle = dutyCycle;
  *(RwReg*)pinTable[config->pulsePin].REG_TCCx_CCBy = dutyCycle;
  while (timerTable[_timerNumber].TCCx->SYNCBUSY.vec.CCB);

  _state = STEPPER_IDLE;
  return _state;
}

/**************************************************************************/
/*!
  @brief  Updates the PER register to change the frequency of the pulses
  @param steps number of timer ticks 
*/
/**************************************************************************/
void Stepper::_updateTimer(unsigned long long int steps) {

  // Set the periode
  timerTable[_timerNumber].steps = steps;
  *(RwReg*)timerTable[_timerNumber].REG_TCCx_PERB = timerTable[_timerNumber].steps;
  while (timerTable[_timerNumber].TCCx->SYNCBUSY.bit.PERB);

#ifdef STEPPER_VERBOSE
  Serial.print("TCCx_PERB: "); Serial.println(steps);
#endif

  return;
}

/**************************************************************************/
/*!
  @brief  Enables the ready configured timer peripherial
  @param enable Set true to enable and false to disable
*/
/**************************************************************************/
void Stepper::_enableTimer(bool enabled) {
  //Enable Timer
  timerTable[_timerNumber].enabled = enabled;
  if (timerTable[_timerNumber].enabled) {
    *(RwReg*)timerTable[_timerNumber].REG_TCCx_CTRLA |= TCC_CTRLA_ENABLE;
  } else {
    *(RwReg*)timerTable[_timerNumber].REG_TCCx_CTRLA &= ~(TCC_CTRLA_ENABLE);
  }
  while (timerTable[_timerNumber].TCCx->SYNCBUSY.bit.ENABLE);

#ifdef STEPPER_VERBOSE
  Serial.print("TCCx_CTRLA: "); Serial.println(enabled);
#endif
  
  return;
}

/**************************************************************************/
/*!
  @brief  Enables the stepper driver by toggling the enable pin.
  @param enable Set true to enable and false to disable
  @returns State STEPPER_IDLE for disabled drive and STEPPER_RUNNING for
  enabled drive.
*/
/**************************************************************************/
int Stepper::enableDriver(bool enabled) {
  //Enable driver: enabled XOR active low flag
  digitalWrite(config->enablePin, config->enableActiveLow ^ enabled);

#ifdef STEPPER_VERBOSE
  Serial.print("EnablePin: "); Serial.println(config->enableActiveLow ^ enabled);
#endif
  
  // Update state and homed flags
  if (enabled == true) {
    _state = STEPPER_RUNNING;
  } else {
    _state = STEPPER_IDLE;
    positionTable[_timerNumber].homed = false;
  }
  return _state;
}

/**************************************************************************/
/*!
  @brief  Calculates the current speed of the stepper motor
  @returns steps/sec currently generated by the timer
*/
/**************************************************************************/
float Stepper::getSpeed() {  
  return (static_cast<float>(VARIANT_MCK)) / (_GCLKDiv * timerTable[_timerNumber].TCCDiv * timerTable[_timerNumber].steps);
}

/**************************************************************************/
/*!
  @brief  Reads the current position value
  @returns current postion in steps
*/
/**************************************************************************/
int Stepper::getPosition() {
  // Return current position
  return positionTable[_timerNumber].position;
}

/**************************************************************************/
/*!
  @brief  Reads the current status of the stepper driver
  @returns current status
*/
/**************************************************************************/
int Stepper::status() {
  // Return current state
  return _state;
}

/**************************************************************************/
/*!
  @brief  Reads whether the motor had run into a position booundary or not.
  Clears the fault flag and deals with it.
  @returns fault status
*/
/**************************************************************************/
bool Stepper::hasFault() {
  // Check for fault (boundary violation)
  if (positionTable[_timerNumber].fault == false) {
    return false;
  } else {
    // Stop timer, as it can't issue any pulses any more
    _enableTimer(false);

    // clear fault flag, as we now know the fault and can act on it
    positionTable[_timerNumber].fault == false; 

    // We do not know whether steps had been lost, so we must home first 
    positionTable[_timerNumber].homed == false;

    return true;
  }
}

/**************************************************************************/
/*!
  @brief  If the stepper is in state STEPPER_RUNNING it writes the
  configured position of home as current position.
*/
/**************************************************************************/
void Stepper::setHome() {
  // Set position counter to home position
  if (_state == STEPPER_RUNNING) {
    positionTable[_timerNumber].position = config->homePosition;
    positionTable[_timerNumber].homed = true;
  }
  return;
}

/**************************************************************************/
/*!
  @brief  Reads home flag to determine whether the drive is homed and 
  the position value can be trusted.
  @returns true for homed, false for not homed
*/
/**************************************************************************/
bool Stepper::isHomed() {
  return positionTable[_timerNumber].homed;
}

/**************************************************************************/
/*!
  @brief  Updates the timer with a new frequency when in state STEPPER_RUNNING.
  Timer gets disabled for pulses slower as counter size to skip on this cycle.
  Max speed is limited to 2* minimum pulse width or maxVelocity, 
  whichever is smaller. Must be periodically called with motionPeriode.
  @param speed Desired speed in steps/sec. 
  @param direction DIRECTION_CW or CIRECTION_CCW to determine turning direction
  @returns false if stepper is not running, true else.
*/
/**************************************************************************/
bool Stepper::doMove(int speed, Direction direction) {
  static unsigned int steps = 2;

  // Check if stepper is in running state
  if (_state != STEPPER_RUNNING) {
    return false;
  }

  // Set direction
  positionTable[_timerNumber].direction = direction;
  digitalWrite(config->dirPin, direction);  // stepper direction and inv config relate bitwise XOR

  // Clamp max speed to max pulse per second of the driver.
  if (speed > config->maxVelocity) {
    speed = config->maxVelocity;
  }

  // Calculate period for speed
  steps = VARIANT_MCK / (_GCLKDiv * timerTable[_timerNumber].TCCDiv * speed);

  // Clamp resolution to TCCx's counter size and 2 * duty cycle
  // Disable timer if pulserate is too slow
  if (steps < 2 * timerTable[_timerNumber].dutyCycle) {
    steps = 2 * timerTable[_timerNumber].dutyCycle;
  }
  if (steps > timerTable[_timerNumber].counterSize) {
    steps = timerTable[_timerNumber].counterSize;
    _enableTimer(false);
  } else if (timerTable[_timerNumber].enabled == false) {
    _enableTimer(true);
  }

  // Set the periode
  _updateTimer(steps);

  return true;
}

/**************************************************************************/
/*!
  @brief  Sets the main clock divider and connects the 48MHz clock to the
  timer peripherials TCC0 to TC3.
  @param GCLKDiv Divider for main clock fed to peripherials. Typically 1
*/
/**************************************************************************/
void Stepper::_setClockDivider(unsigned int GCLKDiv) {
  // Configure input clock
  // Configure GCLK4 to use DFLL48M
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Clamp GCLKDiv to 1 - 255
  if (GCLKDiv < 1) {
    GCLKDiv = 1;
  }
  if (GCLKDiv > 255) {
    GCLKDiv = 255;
  }
  _GCLKDiv = GCLKDiv;
  
  // Set GCLK4's prescaler
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(_GCLKDiv) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  // Connect GCLK4 to TCC0, TCC1, TCC2, and TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while (GCLK->STATUS.bit.SYNCBUSY);
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC2_TC3;
  while (GCLK->STATUS.bit.SYNCBUSY);
}

//This ISR is called at the end or TOP of each PWM cycle
void TCC0_Handler() {
  // Check for Crash
  if (positionTable[0].homed && !(positionTable[0].minSafePosition < positionTable[0].position <= positionTable[0].maxSafePosition)) {
    // Raise fault flag
    positionTable[0].fault = true;

    // Disable timer and wait for sync
    REG_TCC0_CTRLA &= ~(TCC_CTRLA_ENABLE);
    while (TCC0->SYNCBUSY.bit.ENABLE);

  } else {
    // Increment position 
    if (positionTable[0].direction) {
      positionTable[0].position++;
    } else {
      positionTable[0].position--;
    }
  }

  //Need to reset interrupt
  REG_TCC0_INTFLAG = TC_INTFLAG_OVF; 
}

//This ISR is called at the end or TOP of each PWM cycle
void TCC1_Handler() {
  // Check for Crash
  if (positionTable[1].homed && !(positionTable[1].minSafePosition < positionTable[1].position <= positionTable[1].maxSafePosition)) {
    // Raise fault flag
    positionTable[1].fault = true;

    // Disable timer and wait for sync
    REG_TCC1_CTRLA &= ~(TCC_CTRLA_ENABLE);
    while (TCC1->SYNCBUSY.bit.ENABLE);

  } else {
    // Increment position 
    if (positionTable[1].direction) {
      positionTable[1].position++;
    } else {
      positionTable[1].position--;
    }
  }

  //Need to reset interrupt
  REG_TCC1_INTFLAG = TC_INTFLAG_OVF; 
}

//This ISR is called at the end or TOP of each PWM cycle
void TCC2_Handler() {
  // Check for Crash
  if (positionTable[2].homed && !(positionTable[2].minSafePosition < positionTable[2].position <= positionTable[2].maxSafePosition)) {
    // Raise fault flag
    positionTable[2].fault = true;

    // Disable timer and wait for sync
    REG_TCC2_CTRLA &= ~(TCC_CTRLA_ENABLE);
    while (TCC2->SYNCBUSY.bit.ENABLE);

  } else {
    // Increment position 
    if (positionTable[2].direction) {
      positionTable[2].position++;
    } else {
      positionTable[2].position--;
    }
  }

  //Need to reset interrupt
  REG_TCC2_INTFLAG = TC_INTFLAG_OVF; 
}

