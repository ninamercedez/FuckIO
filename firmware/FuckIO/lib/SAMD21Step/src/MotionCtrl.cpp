#include <MotionCtrl.h>

// Reference to the MotrionCtrl instance for access inside TC3 ISR
//MotionCtrl* __motionObject = NULL;

/**************************************************************************/
/*!
  @brief  Introduces one stepper object to the motion control instance.
  Setup of TC3 to generate the ticks to do motion calculations.
  @param motionPeriode Timer periode in [ms] after which the movment calculation 
  is upadted. Typically set to 5.0 ms. Longer times may result in choppy 
  movement. Shorter times increase the processor laoding.
  @param stepper Pointer to an array of stepper object hows motion 
  needs to be controlled. Maximum of 3 elements.
  @return 0 if failed, 1 on sccuess
*/
/**************************************************************************/
int MotionCtrl::begin(float motionPeriode, Stepper* stepper[]) {

  //Copy pointer to config
  _stepperCount = sizeof(stepper)/sizeof(stepper[0]);
  if (_stepperCount > 3) {
    _stepperCount = 0;
    return 0;
  }

  for (int i = 0; i < _stepperCount; i++) {
    motionTable[i].stepper = stepper[i];
  }

  // Store motion periode
  _motionPeriode = motionPeriode;

  // Initialize TC3
  _initTimers();

  return 1;
}

/**************************************************************************/
/*!
  @brief  Introduces one stepper object to the motion control instance.
  Setup of TC3 to generate the ticks to do motion calculations.
  @param motionPeriode Timer periode in [ms] after which the movment calculation 
  is upadted. Typically set to 5.0 ms. Longer times may result in choppy 
  movement. Shorter times increase the processor laoding.
  @param stepper Pointer to the stepper object whos motion needs to be controlled 
  @return 0 if failed, 1 on sccuess
*/
/**************************************************************************/
int MotionCtrl::begin(float motionPeriode, Stepper* stepper) {
  
  //Copy pointer to config
  _stepperCount = 1;
  motionTable[0].stepper = stepper;

  // Store motion period
  _motionPeriode = motionPeriode;

  // Initialize TC3
  _initTimers();

  return 1;
}

/**************************************************************************/
/*!
  @brief  Initializes all timers. First the stepper timers are initialized
  as they provide the clock also for TC3. Then TC3 is setup to call the 
  motion calculation periodically. 
*/
/**************************************************************************/
void MotionCtrl::_initTimers() {
  // First initialize all TCC timers of steppers
  for (int i = 0; i < _stepperCount; i++) {
    motionTable[i].stepper->initTimer(_motionPeriode);
  }

  // Disable TC3
  _TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (_TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  _TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (_TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  _TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (_TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  _TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (_TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set timer frequency
  int compareValue = (VARIANT_MCK / (1024 * 5)) - 1;   // trigger at 5 Hz for test
  _TC->COUNT.reg = map(_TC->COUNT.reg, 0, _TC->CC[0].reg, 0, compareValue); // Sets counter value --> might be usefull for step counters??
  _TC->CC[0].reg = compareValue;
  while (_TC->STATUS.bit.SYNCBUSY == 1);

  // Enable the compare interrupt
  _TC->INTENSET.reg = 0;
  _TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable Timer
  _TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (_TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

}        

void MotionCtrl::_updateMotion(){
  Serial.println("Member function called");
  return;
}

void MotionCtrl::handle_isr(){
    MotionCtrl::instance()._updateMotion();// get the singleton instance and call the method.
}

//This ISR is called at the end or TOP of each PWM cycle
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // Check if this interrupt is due to the compare register matching the timer count
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Update motion calculation
    MotionCtrl::handle_isr();
  }

}



