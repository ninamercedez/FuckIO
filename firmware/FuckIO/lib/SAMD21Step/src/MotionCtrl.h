#ifndef MotionCtrl_h
#define MotionCtrl_h

#include <Arduino.h>
#include <SAMD21Step.h>

typedef enum {
    STOP = 0,
    ACCELERATING = 1,
    ON_SPEED = 2,
    DECELERATING = 3
} MotionPhase;

typedef struct {
    Stepper* stepper;                   // Reference to the stepper motor
    MotionPhase trapezPhase;            // In with phase of the motion are we
    Direction direction;                // Direction the motor must turn to reach the target position
    long long int targetPosition;       // The target position of the motion
    int currentSpeed;                   // Current speed in [steps / motion periode]
    int maxSpeed;                       // maximum speed of the motion in [steps / motion periode]
    int accelDelta;                     // acceleration delta in [steps / motion periode ^2]
    int decelDelta;                     // deceleration delta in [steps / motion periode ^2]
    int SpeedAtTarget;                  // Speed when reaching the target in [steps / motion periode]
    int motionPeriodeCount;             // Counting the motion periodes that have already elapsed for this motion
    int brakingPoint;                   // number of motion periodes to elapse until deceleration starts
    long long int lastPosition;         // position after the end of the last motion periode
    long long int estimatedPosition;    // expected position at end of this motion periode 
    bool nextOnTarget;                  // True if target is reached after this motion periode.
} MotionParameter;

static MotionParameter motionTable[] {
    {NULL, STOP, DIRECTION_CW, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {NULL, STOP, DIRECTION_CW, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {NULL, STOP, DIRECTION_CW, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

class MotionCtrl {
  public:
    static MotionCtrl& instance() {                      // Define this class as a singleton
        static MotionCtrl _instance;
        return _instance;
    }
    ~MotionCtrl() {}
    int begin(float motionPeriode, Stepper* stepper);
    int begin(float motionPeriode, Stepper* stepper[]);
    int home(Stepper* stepper, int speed);
    int stop(Stepper* stepper);
    void Estop();
    int moveTo(Stepper* stepper, long long int position, int speed, int acceleration, int deceleration);
    int moveTo(Stepper* stepper, long long int position, int speed, int acceleration, int deceleration, void (*callback_func)(void));
    int moveInc(Stepper* stepper, long long int position, int time, bool stopMoveAtEnd);
    int moveInc(Stepper* stepper, long long int position, int time, bool stopMoveAtEnd, void (*callback_func)(void));
    static void handle_isr();
  private:
    MotionCtrl() {}                                     // Do not allow initiation outside of MotionCtrl
    MotionCtrl(const MotionCtrl&);                      // Do not allow copies
    MotionCtrl & operator = (const MotionCtrl &);       // Do not allow copies, part deux
    void _updateMotion();
    void _initTimers();
    int _stepperCount = 0;                              
    float _motionPeriode = 5.0;                         // recalculation periode for updating the speed of the stepper   
    TcCount16* _TC = (TcCount16*) TC3;                  // Pointer to TC3 
    void (*callback_atTargetPosition)(void);            // Function to be called when the target position is reached

};


#endif