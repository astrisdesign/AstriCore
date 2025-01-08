#ifndef PULSE_PAIR_STEPPERS_H
#define PULSE_PAIR_STEPPERS_H

#include <Arduino.h>
#include <IntervalTimer.h>

class PulsePairSteppers {
private:
    const int stepPin, dirPin1, dirPin2, enablePin1, enablePin2;
    volatile float highPulseUs, lowPulseUs;
    volatile int dir, stepSpeed, targetSpeed, maxSpeed, maxDeltaV, pulseWait;
    volatile bool pulseState;
    IntervalTimer stepTimer;
    static PulsePairSteppers* isrInstance;
    
    void calculatePulseWait();
    static void timerISR();
    bool getDirection() const;

public:
    PulsePairSteppers(int sp, int dp1, int dp2, int ep1, int ep2, 
                      int maxSp = 35000, float hP_Us = 3.0f);
    void setVelocity(int stepsPerSecond);
    void enable();
    void disable();
    int getStepSpeed() const;
    void setMaxSpeed(int maxSp);
};

#endif