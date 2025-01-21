#ifndef PULSE_PAIR_STEPPERS_H
#define PULSE_PAIR_STEPPERS_H

#include <Arduino.h>
#include <IntervalTimer.h>

class PulsePairSteppers {
private:
    const int pulsePin, dirPin1, dirPin2, enablePin1, enablePin2;
    volatile float highPulseUs, lowPulseUs;
    volatile int dir, pulseSpeed, targetSpeed, maxSpeed, maxDeltaV, pulseWait;
    volatile bool pulseState;
    IntervalTimer pulseTimer;
    static PulsePairSteppers* isrInstance; // Static instance pointer for ISR access to member vars

    void calculatePulseWait(); // pulseWait prevents runaway acceleration and motor lockout.
    static void timerISR(); // pulse hardware timer interrupt service routine

public:
    PulsePairSteppers(int sp, int dp1, int dp2, int ep1, int ep2, int maxSp = 35000, float hP_Us = 3.0f);
    void setVelocity(int pulsesPerSecond);
    void enable();
    void disable();
    int getpulseSpeed() const; // speed in pulses/s
    bool getDirection() const; // true for CCW, false for CW
    void setMaxSpeed(int maxSp);
};

#endif