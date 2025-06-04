#include "PulsePairSteppers.h"

PulsePairSteppers* PulsePairSteppers::isrInstance = nullptr;

void PulsePairSteppers::calculatePulseWait() { // pulseWait prevents runaway acceleration and motor lockout.
    bool accelerating = (abs(targetSpeed) > abs(pulseSpeed)) && ((targetSpeed * pulseSpeed) > 0);
    pulseWait = accelerating
    ? ((abs(pulseSpeed) - 1500) * 1000) / maxSpeed   // acceleration profile
    : ((abs(pulseSpeed) - 50) * 30) / maxSpeed;  // deceleration profile
}

void PulsePairSteppers::timerISR() { // pulse hardware timer interrupt service routine
    if (isrInstance) {
        if (isrInstance->pulseState) {
            digitalWriteFast(isrInstance->pulsePin, LOW);            // End high pulse
            isrInstance->pulseTimer.update(isrInstance->lowPulseUs); // Switch to low duration
        } else {
            digitalWriteFast(isrInstance->pulsePin, HIGH); // Start high pulse
            isrInstance->pulseTimer.update(isrInstance->highPulseUs);   // Switch to high duration

            if (isrInstance->pulseSpeed != isrInstance->targetSpeed) { // Check if moving at target speed
                if (--isrInstance->pulseWait <= 0) { // Check if waited enough pulses, decrement
                    isrInstance->setVelocity(isrInstance->targetSpeed);
                }
            }
        }
        isrInstance->pulseState = !isrInstance->pulseState;     // Toggle pulse state
    }
}

PulsePairSteppers::PulsePairSteppers(int sp, int dp1, int dp2, int ep1, int ep2, int maxSp, float hP_Us) :
    pulsePin(sp), dirPin1(dp1), dirPin2(dp2),
    enablePin1(ep1), enablePin2(ep2), highPulseUs(hP_Us),
    pulseSpeed(0), targetSpeed(0), maxSpeed(maxSp), maxDeltaV(800), pulseWait(0), pulseState(false) {
    pinMode(pulsePin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);

    digitalWriteFast(pulsePin, LOW);
    digitalWriteFast(dirPin1, HIGH); // Set initial motor direction to CCW
    digitalWriteFast(dirPin2, HIGH);
    digitalWriteFast(enablePin1, HIGH); // Start disabled
    digitalWriteFast(enablePin2, HIGH); // Start disabled
    isrInstance = this;  // Set instance pointer
}

void PulsePairSteppers::setVelocity(int pulsesPerSecond) {
    if (abs(pulsesPerSecond) > maxSpeed) { // Clip new speed setpoint within system speed limit
        pulsesPerSecond = (pulsesPerSecond > 0) ? maxSpeed : -maxSpeed;
    }
    targetSpeed = pulsesPerSecond;

    int deltaV = abs(targetSpeed - pulseSpeed); // Clip speed change within system acceleration limit
    if (deltaV > maxDeltaV) {
        pulsesPerSecond = (targetSpeed > pulseSpeed) ? (pulseSpeed + maxDeltaV) : (pulseSpeed - maxDeltaV);
    }
    pulseSpeed = pulsesPerSecond; // new motor velocity setpoint
    dir = getDirection();
    calculatePulseWait(); // update the acceleration waiting period

    noInterrupts(); // prevent interrupts during setpoint and pin level changes
    if(pulsesPerSecond != 0) {
        digitalWriteFast(dirPin1, dir);  // Motor 1 direction
        digitalWriteFast(dirPin2, dir); // Motor 2 direction
        float totalPeriod = 1000000.0f / abs(pulsesPerSecond);
        lowPulseUs = totalPeriod - highPulseUs; // highPulseUs defined in construction

        digitalWriteFast(pulsePin, HIGH);  // Ensure pulsePin starts HIGH
        pulseState = true;                // Set pulseState to match HIGH
        pulseTimer.begin(timerISR, highPulseUs); // run high pulse cycle timer
    } else {
        pulseTimer.end();
    }
    interrupts();
}

void PulsePairSteppers::enable() {
    noInterrupts(); // prevent interrupts between driver pin signals
    digitalWriteFast(enablePin1, LOW);
    digitalWriteFast(enablePin2, LOW);
    interrupts();
}

void PulsePairSteppers::disable() {
    noInterrupts(); // prevent interrupts between driver pin signals
    digitalWriteFast(enablePin1, HIGH);
    digitalWriteFast(enablePin2, HIGH);
    interrupts();
}

// Getters and setters
int PulsePairSteppers::getpulseSpeed() const { return pulseSpeed; }
bool PulsePairSteppers::getDirection() const { return (pulseSpeed > 0); } // true for CCW, false for CW
void PulsePairSteppers::setMaxSpeed(int maxSp) { maxSpeed = abs(maxSp); } // Ensure it's non-negative