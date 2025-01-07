/*
 * DIY motor control implementation with acceleration limiting.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <IntervalTimer.h>
#include <Teensy41_Pinout.h>

class PulsePairSteppers { // Class for controlling 2 stepper drivers from the same pulse pin
    private:
    const int stepPin, dirPin1, dirPin2, enablePin1, enablePin2;
    volatile float highPulseUs, lowPulseUs;
    volatile int dir, stepSpeed, targetSpeed, maxSpeed, maxDeltaV, pulseWait;
    volatile bool pulseState;
    IntervalTimer stepTimer;

    static PulsePairSteppers* isrInstance; // Add static instance pointer

    void calculatePulseWait() {pulseWait =  (abs(stepSpeed) * 200) / maxSpeed; }

    static void timerISR() { // pulse hardware timer interrupt service routine
        if (isrInstance) {
            if (isrInstance->pulseState) {
                digitalWriteFast(isrInstance->stepPin, LOW);            // End high pulse
                isrInstance->stepTimer.update(isrInstance->lowPulseUs); // Switch to low duration
            } else {
                digitalWriteFast(isrInstance->stepPin, HIGH); // Start high pulse
                isrInstance->stepTimer.update(isrInstance->highPulseUs);   // Switch to high duration

                if (isrInstance->stepSpeed != isrInstance->targetSpeed) { // Check if moving at target speed
                    if (--isrInstance->pulseWait <= 0) { // Check if waited enough pulses, decrement
                        isrInstance->setVelocity(isrInstance->targetSpeed);
                    }
                }
            }
            isrInstance->pulseState = !isrInstance->pulseState;     // Toggle pulse state
        }
    }

public:
    PulsePairSteppers(int sp, int dp1, int dp2, int ep1, int ep2, int maxSp = 40000, float hP_Us = 3.0f) : 
        stepPin(sp), dirPin1(dp1), dirPin2(dp2),
        enablePin1(ep1), enablePin2(ep2), highPulseUs(hP_Us),
        stepSpeed(0), targetSpeed(0), maxSpeed(maxSp), maxDeltaV(10), pulseWait(0), pulseState(false) {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin1, OUTPUT);
        pinMode(dirPin2, OUTPUT);
        pinMode(enablePin1, OUTPUT);
        pinMode(enablePin2, OUTPUT);
        
        digitalWriteFast(stepPin, LOW);
        digitalWriteFast(dirPin1, LOW);
        digitalWriteFast(dirPin2, HIGH); // TEMPORARY - reversed dir from Motor 1 for testing setup
        digitalWriteFast(enablePin1, HIGH); // Start disabled
        digitalWriteFast(enablePin2, HIGH); // Start disabled
        isrInstance = this;  // Set instance pointer
    }

    void setVelocity(int stepsPerSecond) {
        if (abs(stepsPerSecond) > maxSpeed) { // Clip new speed setpoint within system speed limit
            stepsPerSecond = (stepsPerSecond > 0) ? maxSpeed : -maxSpeed;
        }
        targetSpeed = stepsPerSecond;

        int changeSpeedLimit = abs(stepSpeed) + maxDeltaV; // Clip speed change within system acceleration limit
        if (abs(targetSpeed) > changeSpeedLimit) {
            stepsPerSecond = (stepsPerSecond > 0) ? changeSpeedLimit : -changeSpeedLimit;
        }
        stepSpeed = stepsPerSecond; // new motor velocity setpoint
        dir = getDirection();
        calculatePulseWait(); // update the acceleration waiting period

        noInterrupts(); // prevent interrupts during setpoint and pin level changes
        if(stepsPerSecond != 0) {
            digitalWriteFast(dirPin1, dir);  // Motor 1 direction
            digitalWriteFast(dirPin2, !dir); // Motor 2 direction. TEMPORARY reverse for test setup
            float totalPeriod = 1000000.0f / abs(stepsPerSecond);
            lowPulseUs = totalPeriod - highPulseUs; // highPulseUs defined in construction

            digitalWriteFast(stepPin, HIGH);  // Ensure stepPin starts HIGH
            pulseState = true;                // Set pulseState to match HIGH
            stepTimer.begin(timerISR, highPulseUs); // run high pulse cycle timer
        } else {
            stepTimer.end();
        }
        interrupts();
    }

    void enable() {
        noInterrupts(); // prevent interrupts between driver pin signals
        digitalWriteFast(enablePin1, LOW);
        digitalWriteFast(enablePin2, LOW);
        interrupts();
    }

    void disable() {
        noInterrupts(); // prevent interrupts between driver pin signals
        digitalWriteFast(enablePin1, HIGH);
        digitalWriteFast(enablePin2, HIGH);
        interrupts();
    }

    // Getters and setters
    int getStepSpeed() const { return stepSpeed; }
    bool getDirection() const { return (stepSpeed > 0); } // true for CCW, false for CW
    void setMaxSpeed(int maxSp) { maxSpeed = abs(maxSp); } // Ensure it's non-negative
};

// Add static member initialization outside the class
PulsePairSteppers* PulsePairSteppers::isrInstance = nullptr;

// Define PulsePairSteppers motor control object
volatile float speed = 40000; // TEMPORARY - init to 0 when done with testing. ------------------------------------ # INITIAL SPEED SETTING #
volatile float targetSpeed = -speed;
Threads::Mutex motorMutex;
PulsePairSteppers steppers(pin33, pin34, pin31, pin35, pin32);

void ControlThread() {
    int lastSpeed = 0;    // Cache for the last set speed
    int currentSpeed = 0; // Cache current target speed
    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // TEMPORARY LED blinks for testing

        {
        Threads::Scope lock(motorMutex); // motorMutex prevents race accessing targetSpeed
        currentSpeed = targetSpeed;
        }

        if (currentSpeed != lastSpeed) { // Only update the velocity if it has changed
            steppers.setVelocity(currentSpeed);
            lastSpeed = currentSpeed;
        }

        threads.yield();
    }
}

void SensorThread() {
    while(true) {
        Serial.println("SensorThread");
        threads.delay(499);
    }
}

void CommsThread() {
    while(true) {
        Serial.println("CommsThread");
        
        motorMutex.lock();
        targetSpeed = 0;  // short pause
        motorMutex.unlock();
        threads.delay(200);

        motorMutex.lock();
        targetSpeed = speed; // forward
        motorMutex.unlock();
        threads.delay(4000);

        motorMutex.lock();
        targetSpeed = 0;  // short pause
        motorMutex.unlock();
        threads.delay(200);

        motorMutex.lock();
        targetSpeed = -speed; // reverse
        motorMutex.unlock();
        threads.delay(2000);
    }
}

void setup() {
    Serial.begin(115200);
    
    // Simplify pin setup - only LED needed as class handles other pins
    pinMode(LED_BUILTIN, OUTPUT);
    
    delay(200); // Keep delay for DM860T startup
    
    steppers.enable(); // Enable motors using class method
    
    threads.addThread(ControlThread);
    threads.addThread(SensorThread);
    threads.addThread(CommsThread);
}

void loop() {
    threads.yield();
}