/*
 * AccelStepper motor control implementation.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <IntervalTimer.h>
#include <Teensy41_Pinout.h>

// Add the PulsePairSteppers class definition here
class PulsePairSteppers {
    private:
    const int stepPin, dirPin1, dirPin2, enablePin1, enablePin2;
    volatile int stepSpeed;
    volatile bool stepReady;
    IntervalTimer stepTimer;
    Threads::Mutex velocityMutex;
    
    // Add static instance pointer
    static PulsePairSteppers* isrInstance;
    
    // Modify ISR to be static without arguments
    static void timerISR() {
        if (isrInstance) {
            isrInstance->stepReady = true;
        }
    }

public:
    PulsePairSteppers(int sp, int dp1, int dp2, int ep1, int ep2) : 
        stepPin(sp), dirPin1(dp1), dirPin2(dp2),
        enablePin1(ep1), enablePin2(ep2),
        stepSpeed(0), stepReady(false) {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin1, OUTPUT);
        pinMode(dirPin2, OUTPUT);
        pinMode(enablePin1, OUTPUT);
        pinMode(enablePin2, OUTPUT);
        
        digitalWriteFast(stepPin, LOW);
        digitalWriteFast(dirPin1, LOW);
        digitalWriteFast(dirPin2, HIGH);
        digitalWriteFast(enablePin1, HIGH);  // Start disabled
        digitalWriteFast(enablePin2, HIGH);
        isrInstance = this;  // Set instance pointer
    }

    void setVelocity(int stepsPerSecond) {
        Threads::Scope lock(velocityMutex);
        if (abs(stepsPerSecond) > 40000) {
            stepsPerSecond = (stepsPerSecond > 0) ? 40000 : -40000;
        }
        stepSpeed = stepsPerSecond;
        
        if(stepsPerSecond != 0) {
            digitalWriteFast(dirPin1, stepsPerSecond > 0);
            digitalWriteFast(dirPin2, stepsPerSecond < 0);
            float period = 1000000.0f / abs(stepsPerSecond);
            stepTimer.begin(timerISR, period);  // Modified timer setup
        } else {
            stepTimer.end();
        }
    }

    bool checkAndStep() {
        if(stepReady) {
            digitalWriteFast(stepPin, HIGH);
            delayMicroseconds(3);
            digitalWriteFast(stepPin, LOW);
            stepReady = false;
            return true;
        }
        return false;
    }

    void enable() {
        digitalWriteFast(enablePin1, LOW);
        digitalWriteFast(enablePin2, LOW);
    }

    void disable() {
        digitalWriteFast(enablePin1, HIGH);
        digitalWriteFast(enablePin2, HIGH);
    }
};

// Add static member initialization outside the class
PulsePairSteppers* PulsePairSteppers::isrInstance = nullptr;

// Replace AccelStepper instance with PulsePairSteppers
volatile float speed = 10000;
volatile float targetSpeed = -speed;
Threads::Mutex motorMutex;
PulsePairSteppers steppers(pin33, pin34, pin31, pin35, pin32);

void ControlTask() {
    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        
        motorMutex.lock();
        int currentSpeed = targetSpeed;
        motorMutex.unlock();
        
        steppers.setVelocity(currentSpeed);
        steppers.checkAndStep();
        
        threads.yield();
    }
}

void SensorTask() {
    while(true) {
        Serial.println("SensorTask");
        threads.delay(499);
    }
}

void CommsTask() {
    while(true) {
        Serial.println("CommsTask");
        
        motorMutex.lock();
        targetSpeed = 0;  // short pause
        motorMutex.unlock();
        threads.delay(200);

        motorMutex.lock();
        targetSpeed = speed; // forward
        motorMutex.unlock();
        threads.delay(2000);

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
    
    threads.addThread(ControlTask);
    threads.addThread(SensorTask);
    threads.addThread(CommsTask);
}

void loop() {
    threads.yield();
}