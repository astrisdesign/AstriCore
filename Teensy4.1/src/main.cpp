/*
 * AccelStepper motor control implementation. The motor setpoint is chaotic because the
 * target position changes in an asynchronous thread. Not sure what to think of that...
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Teensy41_Pinout.h>
#include <AccelStepper.h>

// Motor 1 pin definition (AccelStepper inputs)
const int pulse1 = pin33;
const int dir1 = pin34;
const int enable1 = pin35;

// Motor 2 pin definition (pulse hardwired to Motor 1)
const int dir2 = pin31;
const int enable2 = pin32;

// Motor control and mutex variables
AccelStepper stepper1(AccelStepper::DRIVER, pulse1, dir1);
Threads::Mutex motorMutex;
volatile long target_position1 = 45000;

void ControlTask() {
    // Initialize stepper parameters
    stepper1.setMaxSpeed(40000);
    stepper1.setAcceleration(300000);
    stepper1.setMinPulseWidth(3);
    bool prevDir = stepper1.speed() > 0;

    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
        
        motorMutex.lock();
        if(stepper1.targetPosition() != target_position1) {
            prevDir = stepper1.speed() > 0;
            stepper1.moveTo(target_position1);
            if(prevDir != (stepper1.speed() > 0)) {
                digitalWrite(dir2, prevDir); // #---- NOTE ----# This is opposite direction for Motor 2. Change to !prevDir in final setup
            }
        }
    
        stepper1.run();
        motorMutex.unlock();

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
        
        // Add position control
        motorMutex.lock();
        target_position1 = -target_position1;
        motorMutex.unlock();
        
        threads.delay(3000);
    }
}

void setup() {
    Serial.begin(115200);

    // Pin definitions
    pinMode(LED_BUILTIN, OUTPUT);  // LED pin is output
    pinMode(pulse1, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(enable1, OUTPUT);
    pinMode(enable2, OUTPUT);

    // Initial levels
    digitalWrite(pulse1, LOW); // Start pulse low
    digitalWrite(dir1, LOW); // Start with known dir
    digitalWrite(dir2, HIGH); // Start with known dir
    digitalWrite(enable1, LOW); // enable DM860T 1
    digitalWrite(enable2, LOW); // enable DM860T 2

    delay(200); // Wait for DM860T to enable

    threads.addThread(ControlTask);
    threads.addThread(SensorTask);
    threads.addThread(CommsTask);
}

void loop() {
    threads.yield();
}