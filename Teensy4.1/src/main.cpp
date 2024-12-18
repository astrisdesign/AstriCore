/*
 * Attempting AccelStepper
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Teensy41_Pinout.h>
#include <AccelStepper.h>

// Motor 1 pin definition
const int pulse1 = pin33;
const int dir1 = pin34;
const int enable1 = pin35;

// Motor control and mutex variables
AccelStepper stepper1(AccelStepper::DRIVER, pulse1, dir1);
Threads::Mutex motorMutex;
volatile long target_position = 1000;

void ControlTask() {
    // Initialize stepper parameters
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(500);
    stepper1.moveTo(target_position);
    
    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
        
        motorMutex.lock();
        if(stepper1.targetPosition() != target_position) {
            stepper1.moveTo(target_position);
        }
        motorMutex.unlock();
        
        stepper1.run();
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
        target_position = -target_position;
        motorMutex.unlock();
        
        threads.delay(997);
    }
}

void setup() {
    Serial.begin(115200);

    // Pin definitions
    pinMode(LED_BUILTIN, OUTPUT);  // LED pin is output
    pinMode(pulse1, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(enable1, OUTPUT);

    digitalWrite(pulse1, LOW); // Start pulse low
    digitalWrite(dir1, LOW); // Start with known dir
    digitalWrite(enable1, LOW); // enable DM860T 

    delay(200); // Wait for DM860T to enable

    threads.addThread(ControlTask);
    threads.addThread(SensorTask);
    threads.addThread(CommsTask);
}

void loop() {
    threads.yield();
}