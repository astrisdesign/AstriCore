/*
 * Teensy4.1 motor driver control, load cell reading, and string encoder reading.
 * Encoder prints to serial.
 * USB serial comms development.
 */

#include <Arduino.h>
#include <atomic>
#include <TeensyThreads.h>
#include <HX711.h>
#include "QuadEncoder.h"
#include <Teensy41_Pinout.h>
#include "PulsePairSteppers.h"
#include <ArduinoJson.h>  // Added JSON library

//    #---------- Motor Driver Setup ----------#
volatile float targetSpeed = 0;
Threads::Mutex motorMutex;
PulsePairSteppers steppers(pin33, pin34, pin31, pin35, pin32);

//    #---------- Load Cell Setup -------------#
const int LC1_SCK_PIN = 0;
const int LC1_DAT_PIN = 1;
HX711 loadCell1;
volatile int32_t loadReading1 = 0;
Threads::Mutex loadMutex;

//    #---------- String Encoder Setup --------#
QuadEncoder stringEnc(1, 2, 3, 0); // Channel 1, A=2, B=3, no pullups
volatile int strEncPos = 0; // string encoder position
Threads::Mutex strEncMutex;

//    #----------- Comms Vars -----------------#
const char* msg = ""; // USB serial message
Threads::Mutex commsMutex;

void ControlThread() {
    int lastSpeed = 0;    // Cache for the last set speed
    int currentSpeed = 0; // Cache current target speed
    while(true) {
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
        {
            Threads::Scope lock(loadMutex);
            loadReading1 = loadCell1.read(); // read load cell
        }
        {
            Threads::Scope lock(strEncMutex);
            strEncPos = stringEnc.read(); // read encoder
        }
        threads.delay(100);
    }
}

void CommsThread() { // USB serial comm thread.
    JsonDocument doc;
    int lc1_val, se1_val, mstep_val, listenCount = 0;
    int vel;

    while(true) {
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN)); // TEMPORARY LED blinks for testing
        if (Serial.available() > 0) { // Handle incoming serial data
            String incoming = Serial.readStringUntil('\n');
            JsonDocument cmdDoc;
            DeserializationError error = deserializeJson(cmdDoc, incoming);
            if (!error) {
                if (cmdDoc["setpoints"].is<JsonObject>()) {
                    JsonObject setPoints = cmdDoc["setpoints"];
                    if (setPoints["vel"].is<int>()) {
                        Threads::Scope lock(motorMutex);
                        targetSpeed = setPoints["vel"];
                    }
                }
                if (cmdDoc["msg"].is<const char*>()) {
                        Threads::Scope lock(commsMutex);
                        msg = cmdDoc["msg"];
                    } else {
                        Threads::Scope lock(commsMutex);
                        msg = ""; // If there's no message, msg is cleared.
                    }
            } else {
                Threads::Scope lock(commsMutex);
                msg = "Last input caused a serial decode error";
            }
        }
        listenCount++; // Listens for serial messages more often than sending them.

        if (listenCount >= 9) { // Send out a JSON message
            // Get sensor data snapshots with proper locking
            {
                Threads::Scope lock(loadMutex);
                lc1_val = loadReading1;
            }
            {
                Threads::Scope lock(strEncMutex);
                se1_val = strEncPos;
            }
            {
                Threads::Scope lock(motorMutex);
                vel = targetSpeed;
            }

            // Build JSON structure
            doc.clear();
            JsonObject data = doc["data"].to<JsonObject>();
            data["lc1"] = lc1_val;
            data["se1"] = se1_val;
            data["mstep"] = mstep_val;

            JsonObject setpoints = doc["setpoints"].to<JsonObject>();
            setpoints["vel"] = vel;

            {
                Threads::Scope lock(commsMutex);
                doc["msg"] = msg; // update output text
            }

            // Serialize and transmit
            serializeJson(doc, Serial);
            Serial.println();
            listenCount = 0; // Reset counter
        }

        threads.delay(5); // very tight loop for responsiveness
    }
}

void setup() {
    Serial.begin(115200);
    loadCell1.begin(LC1_DAT_PIN, LC1_SCK_PIN); // init load cell object
    pinMode(LED_BUILTIN, OUTPUT); // Enable Builtin LED flash
    
    delay(210); // Short delay for various things

    steppers.enable(); // DM860T pins low (enable motors)

    stringEnc.setInitConfig(); // Initialize hardware encoder
    stringEnc.init();

    threads.addThread(ControlThread);
    threads.addThread(SensorThread);
    threads.addThread(CommsThread);
}

void loop() {
    threads.yield();
}