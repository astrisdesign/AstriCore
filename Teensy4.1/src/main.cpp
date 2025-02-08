/*
 * Teensy4.1 motor driver control, load cell reading, and string encoder reading.
 * Encoder prints to serial.
 * Load cell sets motor speed.
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
const int strEncZPin = 4; // Z=4
volatile int strEncPos = 0; // string encoder position
std::atomic<bool> strEncZFlag = false; // Encoder Z flag. NOT USED
int32_t strEncLastPos = 0;
void zPinInterrupt() { // Index (Z) interrupt handler. NOT USED
    strEncZFlag = true;
}
Threads::Mutex strEncMutex;

//    #----------- Comms Vars -----------------#
const char* msg = "teyest"; // USB serial message
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
    loadCell1.set_scale(1.0f);
    loadCell1.tare();

    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // TEMPORARY LED blinks for testing
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

void CommsThread() { // USB serial comm thread. COMPLETELY REWORKED
    // Reserve enough capacity for our JSON structure. Adjust as needed.
    JsonDocument doc;

    while(true) {
        int lc1_val, se1_val, mstep_val = 0;
        int vel;

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

        // Handle incoming commands
        if (Serial.available() > 0) {
            String incoming = Serial.readStringUntil('\n');
            JsonDocument cmdDoc;
            DeserializationError error = deserializeJson(cmdDoc, incoming);
            if (!error && cmdDoc["setpoints"].is<JsonObject>() && cmdDoc["msg"].is<const char*>()) {
                JsonObject setPoints = cmdDoc["setpoints"];
                msg = cmdDoc["msg"];
                if (setPoints["vel"].is<int>()) {
                    Threads::Scope lock(motorMutex);
                    targetSpeed = setPoints["vel"];
                }
            }
        }
        threads.delay(10);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200); // Short delay for DM860T startup and serial init
    
    pinMode(LED_BUILTIN, OUTPUT); // Enable Builtin LED flash

    loadCell1.begin(LC1_DAT_PIN, LC1_SCK_PIN); // load cell object  

    steppers.enable(); // DM860T pins low (enable motors)

    stringEnc.setInitConfig(); // Initialize hardware encoder
    stringEnc.init();
    pinMode(strEncZPin, INPUT_PULLUP); // specifies Z pin for ABZ quad. NOT USED
    // attachInterrupt(digitalPinToInterrupt(strEncZPin), zPinInterrupt, FALLING); Interrupts on encoder Z pulse. NOT USED

    threads.addThread(ControlThread);
    threads.addThread(SensorThread);
    threads.addThread(CommsThread);
}

void loop() {
    threads.yield();
}