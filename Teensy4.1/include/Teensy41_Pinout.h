#ifndef TEENSY41_PINOUT_H
#define TEENSY41_PINOUT_H

// #################    TEENSY 4.1 PINOUT    ##################################################################
// Digital pins with special functions, organized by location

// USB Left
const int pin0 = 0;   // PWM, CRX2, CS1, RX1
const int pin1 = 1;   // PWM, CTX2, MISO1, TX1
const int pin2 = 2;   // PWM, OUT2
const int pin3 = 3;   // PWM, LRCLK2
const int pin4 = 4;   // PWM, BCLK2
const int pin5 = 5;   // PWM, IN2
const int pin6 = 6;   // PWM, OUT1D
const int pin7 = 7;   // PWM, RX2, OUT1A
const int pin8 = 8;   // PWM, TX2, IN1
const int pin9 = 9;   // PWM, OUT1C
const int pin10 = 10; // PWM, CS, MQSR
const int pin11 = 11; // PWM, MOSI, CTX1
const int pin12 = 12; // PWM, MISO, MQSL

// USB Right
const int pin13 = 13; // PWM, SCK, #--------------------#*LED*#----#
const int pin14 = 14; // PWM, A0, TX3, SPDIF_OUT
const int pin15 = 15; // PWM, A1, RX3, SPDIF_IN
const int pin16 = 16; // A2, RX4, SCL1
const int pin17 = 17; // A3, TX4, SDA1
const int pin18 = 18; // PWM, A4, SDA
const int pin19 = 19; // PWM, A5, SCL
const int pin20 = 20; // A6, TX5, LRCLK1
const int pin21 = 21; // A7, RX5, BCLK1
const int pin22 = 22; // PWM, A8, CTX1
const int pin23 = 23; // PWM, A9, CRX1, MCLK1

// SD Left
const int pin24 = 24; // PWM, A10, TX6, SCL2
const int pin25 = 25; // PWM, A11, RX6, SDA2
const int pin26 = 26; // A12, MOSI1
const int pin27 = 27; // A13, SCK1
const int pin28 = 28; // PWM, RX7
const int pin29 = 29; // PWM, TX7
const int pin30 = 30; // CRX3
const int pin31 = 31; // CTX3
const int pin32 = 32; // OUT1B

// SD Right
const int pin33 = 33; // PWM, MCLK2
const int pin34 = 34; // RX8
const int pin35 = 35; // TX8
const int pin36 = 36; // PWM, CS
const int pin37 = 37; // PWM, CS
const int pin38 = 38; // A14, CS1, IN1
const int pin39 = 39; // A15, MISO1, OUT1A
const int pin40 = 40; // A16
const int pin41 = 41; // A17

// Important notes:
// - All pins operate at 3.3V - NOT 5V tolerant
// - PWM available on 35 pins
// - 18 Analog inputs (A0-A17)
// - Pin 13 has built-in orange LED
// - Default Serial: Serial1 (pins 0,1)
// - Default I2C: Wire (SCL0=19, SDA0=18)
// - Default SPI: SPI (MOSI=11, MISO=12, SCK=13)

#endif // TEENSY41_PINOUT_H