#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Wire.h>

// I2C Main Bus Pins
#define SDA_PIN 1 
#define SCL_PIN 2
#define TCAADDR 0x70 

// Shared Multiplexer Function
void tcaSelect(uint8_t i);

#endif