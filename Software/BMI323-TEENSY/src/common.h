#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <Wire.h>

#define debug 1
#if debug 
#define D_println(...) Serial.println(__VA_ARGS__)
#define D_print(...) Serial.print(__VA_ARGS__)
#else
#define D_println(...)
#define D_print(...)
#endif




//Helper functions for I2C communication 
uint8_t i2c_write(int addr, int reg, uint16_t data, TwoWire *i2c_port);
uint8_t i2c_read_single(int addr, int reg, uint16_t *data, TwoWire *i2c_port);
uint8_t i2c_read_many(int addr, int reg, uint16_t *data, uint16_t len, TwoWire *i2c_port);
uint16_t mtoi16(uint16_t val);


#endif