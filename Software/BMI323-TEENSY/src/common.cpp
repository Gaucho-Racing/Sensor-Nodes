#include "common.h"

uint8_t i2c_write(int addr, int reg, uint16_t data, TwoWire *i2c_port){
    //Serial.println(data, HEX);
    i2c_port->beginTransmission(addr);
    i2c_port->write(reg);
    i2c_port->write(data & 0x00FF);
    i2c_port->write(data >> 8);
    i2c_port->endTransmission();
    return 0;
}

uint8_t i2c_read_single(int addr, int reg, uint16_t *data, TwoWire *i2c_port){
    i2c_port->beginTransmission(addr);
    i2c_port->write(reg);
    i2c_port->endTransmission();
    i2c_port->requestFrom(addr, 4);
    //the two reads are required as they are dummy bytes, hence we read them to skip
    if(i2c_port->available() == 4){
        i2c_port->read();
        i2c_port->read();
        *data = i2c_port->read();
        *data |= i2c_port->read() << 8;
        return 1;
    }
    return 0;
}

uint8_t i2c_read_many(int addr, int reg, uint16_t *data, uint16_t len, TwoWire *i2c_port){
    i2c_port->beginTransmission(addr);
    i2c_port->write(reg);
    i2c_port->endTransmission();
    i2c_port->requestFrom(addr, len*2 + 2);
    if(i2c_port->available() == len*2 + 2){
        i2c_port->read();
        i2c_port->read();
        for(int i = 0; i < len; i++){
            data[i] = i2c_port->read() << 8;
            data[i] |= i2c_port->read();
        }
        return 1;
    }
    return 0;
}


//flips the two 8 bit bytes to form a 16 bit integer
uint16_t mtoi16(uint16_t val){
    uint8_t low = val & 0xFF;
    uint8_t high = (val >> 8) & 0xFF;
    return (low << 8) | high;
}