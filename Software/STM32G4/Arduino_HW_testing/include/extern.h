#include "Wire.h"

void MLX90640_I2CInit(void){
    return;
}
extern int MLX90640_I2CGeneralReset(void){
    Wire.beginTransmission(0x00);
    Wire.write(0x06);
    Wire.endTransmission();
}
extern int MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data){
    Wire.beginTransmission(slaveAddr);
    Wire.write(startAddress >> 8);
    Wire.write(startAddress & 0x00FF);
    Wire.endTransmission();
    Wire.requestFrom(slaveAddr, nMemAddressRead*2);
    int i = 0;
    while (Wire.available()){
        data[i] = Wire.read()<<8 | Wire.read();
        i++;
    }
    return 0;
}
extern int MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data){
    Wire.beginTransmission(slaveAddr);
    Wire.write(writeAddress >> 8);
    Wire.write(writeAddress & 0x00FF);
    Wire.write(data >> 8);
    Wire.write(data & 0x00FF);
    Wire.endTransmission();
    return 0;
}
extern void MLX90640_I2CFreqSet(int freq){
    Wire.setClock(freq);
    return;
}