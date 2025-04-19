#include "main.h"
#include "i2c.h"
#include "aspeed.h"

uint8_t ASpeedRead(float *data){
    uint8_t data_read[8] = {0x00};
    uint8_t ret_val;
    ret_val = HAL_I2C_Master_Receive(&hi2c1, 0x28 << 1, data_read, 4, 100);
    // uint16_t pressure_cnt = (data_read[0] & 0x3F ) << 8;
    // pressure_cnt = pressure_cnt | data_read[1];
    uint16_t pressure_cnt = ((uint16_t)(data_read[0] & 0x3F) << 8) | data_read[1];
    uint16_t temp_cnt = (((uint16_t)(data_read[2]) ) << 3) | ((data_read[3] & 0xE0) >> 5); 
    float pressure = (((float)pressure_cnt - 0.1 * 16383) * (1.0f + 1.0f)) / (0.8 * 16383) - 1.0f; 
    float temp = (temp_cnt * 200.0f)/ 2047.0f - 50.0f;
    data[0] = pressure;
    data[1] = temp;
    return ret_val;
}
void ASpeedPing( void){
    uint8_t data_read[8] = {0x00};
    HAL_I2C_Master_Receive(&hi2c1, 0x28 << 1, data_read, 4, 100);
}