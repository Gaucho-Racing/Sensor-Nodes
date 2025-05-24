#ifndef __BMI323_H__
#define __BMI323_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

/* BMI323 Register Map */
#define BMI323_REG_CHIP_ID       0x00


/* BMI323 Chip ID Value */
#define BMI323_CHIP_ID           0x43

/* Function prototypes */
HAL_StatusTypeDef BMI323_Init(void);
HAL_StatusTypeDef BMI323_ReadReg(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMI323_WriteReg(uint8_t reg, uint8_t data);


#ifdef __cplusplus
}
#endif

#endif /* __BMI323_H__ */ 