#include <stdint.h>

void MPU_Write(uint8_t Address, uint8_t Register, uint8_t Data);

void MPU_Read(uint8_t Address, uint8_t Register, uint8_t *buffer, uint8_t size);

void MPU6050_Init(void);

float MPU6050_Read_Accel(void);
