#include "stm32f4xx.h"

//I2C Functions
void I2C_Config(void);
void I2C_Start_Transmitting(uint8_t Adress);
void I2C_Start_Receiving(uint8_t Adress);
void I2C_Stop(void);
uint8_t I2C_Read(I2C_TypeDef* I2Cx);
void I2C_Master_Write(uint8_t Data);

//MPU Functions
void MPU_Wake(uint8_t Adress);
void MPU_Config(uint8_t Adress);
void MPU_ReadAll(uint8_t* temp, uint8_t Adress);
void MPU_GetRawData(uint8_t* temp, float* aX, float* aY, float* aZ, float* tC, float* gX, float* gY, float* gZ);