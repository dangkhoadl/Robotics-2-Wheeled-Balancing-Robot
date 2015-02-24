#include "MPU6050_tm.h"

//I2C Functions
//Thiet lap I2C
void I2C_Config(void)
{
	/* 
	I2C1 	SCL pin PB6
				SDA pin PB9
	Speed	100 KHz
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//Config I2C Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA
	//End
	
	// Config I2C1 
	I2C_InitStructure.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStructure);	
	//End
	
	//I2C Interupt Config
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//End
	
	I2C_Cmd(I2C1, ENABLE);
}

/* I2C bat dau SEND toi MPU*/
void I2C_Start_Transmitting(uint8_t Adress)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	I2C_Send7bitAddress(I2C1, (Adress), I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
}

/* I2C bat dau RECEIVE tu MPU*/
void I2C_Start_Receiving(uint8_t Adress)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	I2C_Send7bitAddress(I2C1, (Adress), I2C_Direction_Receiver);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);
}

/* Gui bit STOP*/
void I2C_Stop(void)
{
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != ERROR);
}

/* Master Send*/
void I2C_Master_Write(uint8_t Data)
{
	I2C_SendData(I2C1, Data);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
}

/* Read Received Byte*/
uint8_t I2C_Read(I2C_TypeDef* I2Cx)
{
	uint8_t Data;
		Data = I2C_ReceiveData(I2Cx);
		return Data;
}
//----------------------------------------------------------



//MPU I2C Functions
/* Config MPU6050 */
void MPU_Config(uint8_t Adress)
{
	I2C_Start_Transmitting(Adress<<1);
	I2C_Master_Write(0x19);
	I2C_Master_Write(0x07);
	I2C_Master_Write(0x00);
	I2C_Master_Write(0x00);
	I2C_Master_Write(0x00);
	I2C_Stop();
}

/* Wake MPU6050*/
void MPU_Wake(uint8_t Adress)
{
	I2C_Start_Transmitting(Adress<<1);
	I2C_Master_Write(0x6B);
	I2C_Master_Write(0x01);
	I2C_Stop();
}

/* Doc MPU6050 toan bo cac thanh ghi so lieu */
void MPU_ReadAll(uint8_t* temp, uint8_t Adress)
{
	uint8_t ReadCount;
	
	I2C_Start_Transmitting(Adress<<1);

	I2C_Master_Write(0x3B);

	I2C_Start_Receiving(Adress<<1);
	
	for(ReadCount = 0; ReadCount < 13; ReadCount++)
	{
	I2C_AcknowledgeConfig(I2C1, ENABLE);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	temp[ReadCount] = I2C_Read(I2C1);
	}
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);

	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	temp[13] = I2C_Read(I2C1);
}

/* Chuyen so lieu tu MPU6050 nhi phan sang so thuc*/
void MPU_GetRawData(uint8_t* temp, float* aX, float* aY, float* aZ, float* tC, float* gX, float* gY, float* gZ)
{
	int16_t temp_val;
		
	temp_val = temp[0]<<8|temp[1];
	*aX = temp_val/16384.0;
	
	temp_val = temp[2]<<8|temp[3];
	*aY = temp_val/16384.0;
	
	temp_val = temp[4]<<8|temp[5];
	*aZ = temp_val/16384.0;
	
	temp_val = temp[6]<<8|temp[7];
	*tC = temp_val/340.0 + 36.53;
	
	temp_val = temp[8]<<8|temp[9];
	*gX = temp_val/131.0;
	
	temp_val = temp[10]<<8|temp[11];
	*gY = temp_val/131.0;
	
	temp_val = temp[12]<<8|temp[13];
	*gZ = temp_val/131.0;
}
//---------------------------------------------------------------------------
