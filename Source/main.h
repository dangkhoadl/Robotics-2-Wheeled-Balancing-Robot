/******************************************************************************************
I2C 
						SCL PB 6
						SDA PB 9 
****************************************************************************************					
UART
	USART2		TX pin PA2
						RX pin PA3
	Baudrate	57600
	DMA				TX DMA1_Stream6 Channel 4
	Interupt	RX USART2_IRQHandler
****************************************************************************************
PWM
						PD12		TIM4_Chanel_1					FW_L
						PD13		TIM4_Chanel_2					FW_R
						PD14		TIM4_Chanel_3					BW_L
						PD15		TIM4_Chanel_4					BW_R
	
*******************************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

/* Include core modules */
#include "stm32f4xx.h"
#include "tm_stm32f4_mpu6050.h"
//#include "MPU6050_tm.h"
//#include "arm_math.h"
#include "UART_DMA.h"
#include "PWM.h"
#include "Encoder.h"
#include "filter.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//#define MPU_Adress 0x68
#define PI				 	3.14159265358979f
#define dt 				 	0.010f          //Sampling Interval
#define f_s				 	100.0f				 //Sampling rate = 1/dt						
#define MAX_TILT  	20.0f
#define MAX_VELO   	200.0f
#define MAX_POSI   	1.0f
//#define MAX_TURN    100.0f
#define TURN_OFFSET	0.05f
#define DUTY_OFFSET	0

/************************Global variables ----------------------------------------------------------*/

	//Tx
	#define txsize 7
	uint8_t txbuffer[txsize];

	//RX
	#define rxsize 21 //min = 3 channel * 7 bytes = 21
	uint8_t rxbuffer[rxsize];	
	
	//MPU 6050
	TM_MPU6050_t MPU6050_Data0;
	TM_MPU6050_Result_t MPU6050_Status;
	typedef struct
	{
						float raw;
						float filted_theta[2];
		const	  float theta_offset;
	} THETA_STRUCT;
	THETA_STRUCT  Pitch =  {0,{0,0},-2.24f};
	//THETA_STRUCT	Yaw   =  {0,{0,0},0};

	//Encoder
	typedef struct 
	{
		float 			 velo_fb;
		float				 posi_fb;
		int16_t 		 temp[2],
								 diff;
		TIM_TypeDef* timer;
	} WHEEL_ENCODER_STRUCT; 
	WHEEL_ENCODER_STRUCT 	 	 Left = {0,0,{0,0},0,TIM3};
	WHEEL_ENCODER_STRUCT		 Right= {0,0,{0,0},0,TIM5};
	
	typedef struct
	{
		float velo_raw;
		float posi_raw;
		float velo_filted[2];
		float posi_filted[2];
	}VELO_STRUCT;
	VELO_STRUCT Linear;
//	VELO_STRUCT Angular;
	
	//PID
	typedef struct
	{
				float   Kp;
				float   Ki;
				float   Kd;
				float   P_Part;
				float   I_Part;
				float   D_Part;
				float 	set_point;
				float		feedback;
const		float   maxVal;
				float 	e[3];
				float   U[2];
				float		filted_U[2];
	} PID_STRUCT;
	PID_STRUCT PID_POSI   		= {0.5,0.02,0,0,0,0,0,0,MAX_POSI,{0,0,0},{0,0},{0,0}};
	PID_STRUCT PID_VELO_HOLD 	= {1.55,0.15,0,0,0,0,0,0,MAX_VELO,{0,0,0},{0,0},{0,0}};
	PID_STRUCT PID_VELO_RUN 	= {1.55,0.15,0,0,0,0,0,0,MAX_VELO,{0,0,0},{0,0},{0,0}};
	PID_STRUCT PID_TILT				= {10,2.9,0.4,0,0,0,0,0,MAX_TILT,{0,0,0},{0,0},{0,0}};
	
	//Turning
	float setspeed=0;
	float setturn=0;
	float	U_offset=0;
		
	//PWM	
	typedef enum
	{
		LEFT = 0,
		RIGHT,
	}Motor_t;
	
	typedef struct
	{
		uint16_t duty;
		float		U0_100;							// -100 < U0_ref < 100
		float   PWM;
		Motor_t Side;
	}PWM_STRUCT;
	PWM_STRUCT	Left_motor  = {0,0,0,LEFT};
	PWM_STRUCT	Right_motor = {0,0,0,RIGHT};
	
	
/* Systick */
uint16_t SysTicker = 0;
void SEND_UART (uint16_t time);
void MAIN_PROG (uint16_t time);

/* Subroutines */
void ENC_PROCESSING (WHEEL_ENCODER_STRUCT *S);
void PID (PID_STRUCT *S,float LPFGain);
void PWM (PWM_STRUCT *S);
		void (*func1 ) (TIM_TypeDef* TIMx,uint32_t Compare1);
		void (*func2 ) (TIM_TypeDef* TIMx,uint32_t Compare1);
void ResetPID (PID_STRUCT *S);
void Delay(uint32_t nCount);// t = nCount (ms)
	
#endif 
