#include "main.h"

int main(void) 
{
	/* Init */
	TIM_Config();
	PWM_Config();
	ENC_Config();
	UART_DMA_CONFIG(txbuffer,txsize,rxbuffer,rxsize,57600);
	USART_config ();
	/* MPU 1 */
	MPU6050_Status = TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_2G, TM_MPU6050_Gyroscope_250s);
	Delay (200);
	/* MPU2 */
	//I2C_Config();
	//MPU_Config(MPU_Adress);
	//MPU_Wake(MPU_Adress);
	
	if (SysTick_Config(SystemCoreClock / 1000))
  {/* Capture error */ while (1);}
	
	while (1);	
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
			static float   rx_data;
			static uint8_t rxindex=0,i;
		
			/* Read data from the receive data register */
			rxbuffer[rxindex] = USART_ReceiveData(USART2);
			rxindex++;
			if (rxindex == rxsize) rxindex=0;
			for (i=0;i<=rxsize-6;i++)
			{
				if ((rxbuffer[i] == 0xFF) && (rxbuffer[i + 6] == 0xFE))
				{
					memcpy(&rx_data,&rxbuffer[i+2],4);
					switch (rxbuffer[i+1])
					{
						case 0xE0:PID_VELO.Kp = (float)rx_data;break;
						case 0xE1:PID_VELO.Ki = (float)rx_data;break;
						case 0xE2:PID_VELO.Kd = (float)rx_data;break;
						case 0xE3:PID_TILT.Kp = (float)rx_data;break;
						case 0xE4:PID_TILT.Ki = (float)rx_data;break;
						case 0xE5:PID_TILT.Kd = (float)rx_data;break;
						case 0xE6:setspeed = (float)rx_data;break;
						case 0xE7:setpoint = (float)rx_data;break;
					}
				}
			}
	}
}

/************************Systick******************************************/
void SysTick_Handler(void)
{
	SysTicker ++;
	
  MAIN_PROG (10);
	SEND_UART (99);
}

uint16_t GetTicker(void)
{return SysTicker;}


void SEND_UART (uint16_t time)
{
	static uint16_t ticker = 0;
	
	if ((uint16_t)(ticker+time)== GetTicker()) 
	{
		ticker = GetTicker();
		
		SENDDATA(PID_VELO.set_point,txbuffer,0);
		SENDDATA(PID_VELO.feedback,txbuffer,1);
		SENDDATA(PID_TILT.set_point,txbuffer,2);
		SENDDATA(PID_TILT.feedback,txbuffer,3);
		SENDDATA(Left_motor.U0_100,txbuffer,4);
	}
}

void MAIN_PROG (uint16_t time)
{
	static uint16_t ticker = 0;
	
	if ((uint16_t)(ticker+time)== GetTicker()) 
	{
		ticker = GetTicker();
		
		//Encoder section
		RPM (&Left);
		RPM (&Right);
	
		Linear.raw = (Left.velo_fb - Right.velo_fb)/ 2.0f;
		Smooth_filter (Linear.raw,Linear.filted,0.1);
		PID_VELO.feedback = Linear.filted[0];
		
		/*MPU 1*/
		/* Read all data */
		MPU6050_Status = TM_MPU6050_ReadAll(&MPU6050_Data0);
		
		//Pitch
		/* Complementary filter*/
		Pitch.raw = (0.98f * (Pitch.raw - MPU6050_Data0.Gyroscope_Y * dt)) + (0.02f * (atan2f(MPU6050_Data0.Accelerometer_X, MPU6050_Data0.Accelerometer_Z) * 180.0f/PI));
		Smooth_filter (Pitch.raw,Pitch.filted_theta,0.25);
		PID_TILT.feedback = Pitch.filted_theta[0] - Pitch.theta_offset;
		
		//Yaw
//		Yaw.raw = - MPU6050_Data0.Gyroscope_Z;
//		/* HighPass filter*/
//		Yaw.filted_theta[0] = HPF(Yaw.raw,10,f_s);
//		PID_TURN.feedback = Yaw.filted_theta[0] - Yaw.theta_offset;
		
	
		
		if ((PID_TILT.feedback < 45) && (PID_TILT.feedback > -45))
		{
			//Preprocessing
			if ((setspeed-PID_VELO.set_point) > 0)
			{
				PID_VELO.set_point += 0.5f;
			}
			
			if ((setspeed-PID_VELO.set_point) < 0)
			{
				PID_VELO.set_point -= 0.5f;
			}
		
			//PID VELOCITY
			PID (&PID_VELO,0.34);
			PID_TILT.set_point = -PID_VELO.filted_U[0] * MAX_TILT;
		
			//PID TILTING	
			PID (&PID_TILT,0.36);
		
			//Turning	
			//PID (&PID_TURN,0.12);
			U_offset = -setpoint*TURN_OFFSET;
			
			
			//FUSION 
			Left_motor.PWM  = PID_TILT.filted_U[0] + U_offset;
			Right_motor.PWM = PID_TILT.filted_U[0] - U_offset;
				
			//PWM section
			PWM (&Left_motor);
			PWM (&Right_motor);
		}
		else
		{			
			TIM_SetCompare1(TIM4,0);
			TIM_SetCompare2(TIM4,0);
			TIM_SetCompare3(TIM4,0);
			TIM_SetCompare4(TIM4,0);
			GPIO_ResetBits(GPIOD,GPIO_Pin_12);
			GPIO_ResetBits(GPIOD,GPIO_Pin_13);
			GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			GPIO_ResetBits(GPIOD,GPIO_Pin_15);
			ResetPID (&PID_TILT);
			ResetPID (&PID_VELO);
		}
	}
}

void Delay(uint32_t nCount)
{
  nCount = 42000 *nCount;
	while(nCount--);// tdelay (ms)= nCount [(4/168Mhz*1000)*nCount*42000]
}

void PID (PID_STRUCT *S,float LPFGain)
{
	/*  P_part = Kp*e[0]
			I_Part = I_part + Ki*e[0]*dt
			D_Part = Kd* (e[0]-e[1])/dt
			u[n] = P_part + I_part + D_part*/
//		S->P_Part  = S->Kp * S->e[0];
//		S->I_Part += S->Ki * S->e[0]* dt;
//			if(S->I_Part>0.01f) S->I_Part = 0.01;
//			else if(S->I_Part <-0.01) S->I_Part = -0.01;
//		S->D_Part =  S->Kd * (S->e[0]-S->e[1])/dt;
//		S->U = S->P_Part + S->I_Part + S->D_Part;
//    /* Update e */
//    S->e[1] = S->e[0];
	
		S->e[0] = (S->set_point - S->feedback)/(2.0f*S->maxVal);
//	if((PID_VELO.e[0] > -0.1f)&&(PID_VELO.e[0]<  0.1f))  {PID_VELO.e[0]= 0;}
		if(S->e[0] < -1.0)  									{S->e[0]= -1.0;}
		if(S->e[0] > 1.0f)  									{S->e[0]= 1.0f;}

		S->P_Part =  S->Kp* 		(S->e[0] - 	S->e[1]);
		S->I_Part =  S->Ki*dt/2.0f*	(S->e[0] + 	S->e[1]);
		S->D_Part =  S->Kd/dt*   (S->e[0] - 2.0f*S->e[1] + S->e[2]);
	
		S->U[0] = S->U[1] + S->P_Part + S->I_Part + S->D_Part;
		
		/* Update e */
		S->e[2] = S->e[1];
		S->e[1] = S->e[0];
		/* Update U */
		S->U[1] = S->U[0];
		
		Smooth_filter (S->U[0],S->filted_U,LPFGain);
		if (S->filted_U[0] <-1) 				 		S->filted_U[0] = -1;
		if (S->filted_U[0] >1)					 		S->filted_U[0] =  1;
		
		
}

void RPM (WHEEL_ENCODER_STRUCT *S)
{
		S->temp[1] = S->temp[0];
		S->temp[0] = EncoderValue(S->timer);
		S->diff = S->temp[0] - S->temp[1];
		
		if (S->diff> 10000) S->diff=(S->temp[0]-0xFFFF)+(S->temp[1]);//overflow while counting down
		if (S->diff<-10000) S->diff=(0xFFFF-S->temp[1])+(S->temp[0]);//overflow while counting up
		
		S->velo_fb = (float)S->diff*(f_s)*60.0f/28056.0f;		// RPM = (pulse/28056/dt)*60 // encoder 334 pulses per round*4*21 (gear21)= 28056
}

void PWM (PWM_STRUCT *S)
{	
	if (S->Side == LEFT)
	{
		func1 = TIM_SetCompare1;
		func2 = TIM_SetCompare3;
	}
	if (S->Side == RIGHT)
	{
		func1 = TIM_SetCompare2;
		func2 = TIM_SetCompare4;
	}
	if (S->PWM>0)																								//Forward
			{
				S->duty = (uint16_t)(DUTY_OFFSET+ S->PWM *(16800-DUTY_OFFSET));
				
				func1(TIM4,S->duty);
				func2(TIM4,0);
			}
	else if (S->PWM<0)																					//Reverse
			{
				S->duty = (uint16_t)(-DUTY_OFFSET-S->PWM*(16800-DUTY_OFFSET));
		
				func1(TIM4,0);
				func2(TIM4,S->duty);
			}
			else 
			{
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,0);
				TIM_SetCompare3(TIM4,0);
				TIM_SetCompare4(TIM4,0);
			}
		S->U0_100 = S->PWM * 100; 
}

void ResetPID (PID_STRUCT *S)
{
		/* Clear the state buffer.  The size will be always 3 samples */
		memset(S->e, 0, 3u * sizeof(float));
}
