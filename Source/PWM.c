/******************************************************************************************************



******************************************************************************************************/
#include "stm32f4xx.h"
#include "PWM.h"


void TIM_Config(void)
{
  /* Configure the TIM4 Output Channels */

	GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* TIM4-APB1(42Mhz) clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIO D clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* GPIOD Configuration: */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	/* Connect TIM4 pins to AF*/  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  
    /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 16799; //PWM = (42Mhz*2)/{(16799+1)*(0+1)*(0+1)}=5kHz
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
}

void PWM_Config(void)
{
  /* Configure PWM mode for TIM4 Channels 1->4*/
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
  /*PWM Mode Init configuration*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	////////////////////////////////// TIM4////////////////////////////////
	//Init channel 1 
  TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//Init channel 2
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//Init channel 3
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	//Init channel 4
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE); /* TIM4 enable counter */
	TIM_CtrlPWMOutputs(TIM4, ENABLE); /* TIM4 Main Output Enable */
}
