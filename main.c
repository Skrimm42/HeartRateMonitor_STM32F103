/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "global_var.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
__IO void delay(__IO uint32_t nCount);
void GPIO_Setup(void);
void ADC_Setup(void);
void USART1_Init(void);
void EXTINT_Setup(void);
void TIM1_Setup(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */



int main(void)
{
 
  //TIM3 - for ADC trig
  //PB.13 - LED
  //PA.07 - ADC1 input (2.5V max, Uref = 3.0V)
  
  GPIO_Setup();
  //if button pressed after delay, continue. Else goto standby
  delay(12000000L);
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET) PWR_EnterSTANDBYMode();
   
  //signal turnon (LED blinking)
  bool sw = 1;
  for(uint8_t i = 0; i < 7; i++)
  {
    if(sw){
    GPIO_SetBits(GPIOB, GPIO_Pin_3);// LED Off

    }else{
      GPIO_ResetBits(GPIOB, GPIO_Pin_3);// LED On
    }
    sw = !sw;
    delay(1200000);    
  }
  
  USART1_Init();
  TIM1_Setup();
  ADC_Setup();
  EXTINT_Setup(); 
  /* Infinite loop */
  while (1)
  {
   
  }
}

__IO void delay(__IO uint32_t nCount)
{
 while(nCount--)
  {
  }
   
}


void TIM1_Setup(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  DMA_InitTypeDef DMA_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;  
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
 /* Configure PA.08 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
   // TIM1 clock enable 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  // Enable the TIM1 global Interrupt  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_SetPriority(TIM1_CC_IRQn, 0xF);//min priority
  
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // Maximum (ARR)
  TIM_TimeBaseStructure.TIM_Prescaler = 2196;  //Prescaler = ((SystemCoreClock ) 900 kHz) - 1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //TIMx_CR1->CKD
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;// ?????? ??? TIM1 ? TIM8
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //every rising edge
  TIM_ICInitStructure.TIM_ICFilter = 0x0; //input signal filter 
  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  // To Reset capture register after capture occurs
  // Select the TIM1 Input Trigger: TI1FP1  
  TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);  // Select the slave Mode: Reset Mode 
  
   // For reset Frequency variable  if no input present
  //If Counter register reaches the CC2 register, interrupt is pending
  // in which Frequency var become zero.
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0xFFFE;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  
   //------DMA---------------------------------------------------------------
  // DMA clock enable  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* DMA1 channel2 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM1 -> CCR1));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Tim1_buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = TIM_BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
    
   /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  
  // TIM8 Update DMA Request enable  
  TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE);
  
   // TIM enable counter  
  TIM_Cmd(TIM1, ENABLE);
  
  // Enable the CC2 Interrupt Request  
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
}


void EXTINT_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI13 Line to PC.13 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void GPIO_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  
  
  
  /* GPIOA Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  
  /* Configure input pa.0 (wk_up) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   
  /* Configure PB.3 (LED) */
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // ENABLE Wake Up Pin
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  PWR_WakeUpPinCmd(ENABLE);
}


void ADC_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  NVIC_InitTypeDef    NVIC_InitStructure;
  
  // TIM3 Setup
  // TIM3 - update event used as TRGO signal to start ADC1 conversion 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 1439;          
  TIM_TimeBaseStructure.TIM_Prescaler = 99;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
  
  
  /* ADCCLK = PCLK2/6 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure PA.07 (ADC12_IN7) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //Enable DMA1 channel IRQ Channel 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0x0F);//Lowest
  
  
  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1 -> DR));
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvv_buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC_BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // Enable DMA1 Channel Transfer Complete interrupt
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_55Cycles5);
  
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  
  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
  
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  
  /* TIM1 counter enable */
  TIM_Cmd(TIM3, ENABLE);
  
  
}


void USART1_Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
  
  /* Configure USART1 Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure USART1 Rx as alternate function  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
//  tmp = GPIOA -> ODR;
//  tmp |= 0x00000004;
//  GPIOA -> ODR = tmp;
  
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* Configure USARTy */
  USART_Init(USART1, &USART_InitStructure);
  
  USART_Cmd(USART1, ENABLE);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}


#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
