// user.c

#include "user.h"
#include "global_var.h"


#define TIMEOUT 1000000

extern __IO void delay(__IO uint32_t nCount);

uint32_t Timeout_cntr;


void userapp(void)
{
  static uint16_t cnt;
  
  if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
  {
    if(cnt++ > N_TIMEOUT_S)/* Enters STANDBY mode */
    {
            //signal turnon
      bool sw = 1;
      for(uint8_t i = 0; i < 12; i++)
      {
        if(sw){
          GPIO_SetBits(GPIOB, GPIO_Pin_3);// LED Off
          
        }else{
          GPIO_ResetBits(GPIOB, GPIO_Pin_3);// LED On
        }
        sw = !sw;
        delay(500000);    
      }
      PWR_EnterSTANDBYMode();   
    }
  }else {
    cnt = 0;
  }
  
  USART_TxData();
  
  
}


void USART_TxData(void)
{
  uint8_t i;
  uint8_t ADC_Data_8bit;
  static bool comp_rez;
  static uint8_t j;
  bool b, b1;
  static bool b1_;
  
  for (i=0; i < ADC_BUFF_SIZE; i++)
  {
    ADC_Data_8bit = ADCConvv_buf[i] >> 4;
    
    USART_SendData(USART1, ADC_Data_8bit);
    // wait transmit buffer empty flag
    Timeout_cntr = 0;
    while((USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET) && (Timeout_cntr < TIMEOUT))
    {
      Timeout_cntr++;
    }
    if(Timeout_cntr > TIMEOUT)
    {
      GPIO_ResetBits(GPIOB, GPIO_Pin_3);
      while(1);//trap
    }
    
    //Kostyl'
    if(ADC_Data_8bit > ADC_COMPARE) b1 = 1;
    else b1 = 0;
    
    b = (b1 ^ b1_) & (!b1_);
    b1_ = b1;
    if(b)
    {
      test_point_count[j] = point_count;
      j++;
      if(j>99)
        j=0;  
      
      point_count = 0;
    }

    point_count++;
    
   //test
  }
}

