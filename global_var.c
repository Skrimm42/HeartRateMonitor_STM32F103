/* Global Variables 

  global_var.c
  
*/

#include "global_var.h"

uint16_t ADCConvertedValue;
uint16_t ADCConvv_buf[ADC_BUFF_SIZE];

bool wk_up_init;

uint16_t Tim1_buf[TIM_BUFF_SIZE];
bool tim_run;
    
uint16_t point_count;
uint16_t test_point_count[100];
