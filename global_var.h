//global_var.h

#ifndef __GLOBAL_VAR_H__
#define __GLOBAL_VAR_H__

#include "stm32f10x.h"
#include <stdbool.h>

#define ADC_BUFF_SIZE 20
#define TIM_BUFF_SIZE 200
#define N_TIMEOUT_W 50
#define N_TIMEOUT_S 50

extern uint16_t ADCConvertedValue;
extern uint16_t ADCConvv_buf[ADC_BUFF_SIZE];
extern bool wk_up_init;
extern uint16_t Tim1_buf[TIM_BUFF_SIZE];

extern bool tim_run;

#endif //__GLOBAL_VAR_H__
