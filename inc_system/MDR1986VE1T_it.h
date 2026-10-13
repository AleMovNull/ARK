/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MDR1986VE1T_IT_H
#define __MDR1986VE1T_IT_H

/* Includes ------------------------------------------------------------------*/
#include <MDR32F9Qx_config.h>
#include "types.h"



/* Timer counter */
extern vuint32_t TimerCounter;
/* Current value of the ADC1_RESULT register */
extern vuint32_t ADC1_Value;



void SysTick_Handler(void);
void TIMER1_IRQHandler(void);
void ARINC429R_IRQHandler(void);
void HardFault_Handler(void);
void UART1_IRQHandler(void);


#endif // MDR1986VE1T_IT_H



