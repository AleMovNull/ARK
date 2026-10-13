/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"
#include <MDR32F9Qx_uart.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_port.h>
#include "MDR1986VE1T_it.h"
#include <MDR32F9Qx_timer.h>
#include "MDR32F9Qx_arinc429r.h"
#include "MDR32F9Qx_arinc429t.h"
#include "Init_APK25.h"
#include "APK25_Debug.h"
#include "APK25_SF.h"
#include "APK25_Angle.h"
#include "APK25_Chassis.h"
#include "APK25_FC.h"
#include "APK25_TransRec.h"


extern uint32_t Out_Word_Angl;  // enable (Angle.c)
extern uint32_t Out_Word_Frequency; // enable (SF.c)
extern uint16_t arinc_AB; // enable
extern uint8_t  phase;    // Фаза выполнения ФК 
uint32_t In_Word_ARINCA = 0; // enable
uint32_t In_Word_ARINCB = 0; // enable
uint32_t In_Word_ARINC = 0; // enable


vuint32_t ADC1_Value = 0;
vuint32_t TimerCounter = 0;

uint16_t  Addr = 0;
uint16_t Data = 0; 
uint8_t  buffer_is_1 = 0;




extern DATA switch_sin_0;
extern DATA switch_sin_1;
extern uint8_t  tick_ARINC_tr;
extern uint16_t tick_ARINC_rc;
extern uint16_t tick_device;
extern uint16_t tick_time;
extern uint16_t tick_time_ok;
extern uint16_t tick_time_pp;
extern uint16_t tick_tugl;
extern uint16_t tick_ctrl;   // Счетчик временных интервалов контроля 
extern uint16_t tick_f_ctrl; // Счетчик временных интервалов ФК
extern uint16_t tick_hog_sd; // Счетчик временных интервалов захвата СД 
extern uint16_t ID;          // Идентификатор приемника

uint8_t counter_ARINC = 0;  // Счетчик передач ARINC429
extern  uint32_t flags;
extern uint8_t state_working;
extern volatile uint32_t ReadBytes;
extern  uint32_t ReadArray[64];
extern float fr_kHz_real;

uint8_t stat_id = 0;
uint8_t r_byte = 0;
uint16_t size_mass = 0;    // Размер прочитанного массива 


void Delay_Phase(uint16_t value);


//---------------------------------------------------------------------------------------
/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{

	while (1);
}
//---------------------------------------------------------------------------------------
void SysTick_Handler(void)
{ 
   
}
//---------------------------------------------------------------------------------------
void Delay_Phase(uint16_t value)
{
	  uint16_t i = 0;
	  for(i = 0; i <value; i++) i = i;	
}
//---------------------------------------------------------------------------------------

void TIMER1_IRQHandler(void)
{
	  if(TIMER_GetITStatus(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH1) == SET)	
	  {
		    tick_device++;
			  tick_ARINC_tr++;
			  tick_time++;
			  tick_time_ok++;
			  tick_ctrl++;
			  tick_f_ctrl++;
			  tick_hog_sd++;
			  tick_time_pp++; 
			  tick_tugl++; 
			
			  if(tick_ARINC_rc<PERIOD_MORE_LOST)
				    tick_ARINC_rc++; 
			  // Захват сигнала АДС ВЫПОЛНЕНО (ADC_DONE0)
		    TIMER_ClearFlag(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH1);	
        
        		
		    
        if((state_working == COMPASS)||(state_working == CONTROL)||
					 (state_working == FUNC_CONTROL))
					
        {		
            MDR_ADC->ADC1_CFG  |= (1<<1);
					  while((MDR_ADC->ADC1_STATUS&(1<<2))!=(1<<2));
					  Data = (uint16_t)MDR_ADC->ADC1_RESULT&0x0FFF;
					  Data ^= 0x0FFF; 
					   
					  					
					
					  // Чтение адреса
			      Addr = (PORT_ReadInputData(MDR_PORTA) >> 7 ) & 0x1FF;
					  
					  // Чтение сигнала ПЕРЕКЛЮЧЕНИЕ SIN
			      buffer_is_1 = PORT_ReadInputDataBit(MDR_PORTA, BIT_SWITCH_SIN);
            
					  
					  // Заполнение буфера
	          if(buffer_is_1==Bit_SET)
     	          switch_sin_1._word[Addr] = Data;
				    else
		            switch_sin_0._word[Addr] = Data;
					  
					
					  /* 
						// Заполнение буфера
	          if(buffer_is_1==Bit_SET)
						{
     	           if((Addr + sd) < 512)
							       switch_sin_1._word[Addr+sd] = Data;
                 else
                     switch_sin_1._word[Addr+sd - 512] = Data;										
						}
				    else
						{ 
						     switch_sin_0._word[Addr] = Data;
						}								 
						*/
						
					  size_mass++;
						
				}		
				
				
				// Периодическая выдача сообщений
			  if(tick_ARINC_tr >= PERIOD_54_MS)
			  {
				    counter_ARINC++;
				    if(flags & ENABLE_FUNC_CONTR)
						{
						    if((phase == PHASE_TEST_ARINC_A)||
		               (phase == PHASE_TEST_ARINC_B)) 	
								{
									  if(arinc_AB == CHANNEL_A) // Версия платы 1.0
										{	
				                ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Frequency); 
										}	
									  else
	  	                  ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Frequency); 
								    tick_ARINC_tr = 0;
									  counter_ARINC = 0;
									  return;
									
								}
						}							
					  
						/*
					  switch(counter_ARINC)
					  {
					      case 1: case 2:
								    if(arinc_AB == CHANNEL_A)
				                ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Angl); 
									  else
	  	                  ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Angl); 
	                  break;
							  case 3:
								    if(arinc_AB == CHANNEL_A)
									  {	
								        ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Angl); 
		                    ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Frequency);
									  }
									  else
									  {	
          			        ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Angl);
											  ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Frequency); 
									  }	
                    counter_ARINC = 0;
	                  break;									
					  }
						*/
						
						if((state_working == ANTENNA_TLF)||(state_working == ANTENNA_TLG))
						    Out_Word_Angl = 0x28000272;	
						
						
						switch(counter_ARINC)
					  {
					      case 1: case 2:
								    ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Angl); 
									  ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Angl); 
	                  break;
							  case 3:
									  ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Angl); 
		                ARINC429T_SendData(ARINC429T_CHANNEL1,Out_Word_Frequency);
									  ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Angl);
										ARINC429T_SendData(ARINC429T_CHANNEL2,Out_Word_Frequency); 
									  counter_ARINC = 0;
	                  break;									
					  }
						
						
				    tick_ARINC_tr = 0;
			  }
    } // end TIMER_GetITStatus(_CH1)
} // end 	TIMER1_IRQHandler();	

//---------------------------------------------------------------------------------------
void ARINC429R_IRQHandler(void)
{
	  uint16_t receiv_ID = 0; 
	
	  if(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL1, ARINC429R_FLAG_ERR) == SET)
	  {
		    // Сброс флага
		    ARINC429R_ITConfig(ARINC429R_IT_INT_ER, DISABLE);
		    // Переключение канала
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, DISABLE);
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, ENABLE);
		    return;
	  }	
	
	  if(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL2, ARINC429R_FLAG_ERR) == SET)
	  {
		    // Сброс флага
		    ARINC429R_ITConfig(ARINC429R_IT_INT_ER, DISABLE);
		    // Переключение канала
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, DISABLE);
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, ENABLE);
		    return;
	  }
	    
	  if(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL1, ARINC429R_FLAG_DR) == SET)
	  {
	      // Собственно прием сообщеия
			  ARINC429R_SetChannel(ARINC429R_CHANNEL1);
	      while(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL1, ARINC429R_FLAG_DR) == SET)
		    In_Word_ARINCA = ARINC429R_ReceiveData();
			  receiv_ID = In_Word_ARINCA & MASK_IDENTIFIER;
				// Прием сообщений с ID приемника или ID равным нулю
				if((receiv_ID == ID)||(receiv_ID == 0))
				{	
				    In_Word_ARINC = In_Word_ARINCA;
            tick_ARINC_rc = 0;
            flags &= (0xFFFFFFFF - FAIL_ARINC_RC);	
					  //Is_Telephon(In_Word_ARINC);
				}	
	  }	
		
	  if(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL2, ARINC429R_FLAG_DR) == SET)
	  {
	      // Собственно прием сообщения
			  ARINC429R_SetChannel(ARINC429R_CHANNEL2);
	      while(ARINC429R_GetFlagStatus(ARINC429R_CHANNEL2, ARINC429R_FLAG_DR) == SET)
		    In_Word_ARINCB = ARINC429R_ReceiveData();
					
				receiv_ID = In_Word_ARINCB & MASK_IDENTIFIER;
				if((receiv_ID == ID)||(receiv_ID == 0))
				{
				    In_Word_ARINC = In_Word_ARINCB; 
				    tick_ARINC_rc = 0;
				    flags &= (0xFFFFFFFF - FAIL_ARINC_RC);            
					  //Is_Telephon(In_Word_ARINC);
				}	
				
	  }

    // При высталении частоты вне (ОК и ФК) и при частоте внутри диапазона 
 		if(((flags & ENABLE_FUNC_CONTR)==0)&&((flags & ENABLE_OPER_CONTR)==0)&&((fr_kHz_real>=150.0)&&
			                                                                      (fr_kHz_real<=1750.0)))
		
		    Out_Word_Frequency = In_Word_ARINC;//&0x1FFFFFFF;
}
//---------------------------------------------------------------------------------------
void UART1_IRQHandler(void)		   
{
    if (UART_GetITStatusMasked(MDR_UART1, UART_IT_RX) == SET)
    {
        UART_ClearITPendingBit(MDR_UART1, UART_IT_RX);
	       // Processing on inerruption
	      Receive_Debug_Packet(UART_ReceiveData(MDR_UART1)&0xFF);        	 
	      Work_Debugger();    
    }
}
//---------------------------------------------------------------------------------------








