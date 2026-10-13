
/*
    Программа АРК-25 
		
*/

#include "MDR1986VE1T.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_bkp.h"
#include "MDR32F9Qx_eeprom.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "Init_APK25.h"
#include "MDR32F9Qx_uart.h"
#include "MDR1986VE1T_it.h"
#include "APK25_Angle.h"
#include "MDR32F9Qx_arinc429r.h"
#include "MDR32F9Qx_arinc429t.h"
#include "APK25_QBUS.h"
#include "APK25_Chassis.h"
#include "APK25_SF.h"
#include "APK25_FC.h"
#include "APK25_TransRec.h"
#include "APK25_Modes.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int i = 0;         // Счетчик  
volatile DATA switch_sin_0;          // Буфер оценок АЦП для сигнала Коммутация SIN равного 0
volatile DATA switch_sin_1;          // Буфер оценок АЦП для сигнала Коммутация SIN равного 1   
uint16_t tick_ARINC_tr;     // Счетчик импульсов адреса для тактирования пачек ARINC429 на передачу
uint16_t tick_ARINC_rc;     // Счетчик импульсов адреса для тактирования пачек ARINC429 на прием
uint16_t tick_device;       // Счетчик импульсов адреса для тактирования периодов опроса переферии
uint16_t tick_time;         // Счетчик временных интервалов 
uint16_t tick_time_ok;      // Счетчик временных интервалов 
uint16_t tick_time_pp;      // Счетчик временных интервалов переходных процессов


uint32_t flags = 0;         // Регистр состояний 
uint16_t tick_ctrl = 0;     // Счетчик временных интервалов контроля 
uint16_t tick_f_ctrl = 0;   // Счетчик временных интервалов ФК 
uint16_t tick_hog_sd = 0;   // Счетчик временных интервалов захвата СД 

extern uint32_t Out_Word_Angl;  // enable (Angle.c)
extern uint32_t Out_Word_Frequency; // enable (SF.c)
extern uint16_t input_frequency;
extern uint32_t In_Word_ARINC_0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



extern uint8_t loaded;
extern uint8_t en_trans;
extern uint16_t ram_022064;  // accounting heading angle


extern uint16_t receiver_D27D30;            // enable
extern uint32_t In_Word_ARINC;              // enable 
extern uint8_t state_working;
extern uint16_t arinc_AB;                   // Состояние бита выбора канала ARINC429
extern uint16_t ID;                         // Идентификатор приемника

volatile uint32_t ReadBytes = 0;
uint32_t ReadArray[64];
uint16_t AD_test = 0;


extern uint16_t ram_022066; 
extern uint16_t Addr;
extern uint8_t  buffer_is_1;
										
uint32_t arinc_word = 0;
uint32_t arinc_fr = 0;



int main()
{
  	 // Инициализация АРК-25 при включении притания
	   // Конфигурация тактирования без умножения частоты
	   ClockConfig_Low();
	   // Конфигурация тактирования с умножением частоты
	   ClockConfig();
	   // Конфигурация портов
	   PortsConfig();
	   
	   // Инициализация АЦП
	   Init_ADC();
	   
	   // Инициализация шины QBUS
	   Init_QBUS();
	   // Инициализация шассиа
	   Init_Chassis();
	   // Контроллер завелся
	   Device_Work();
	   // Предварительный опрос переферии   11.11.2019
	   Check_Switch_ARC();
	   // Формирование начальных слов для передатчика ARINC429
	   Set_Start_Message_Freq_Status(); // пустышка
	   Word_Angl_WR( ARINC_ANGLE_NO_DATA,  // Матрица состояния
	                 2048);         // Курсовой угол
		 tick_device = 0;
	   // Инициализация интерфейса ARINC429
	   InitARINC();
	   // Инициализация отладчика
	   ConfigUART1();	
	   // Сброс буферов отсчетов АЦП
	   for(i = 0; i<SIZE_BUFFER_WORD; i++)
	   {
	    switch_sin_0._word[i] = 0; 
		  switch_sin_1._word[i] = 0; 
	   }
	   tick_ARINC_tr  = 0;
		 tick_ARINC_rc  = 0;
		 tick_device = 0;
		
		 // Инициализация канала захвата сигнала АЦП ВЫПОЛНЕНО
	   Timer1Config(1);             
	   NVIC_EnableIRQ(TIMER1_IRQn);
		 
		  
		 
    while(1)
	  {
        // Периодический опрос переферии 
	      if(tick_device >= PERIOD_250_MS)
        {				
            if(((flags & ENABLE_FUNC_CONTR)== 0)||
						   ((flags & ENABLE_OPER_CONTR)== 0)) 
			          Check_Switch_ARC();
			      tick_device = 0;
		    }
	
      
			  arinc_word = In_Word_ARINC; // определяется при приеме сигнала ARINC (прерывание MDR1986..)
			  // Частота на входе
			  Determ_Frequency_Input(arinc_word);
			
			  // Проверка диапазона
			  //arinc_word = Check_Freq_Range(arinc_word);
			
		    // Анализ входного слова слова и проверка ЦПП
				// В функции условия выбирается и включается ТЛФ-ТЛГ
		    if(Is_Input_Message_Fail(arinc_word)==MESSAGE_OK)
		    {	
			      // Обработка входного сообщения Частота/Функция
			      Process_In_Word_ARINC(arinc_word);				
		    }	
			  // Отработка режима 
		    Work_Mode_APK();
			  // Передача служебного сообщения
        Switch_Message();
		
	  }				
	 
}


