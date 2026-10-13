#include "APK25_TransRec.h"


#include "APK25_SF.h"
#include "APK25_FC.h"
#include "APK25_Chassis.h"
#include "APK25_QBUS.h"
#include "Init_APK25.h"
#include "APK25_Angle.h"

//--------------------------------------------------------------------

//extern uint16_t register_D18D19; //enable
//extern uint16_t register_D28D29; // enable
extern uint32_t In_Word_ARINC;
extern uint16_t receiver_D27D30;            
extern uint32_t flags;                      // Регистр флагов  
extern uint8_t  state_working;              // Режим работы АРК-25
extern uint32_t Out_Word_Angl;              // Выходное слово КУРС
extern uint32_t Out_Word_Frequency;         // Выходное слово Частота
extern uint16_t receiver_D26;
extern uint16_t ID;                         // Идентификатор приемника
extern uint16_t tick_ARINC_rc;              // Счетчик импульсов адреса для тактирования пачек ARINC429 на прием
extern uint16_t code_failure;               // Код отказа
extern uint16_t frequency;	                // Код частоты в двоично-десятичном коде
extern float    frequency_kGz;
extern uint16_t input_frequency;            // Входная частота из ЦВС
extern uint16_t arinc_AB;                   // Выбор канала приемного канала ARINC429
extern uint8_t  phase;                      // Фаза выполнения ФК
uint32_t arinc_fail_fr = 0;                 // Слово для частот за пределами диапазона

float fr_kHz_real = 0;
uint16_t tick_tugl = 0;


//--------------------------------------------------------------------------
//   Формирование сообщения КУРС для передатчика
//--------------------------------------------------------------------------
void Word_Angl_WR( uint32_t state,       // Матрица состояния
	                 uint16_t code_angle)  // Курсовой угол
{
    Out_Word_Angl = 0;
    // Ввод кода угла
	  Out_Word_Angl = (code_angle&0x1FFF);
    Out_Word_Angl <<= 16;
    // Ввод матрицы состояния
    Out_Word_Angl |= state;
	  // Ввод идентификатора
	  Out_Word_Angl |= ID;
	  // Ввод метки (адреса)
	  Out_Word_Angl += 0x72;
}
//--------------------------------------------------------------------------
//   Формирование сообщения Частота для передатчика
//--------------------------------------------------------------------------
void Out_Word_Frequency_WR( uint32_t state)       // Матрица состояния
{	                          
    if(fr_kHz_real==0.0)
		{
        Out_Word_Frequency  = ZERO_POINT_ZERO;
			  // Ввод идентификатора
        Out_Word_Frequency |= ID;
			  // Ввод матрицы состояния
	      Out_Word_Frequency |= state;
        return;
		}			
    
  	if(fr_kHz_real==0.5)
		{ 	 
	      if(tick_tugl>1300)
				    Out_Word_Frequency = ZERO_POINT_5;	
				if(tick_tugl>2600)
				{	
				    tick_tugl = 0;
					  input_frequency = 0;
					  Out_Word_Frequency = ZERO_POINT_ZERO; 
				}
				// Ввод идентификатора
        Out_Word_Frequency |= ID;     
				// Ввод матрицы состояния
	      Out_Word_Frequency |= state;
				return;
		}
    		
		if((fr_kHz_real<150.0)||(fr_kHz_real>1750.0))
	  { 	 
	      if(tick_tugl>1300)
				    Out_Word_Frequency = In_Word_ARINC;    
       	if(tick_tugl>2600)
				{	
				    tick_tugl = 0;
					  input_frequency = 0;
					  Out_Word_Frequency =  CLEAR_INDICATOR; 
            // Ввод идентифкатора
            Out_Word_Frequency |= ID;	
            // Ввод матрицы состояния
	          Out_Word_Frequency |= state; 							
				}           				
				return;
		}
     
    // Частота в рабочем диапаоне	
    Out_Word_Frequency = 0;	
	  // Ввод двоично-десятичного кода частоты 
	  Out_Word_Frequency = input_frequency;
	  Out_Word_Frequency <<= 13;
	  // Ввод матрицы состояния
	  Out_Word_Frequency |=  state;
	  // Ввод режима АРК
	  if(state_working != COMPASS)
        Out_Word_Frequency |= ANTENNA_COMPASS;	
	  if(state_working == CONTROL)
		    Out_Word_Frequency &= (0xFFFFFFFF - ANTENNA_COMPASS);	   
		// Ввод подрежима АРК
		if(state_working == ANTENNA_TLG)
        Out_Word_Frequency |= TELEPHON_TELEGRAPH;
    // Ввод идентификатора
	  Out_Word_Frequency |= ID;
	  // Ввод метки (адреса)
	  Out_Word_Frequency += 0x1A;
}


//--------------------------------------------------------------------------
//Формирование сообщения Частота для передатчика для OK
//--------------------------------------------------------------------------
void Out_Word_Frequency_WR_OK(void)
{	                          
    // Признак отсутствия отказа
		Out_Word_Frequency = 0x03FFC000;			    
    Out_Word_Frequency |= ARINC_FR_FC;
	  // Ввод идентификатора
    Out_Word_Frequency |= ID;
	
    // Ввод метки (адреса)
    Out_Word_Frequency += 0x1A;	   
}
//--------------------------------------------------------------------------
//Формирование сообщения Частота для передатчика для ФK
//--------------------------------------------------------------------------
uint8_t Out_Word_Frequency_WR_FC(void)
{	                          
    uint16_t code_fail = 0;
	  Out_Word_Frequency = 0;
	
	  
	  if(code_failure == ERROR_NO)
    {			 
	      // Признак отсутствия отказа
		    Out_Word_Frequency = 0x03FFC000;			    
    }
    else
    {
	      // Подключение кабеля с шиной QBUS не рассматривается
	      // Ввод кода выявленной неисправности
	      if(code_failure >= 0x0008)
	      {
		        if(code_failure >= 0x0018)
            {
                if(code_failure >= 0x001A)
				            code_fail = CODE_FAIL_RECEIVER;
                else
                    code_fail = CODE_FAIL_ANTENNA;									 
			      }
            else
                code_fail = CODE_FAIL_RECEIVER;	 				 
        }
        else
            code_fail = code_failure;
			   
			  
			
			
        // Моя добавка из ФК без учета подключения кабеля QBUS
		    if(flags & FAIL_ARINC_RC)
		        code_fail = CODE_FAIL_ARINC;	
		 
        // Операции с кодом отказа
		    Out_Word_Frequency = code_fail;
        Out_Word_Frequency <<= 14;	
         
		    Out_Word_Frequency  |= 0x03C00000;
        if((Out_Word_Frequency & 0x003C0000) == 0)  				 
			       Out_Word_Frequency |= 0x003C0000;
	   }		
     // Ввод идентификатора
     Out_Word_Frequency |= ID;
		 // Ввод режима контроля
     Out_Word_Frequency |= ARINC_FR_FC;
     // Ввод метки (адреса)
     Out_Word_Frequency += 0x1A;
     return PHASE_DELAY_000540;//PHASE_DELAY_007616;		 
	   
}
//--------------------------------------------------------------------
//    Формирование стартового выходного сообщения Частота/Состояние
//--------------------------------------------------------------------
void Set_Test_Message_Freq_Status(void)
{
    if(arinc_AB == CHANNEL_A)
	      Out_Word_Frequency = TEST_FRQ_MESSAGE_A;
    else
		    Out_Word_Frequency = TEST_FRQ_MESSAGE_B;
	// Ввод ID
	Out_Word_Frequency |= ID;		
}
//--------------------------------------------------------------------
//    Формирование стартового выходного сообщения Частота/Состояние
//--------------------------------------------------------------------
void Set_Start_Message_Freq_Status(void)
{
    /*
	  // Состояние АРК - неиспользуемая комбинация
    // Частота - дефектная
	  Out_Word_Frequency = START_FRQ_MESSAGE;  
	  // Ввод ID
	  Out_Word_Frequency |= ID;		
	  */
}

//--------------------------------------------------------------------
//    Быстрый анализ подрежима Телефон/Телеграф
//--------------------------------------------------------------------
void Is_Telephon(uint32_t message)
{
    // Определение состояния и режима работы АРК-25
		switch(message&UNUSED_MESSAGE)
		{
			  case NORMAL_WORK:      // Нормальное функционирование
					  if(message & ANTENNA_COMPASS) 
		        {
		            if(message & TELEPHON_TELEGRAPH)
								{	
				            // Включение режима Телеграф
					          if((flags & SET_MODE_TLPH)== SET_MODE_TLPH)
					              Set_Telegraph();
								}	
						    else
								{	
                    // Включение режима Телефон
					          if((flags & SET_MODE_TLGR)== SET_MODE_TLGR)
					              Set_Telephon();
                } 									
		        }
		        break;
				
		}		
}



uint32_t Check_Freq_Range(uint32_t message)
{
    uint32_t code_fr = 0;
	  uint8_t tet = 0;
	  uint32_t value = 0;
	  uint32_t ret = 0; 
	  fr_kHz_real = 0.0;
	  	  
	  ret =  message&UNUSED_MESSAGE; 		
	
	  // Анализ трех тетрад кода частоты
	  code_fr = message;
		if(code_fr&BIT13)
		    fr_kHz_real = 0.5;
		    
		code_fr = (message>>14);    
	  tet = code_fr&MASK_TETRADA; 	
		
		
		 
		if(tet>=10) 	return message; 
		fr_kHz_real += tet; 
		
    code_fr >>= 4;
		tet = code_fr&MASK_TETRADA; 
		if(tet>=10)   return message; 
		fr_kHz_real += tet*10;
		
		code_fr >>= 4;
		tet = code_fr&MASK_TETRADA; 
		if(tet>=10)  return message; 
		
		fr_kHz_real += tet*100;
		code_fr >>=4; 
		// Анализ кода МГц
		tet = code_fr&0x3;
		fr_kHz_real += tet*1000;
		
		if(fr_kHz_real<150.0)
		{	
		    value = 0; // единицы
			  value <<= 14;
        ret += value;
        value = 5; // десятки
        value <<= 18;
        ret += value;
        value = 1; // сотни
        value <<= 22;
        ret += value;			
		    return ret;	
		}
    if(fr_kHz_real>1750.0)
		{
		    value = 0; // единицы
			  value <<= 14;
        ret += value;
        value = 4; // десятки
        value <<= 18;
        ret += value;
        value = 7; // сотни
        value <<= 22;
        ret += value;	
        value = 1; // тысячи
        value <<= 26;
        ret += value;			
		    return ret;			
    } 		
		return message;

}
//--------------------------------------------------------------------
//    Анализ входного слова и и проверки ЦПП
//    Функция возвращает MESSAGE_FAIL в случаях, когда входное сообщение
//    - содержит адрес не 0x58;
//    - содержит значениеID, не установленное на шасси;
//    - содержит ошибки кодирования в тетрадах двоично-десятичного кода частоты
//    - cодержит ошибку в кодировании МГц
//--------------------------------------------------------------------
uint8_t Is_Input_Message_Fail(uint32_t message)
{
    uint32_t code_fr = 0;
	  uint8_t tet = 0;
	  
	
	  float fr_kHz = 0;
	
	  // Слово ARINC для частот за пределами рабочего диапазона  
	  arinc_fail_fr = 0; 
	  	  
	  // Ошибка уже обнаружена
	  if((flags & ENABLE_FUNC_CONTR)|| (flags & ENABLE_OPER_CONTR))
	  {
		    if(code_failure) return ERROR_NO;
		}	
	
	  // Сброс кода отказа
	  code_failure = ERROR_NO;
	
	  if(flags & ENABLE_FUNC_CONTR) 
	  {	
	      // Блокировка анализа в фазах ФК
		    if((phase == PHASE_TEST_ARINC_A)||
		       (phase == PHASE_TEST_ARINC_B))
		    return MESSAGE_OK;
	  }

	  // Блокировка анализа сообщений при проверке поддиапазонов
	  if(flags & DISABLE_ANALYS_MESSAGE)
	  {
		    return MESSAGE_OK; 
    }	  		
	
	  // Поскольку используется аппаратная фильтрация по адресу и ID
    if(tick_ARINC_rc >= PERIOD_LOST)
	  {	
	      flags |= FAIL_ARINC_RC;
		    if(flags & ENABLE_FUNC_CONTR)
		    {
			      // При сигнале Подключение КПА = 0 (кабель QBUS подключен)
			      // code_failure = ERROR_NO_INFO_FROM_INPUT;
			      // return ERROR_NO_INFO_FROM_INPUT;
		        code_failure = CODE_FAIL_ARINC;
			      return CODE_FAIL_ARINC;
		    }
		    else
		    {
			      code_failure = ERROR_NOT_DATA_FROM_PULT;
			      return ERROR_NOT_DATA_FROM_PULT;			
		    }
	  }	
		else
        flags &= (0xFFFFFFFF - FAIL_ARINC_RC);	
     			 
    
		// Определение состояния и режима работы АРК-25
		switch(message&UNUSED_MESSAGE)
		{
			  case UNUSED_MESSAGE:   // Неиспользуемая комбинация					
				case DO_LAST_MESSAGE:  // Выполнять информацию последнего слова	
					  break;
				case NORMAL_WORK:      // Нормальное функционирование
					  if(message & ANTENNA_COMPASS) 
		        {
		            if(message & TELEPHON_TELEGRAPH)
								    state_working = ANTENNA_TLG;
							  else
							      state_working = ANTENNA_TLF;														
		        }
		        else 
						    state_working = COMPASS;
						if(message & TELEPHON_TELEGRAPH)
						{	
				        // Включение режима Телеграф
					      flags |=  SET_MODE_TLGR;
								flags &= (0xFFFFFFFF - SET_MODE_TLPH);	
					      Set_Telegraph();
						}	
						else
						{	
                // Включение режима Телефон
					      flags  |= SET_MODE_TLPH;
								flags  &= (0xFFFFFFFF - SET_MODE_TLGR);
					      Set_Telephon();
            } 
						break;
				case BIT_CONTROL:      // Контроль
					  state_working = CONTROL;
					  break;
		}			
		    
		// Анализ трех тетрад кода частоты
	  code_fr = message;
		if(code_fr&BIT13)
        fr_kHz = 0.5;   			
    
		code_fr = (message>>14);
	  tet = code_fr&MASK_TETRADA; 	
		
		if(tet>=10)
		{	
		    flags |= FAIL_INPUT_MESSAGE;
			  code_failure = ERROR_CODE_FREQUENCY;
			  return ERROR_CODE_FREQUENCY; 
		}	
		fr_kHz += tet; 
		
    code_fr >>= 4;
		tet = code_fr&MASK_TETRADA; 
		if(tet>=10)
		{	
		    code_failure = ERROR_CODE_FREQUENCY;
			  flags |= FAIL_INPUT_MESSAGE;
			  return ERROR_CODE_FREQUENCY; 
		}
		fr_kHz += tet*10;
		
		code_fr >>= 4;
		tet = code_fr&MASK_TETRADA; 
		if(tet>=10)
		{	
		    code_failure = ERROR_CODE_FREQUENCY;
			  flags |= FAIL_INPUT_MESSAGE;
			  return ERROR_CODE_FREQUENCY; 
		}
		fr_kHz += tet*100;
		code_fr >>=4; 
		// Анализ кода МГц
		tet = code_fr&0x3;
		fr_kHz += tet*1000;
		
		fr_kHz_real = fr_kHz;
		
		
		if(fr_kHz>1750.0)
		    return ERROR_UP_RANGE_FREQUENCY;		
     		
		
 		return MESSAGE_OK;
}
