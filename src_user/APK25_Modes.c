#include "APK25_Modes.h"

#include "APK25_TransRec.h"
#include "APK25_SF.h"
#include "APK25_FC.h"
#include "APK25_Chassis.h"
#include "APK25_QBUS.h"
#include "Init_APK25.h"
#include "APK25_Angle.h"
#include "APK25_Filtr.h"


//--------------------------------------------------------------------
 
extern uint32_t flags;                      // Регистр флагов  
extern uint8_t state_working;               // Режим работы АРК-25
extern uint16_t tick_ARINC_rc;              // Счетчик импульсов адреса для тактирования пачек ARINC429 на прием
extern uint32_t In_Word_ARINC_0;            // Предыдущее слово Частота
extern uint8_t  phase;
extern uint16_t code_failure;               // Код ошибки
uint8_t old_state_working = NOTHING;        // Предыдущий режим работы АРК-25
uint8_t error_ARINC = 0;                    // Код ошибки при проверке канала ARINC429
extern uint32_t Cur_Word_Frequency;
extern uint16_t last;                       // Состоянии переферии  
extern uint8_t time_fc;
extern uint16_t tick_f_ctrl;                // Счетчик временных интервалов ФК
extern uint16_t size_mass;                  // Размер прочитанного массива
extern uint16_t load_f;                     // Загрузка ФНЧ оценок угла
extern ANALYS_ANGLE analys;
extern float fr_kHz_real;
extern uint16_t tick_time_ok;               // Переменная таймера


extern uint16_t frequency_0;	              // Код частоты первое значение
extern uint16_t frequency_00;	              // Код частоты последующее значение





//--------------------------------------------------------------------------
//   Отработка режима работы АРК-25
//--------------------------------------------------------------------------
void Work_Mode_APK(void)
{
    // Проверка разрешения ФК
	  if(tick_f_ctrl>=PERIOD_250_MS)
		{	
	      Is_Functional_Control_Enable();
			  tick_f_ctrl = 0;
		}	
    
			
		if(flags&ENABLE_FUNC_CONTR)	
	      state_working = FUNC_CONTROL;
		else
		{
			  if(state_working == FUNC_CONTROL)
				{
				    // Смена режима работы АРК-25
		        flags = 0; // Сброс всех флагов
					  //Init_L_PASS_Cheb_64();  
					  In_Word_ARINC_0 = 0; 
		        phase  = PHASE_START;
            code_failure = 0;	
					  last = 0;
				    Set_Register_State(ALL_LEDS_OFF);
					  // Анализ входного слова слова и проверка ЦПП
		        if(Is_Input_Message_Fail(Cur_Word_Frequency)==MESSAGE_OK)
		        {	
			          // Обработка входного сообщения Частота/Функция
			          Process_In_Word_ARINC(Cur_Word_Frequency);
		        }					
				}
    }			
		    	
    if(old_state_working != state_working)
	  {
	      // Смена режима работы АРК-25
		    flags = 0; // Сброс всех флагов
		    In_Word_ARINC_0 = 0; 
			  phase  = PHASE_START;
		    last = 0;
		    size_mass = 0;
        load_f = 0;        // Загрузка ФНЧ оценок угла			
			  Set_Telephon();
		    if(state_working == FUNC_CONTROL)
		    {
		        Set_Start_Message_Freq_Status();	
				    flags = ENABLE_FUNC_CONTR; 
				    time_fc = 0;	
            frequency_00 = 0;					
        }
				analys.loaded = 0;	
        analys.i_angle_mass = 0;
        analys.i_d = 0;		
        analys.ready = 0;	
        
        // Формируем сообщение КУРС для передатчика
	      Word_Angl_WR(ARINC_ANGLE_NO_DATA, // Матрица состояния
	                   CODE_90_DEGREES);    // Курсовой угол
				// Получение сигналов управления стрелочным прибором	    
	      Get_Code_Arrow_Device(2048);	
        // Указатель на 90 градусов
        Set_DAC_Angle(); 				
    }
			
	  switch(state_working)
    {
	      case FUNC_CONTROL:
			      Mode_Functional_Control();
			      break;
		    case COMPASS:
					  if((fr_kHz_real<150.0)||(fr_kHz_real>1750.0))
					  {	
	              Word_Angl_WR(ARINC_ANGLE_NO_DATA, 
	                           CODE_90_DEGREES);
		            Get_Code_Arrow_Device(2048);	
                Set_DAC_Angle();								
						}
            else						
				        Work_Mode_Compass();
				    Out_Word_Frequency_WR(ARINC_FR_OK);
				    break;				
		    case ANTENNA_TLF:					  
		    case ANTENNA_TLG:
					  Work_Mode_Antenna();
				    Out_Word_Frequency_WR(ARINC_FR_OK);
	          break;
        case CONTROL:
					  Tugle_Frequency_OK();
			      Mode_Operative_Control();
            break;					
				default:
					  // Формируем сообщение КУРС для передатчика
	          Word_Angl_WR(ARINC_ANGLE_NO_DATA, // Матрица состояния
	                       CODE_90_DEGREES);    // Курсовой угол
				    // Получение сигналов управления стрелочным прибором	    
	          Get_Code_Arrow_Device(2048);	
            // Указатель на 90 градусов
            Set_DAC_Angle(); 
				    break;
				
    }
    old_state_working = state_working;		
}
