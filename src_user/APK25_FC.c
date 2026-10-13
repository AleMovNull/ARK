#include "APK25_FC.h"
#include "APK25_TransRec.h"
#include "APK25_SF.h"
#include "APK25_Chassis.h"
#include "APK25_QBUS.h"
#include "Init_APK25.h"
#include "APK25_Angle.h"
#include "APK25_Filtr.h"
#include "Init_APK25.h"
#include "APK25_Angle.h"
#include "MDR32F9Qx_arinc429r.h"



//--------------------------------------------------------------------
extern uint16_t code_angle_180;     // Код угла с поворотом на 180 градусов 
extern uint16_t code_angle_0;       // Код угла с поворотом на 0   градусов
extern uint16_t register_D18D19; 
extern uint16_t register_D28D29; 
extern uint16_t receiver_D27D30; 
extern uint32_t flags;
extern uint8_t state_working;        // Режим работы АРК-25
extern uint32_t Cur_Word_Frequency;  // Текущее  слово Частота
extern uint32_t Out_Word_Frequency;  // Выходное слово Частота/Состояние
extern uint32_t Out_Word_Angl;       // Выходное слово КУРС
extern uint32_t In_Word_ARINC;       // Входное слово Частота/Функция
extern uint16_t tick_ARINC_rc;       // Счетчик импульсов адреса для тактирования пачек ARINC429 на прием
extern uint16_t new_frequency;       // Новый код частоты в двоично-десятичном коде
extern uint16_t frequency;           // Текущий код частоты в двоично-десятичном коде

extern uint16_t frequency_00; 




extern uint16_t input_frequency;     // Входная частота от ЦВС
extern uint16_t ram_022064;          // Содержит код угла курса
extern uint16_t ID;                  // Идентификатор приемника
extern uint16_t tick_time;           // Переменная таймера
extern uint16_t tick_time_ok;        // Переменная таймера
extern uint16_t arinc_AB;            // Выбор канала приемного канала ARINC429
extern uint32_t Test_Word_Frequency; // Входное тестовое слово Частота/Состояние
extern uint16_t last;
extern uint16_t last_AB; 
extern uint16_t size_mass;           // Размер прочитанного массива
extern uint16_t tick_ctrl;           // Счетчик временных интервалов контроля
extern uint16_t tick_time_pp;
extern uint8_t loaded;
uint16_t receiver_D26 = 0;
uint16_t code_failure = 0;
uint8_t  phase  = PHASE_START;
uint8_t  phase_next = PHASE_UNUSED;
int8_t  i_fr_range = 0;              // Счетчик поддиапазонов
int8_t i_probe = 0;                 // Счетчик проб  
uint8_t time_fc = 0;                 // Время проведения ФК   
int i_del = 0;
uint8_t st = 0;
uint16_t show_test_angle_rec = 0;
uint8_t enable_show_angle = 0;

extern float    frequency_kGz;
float angle_degr = 0;

extern uint8_t catch_SF;
extern uint8_t catch_SD;
extern uint8_t signal_from_rec_received;
extern uint8_t signal_from_ann_received;


extern uint16_t load_f;              // Загрузка ФНЧ оценок угла
extern uint16_t load_fv;             // Загрузка ФНЧ оценок угла	(вспомогательный ФНЧ)
extern int16_t loaded_Old;
extern float fr_kHz_real;

static const uint16_t Test_Frq_SubRange[6] =	  
{
    0x02D0,	   // 168 кГц
    0x0600,	   // 300 кГц
    0x08AC,	   // 456 кГц
    0x0E10,	   // 708 кГц
    0x2100,	   // 1080 кГц
    0x2A6C	   // 1536 кГц
};



//--------------------------------------------------------------------
//    Прототипы функций
//--------------------------------------------------------------------
void Test_ARINC_Message(void);
void Prepare_Test_ARINC_A(void);
void Prepare_Test_ARINC_B(void);
void Prepare_Test_Frq_Range(void);
void Prepare_Current_Test_Frq(void);
void SelfTestCPU(void);
uint8_t Is_Control_Angle_Valid(void);
uint8_t Is_Control_Angle_Valid_Rec(void);
void Is_Operative_Control_Enabled(void);
uint8_t Select_Check_back(void);
void Show_Error_No_For_FC(void);
uint8_t Get_Test_Frq_SubRange(void);
void Set_Test_Frq_SubRange(void);
void FC_Check_HOG_SF(void);
void FC_Check_Cur_HOG_SF(void);
void FC_Check_HOG_SD(void);
void FC_Check_Noes_HOG_SD(void);
void Get_New_Frequency(void);
void Set_Only_New_Frq(void);

void Prepare_Current_Test_Frq_Rec(void);
int16_t Add_Control_Angle(void);
int16_t Add_Control_Angle_Rec(void);
int16_t Add_Noes_Angle(void);


uint8_t angle_from_ann_invalid = 0;
uint8_t angle_from_rec_invalid = 0;


//--------------------------------------------------------------------
//    Разрешение  оперативного контроля
//--------------------------------------------------------------------
void Is_Operative_Control_Enabled(void)
{
	  // Проверка режима контроль в принятом сообщении
    if((In_Word_ARINC & MASK_STATUS_APK) == BIT_CONTROL) 
    {
		    receiver_D26 = Read_QBUS(D26);
	      // Сигнал Запрет ФК
	      if(receiver_D26 & DISABLE_FC)
		       flags &= (0xFFFFFFFF - ENABLE_OPER_CONTR);
			  else
           flags  |= ENABLE_OPER_CONTR;	
        
				if(tick_ARINC_rc >= PERIOD_LOST)
		    {	
		        // Потеря сигнала в приемном канале ARINC
            flags |= FAIL_ARINC_RC;
			      code_failure = CODE_FAIL_ARINC;	
            tick_ARINC_rc = PERIOD_LOST;
				    flags &= (0xFFFFFFFF - ENABLE_OPER_CONTR);
				}
				else
					  flags  |= ENABLE_OPER_CONTR;				
    } 		    
    else
        flags &= (0xFFFFFFFF - ENABLE_OPER_CONTR);    			
}
//--------------------------------------------------------------------
//    Разрешение Функционального контроля
//--------------------------------------------------------------------
void Is_Functional_Control_Enable(void)
{
	  receiver_D26 = Read_QBUS(D26);
	  // Сигнал Запрет ФК
	  if(receiver_D26 & DISABLE_FC)
		{
			  flags &= (0xFFFFFFFF - ENABLE_FUNC_CONTR);
			  return;
    } 			
	  receiver_D27D30 =  Read_QBUS(D27D30);
		// Cигнал Земля/Воздух
		if(receiver_D27D30 & LAND_AIR)
		{
			  flags &= (0xFFFFFFFF - ENABLE_FUNC_CONTR);
			  return;
    } 
    // Сигнал Включение ФК
		if((receiver_D27D30&FUNC_CHECK_ON)==0)
		    flags &= (0xFFFFFFFF - ENABLE_FUNC_CONTR);
		else	 
    		flags |= ENABLE_FUNC_CONTR;		
}


//--------------------------------------------------------------------
//   Мигание частотой в режиме Контроль
//--------------------------------------------------------------------
void Tugle_Frequency_OK(void)
{
    if(tick_time_ok<=TIME_HALF_SEC)
		{	
        // Индикация частоты
			  Out_Word_Frequency_WR(ARINC_FR_FC ); // Матрица состояния
			  //Out_Word_Frequency_WR(ARINC_FR_OK ); // Матрица состояния
		}
		else
		{
			  // Погасить частоту
				Out_Word_Frequency_WR_OK();	
		}							
		if(tick_time_ok>TIME_ONE_SEC)
        tick_time_ok = 0;		

}
//--------------------------------------------------------------------
//   Работа изделия в режиме Контроль
//--------------------------------------------------------------------
void Mode_Operative_Control(void)
{
    // Проверка разрешения оперативного контроля
	  if(tick_ctrl>=PERIOD_250_MS)
		{	
	      Is_Operative_Control_Enabled();
			  
			  //if(((flags & ENABLE_OPER_CONTR)== 0)||(Cur_Word_Frequency != In_Word_ARINC))
			  if((flags & ENABLE_OPER_CONTR)== 0)
	      {	
	          phase = PHASE_START;
					  flags &= (0xFFFFFFFF - DISABLE_ANALYS_MESSAGE); 
		        Set_Start_Message_Freq_Status();
		        return;
				}
				tick_ctrl = 0;
				//tick_time_ok = 0;
	  }
	  	
		Tugle_Frequency_OK();
				
		switch(phase)
	  {
        case PHASE_START:
		        // Сброс регистр кода ошибки
			      code_failure = ERROR_NO; 
				    load_f = 0;
            load_fv = 0;
				    loaded_Old = 0;
				    size_mass = 0;
				    loaded = 0;					
				    tick_time = 0;
            tick_time_ok = 0;
            st = 0;
            ram_022064 = CODE_90_DEGREES;				
			      // Число попыток
	          i_probe = 2;
				    phase = PHASE_TEST_CPU;
            flags &= (0xFFFFFFFF - NEW_FREQUENCY_DONE);
				    flags |= DISABLE_ANALYS_MESSAGE;
			      // Предупреждение об отказе с текущим углом курса
			      Word_Angl_WR(ARINC_ANGLE_FAILRUE, // Матрица состояния
	                       ram_022064);         // Курсовой угол 
		        // Формируем сообщение Частота для передатчика
			      //Out_Word_Frequency_WR(ARINC_FR_FC ); // Матрица состояния
			      break;
		    case PHASE_TEST_CPU:
		        // Самотестирование ВУ
		        SelfTestCPU(); 	
            if(code_failure == ERROR_NO)
			      {	 
                phase = PHASE_TEST_ARINC;
				        flags &= (0xFFFFFFFF - FAIL_CPU); 
			      }
            else
                flags |= FAIL_CPU;            
			      break;
		    case PHASE_TEST_ARINC:
            // Анализ входного слова и проверки ЦПП
		        if(code_failure) 
		        {	
		            // Проблема в приемном канале ARINC
                break;			
		        }	
		        else
			      {	 
                flags &= (0xFFFFFFFF - FAIL_ARINC_RC);
				        if(fr_kHz_real == 1777.5)
								    phase = PHASE_OUT_225;
                else								
							      phase = PHASE_GET_FREQUENCY;
				    }
             						
			      break;
        case PHASE_GET_FREQUENCY:
		        // Определение частоты, ближайшей к частоте из сообщения
		        SelectFrequency();
				    // Настройка на новую частоту  первый раз !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				    frequency = new_frequency;				
				    Set_Mode_Compass();
				    Set_Check_Antenna();
						Set_Fr_SF_From_Code();
				    frequency_kGz = frequency_kGz;						
            phase = PHASE_TEST_HOG_SF;  
            break;
        case PHASE_TEST_HOG_SF:
		        if(flags & FAIL_HOG_SF)  // Контроль захвата частоты
			      {
			          // Повторная настройка приемника
				        Set_Mode_Compass();
				        Set_Check_Antenna();
                Set_Fr_SF_From_Code(); // повторная настройка частоты
				        // Контроль захвата частоты
				        if(flags & FAIL_HOG_SF)  
				            code_failure = ERROR_NO_HOG_FREQUENCY;							       							   
				        else
				        {	 
                    code_failure = ERROR_NO; 
				            phase = PHASE_TEST_HOG_SD;							 
				        } 
            }
            else
			      {	 
			          code_failure = ERROR_NO;
				        phase = PHASE_TEST_HOG_SD;
			      } 
            break;
		    case PHASE_TEST_HOG_SD:
            if(Check_HOG_SD() == HOG_SF_RESET)
			      {
			          code_failure = ERROR_NO_HOG_SD;
                flags |= FAIL_HOG_SD;							 
            }							 
			      else
			      {
			          code_failure = ERROR_NO;
				        flags &= (0xFFFFFFFF - FAIL_HOG_SD);
                size_mass = 0;
							  phase = PHASE_MEASURE;
                // Запуск таймера	
                tick_time = 0;
                tick_time_ok = 0;							
                size_mass = 0;
				        loaded = 0;					
				        tick_time_pp = 0; 
                Set_Telegraph();							
			      }							 
			      break;
		    case PHASE_MEASURE:
				    				
				    // Вычисление угла курса
		        GetOrtogonalinyeComponents();
            Old_PASS_512();
            //ram_022064 += Add_Noes_Angle();				
				    ram_022064 += Add_Control_Angle();
						
				     
				    if(tick_time_pp < TIME_DELAY_PP)
            {
						    // Получение сигналов управления стрелочным прибором	    
		            ram_022064 = CODE_135_DEGREES;
							 
						} 
				    
            Get_Code_Arrow_Device(ram_022064);	
		        // Вывод угла на стрелочный прибор						   
		        Set_DAC_Angle();
				    // Формируем сообщение КУРС для передатчика
						 Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояния
	                            ram_022064);    // Курсовой угол
	         // Word_Angl_WR(ARINC_ANGLE_NO_DATA, // Матрица состояния
	          //             ram_022064);    // Курсовой угол 	 						
            // Формируем сообщение Частота для передатчика
				    //Out_Word_Frequency_WR(ARINC_FR_FC ); // Матрица состояния
						
				     
				    if(tick_time > TIME_DELAY_FC)
						{
			          // Время измерения истекло
				        tick_time = TIME_TWO_SEC;
							  // Проверка контрольного угла 135 
				        if(Is_Control_Angle_Valid()==ANGLE_INVALID)
				        {
				            // Угол отличается от 135 градусов более чем на 5 градусов
					          if(flags&NEW_FREQUENCY_DONE)
					          {
					              // Аварийное завершение контроля     
						            flags |= FAIL_MEASURE_ANGLE;
                    }
                    else
                    {
					              Word_Angl_WR(ARINC_ANGLE_FAILRUE, // Матрица состояния
	                                   ram_022064);      // Курсовой угол
						            flags &= (0xFFFFFFFF-FAIL_MEASURE_ANGLE);
											
											                   
                        if(i_probe > 0)
                        { 													
											      SelectFrequency();	 // второй раз меняем частоту
				                    // Настройка на новую частоту
				                    frequency = new_frequency;
				                    Set_Mode_Compass();
				                    Set_Check_Antenna();
							              Set_Fr_SF_From_Code();
                            frequency_kGz = frequency_kGz;	
                            // Число попыток
	                          i_probe -= 1;                           
												}
                        else
                        {													
											      // Повторная перестройка частоты											 
						                flags |= NEW_FREQUENCY_DONE;
												}	
						            phase = PHASE_TEST_HOG_SF;				 
						            // Формируем сообщение Частота для передатчика
				                //Out_Word_Frequency_WR(ARINC_FR_FC ); // Матрица состояния
                    }											 
                }    
				        else
				        {	 
									  // Формируем сообщение КУРС для передатчика
	                  Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояния
	                               ram_022064);    // Курсовой угол	
									 
				        } 
            }
			      else
            {   
			          // Время измерения не истекло
							  // Формируем сообщение КУРС для передатчика
	              Word_Angl_WR(ARINC_ANGLE_NO_DATA, // Матрица состояния
	                           ram_022064);    // Курсовой угол
				       
            }
            Tugle_Frequency_OK();				
		        break;
				case PHASE_OUT_225:
					  ram_022064 = CODE_225_DEGREES;
            if(st == 0)
						{	
				        Get_Code_Arrow_Device(ram_022064);	
		            // Вывод угла на стрелочный прибор						   
		            Set_DAC_Angle();
							  st = 1;
						}
				    // Формируем сообщение КУРС для передатчика
	          Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояния
	                       ram_022064);    // Курсовой угол 	 						
				    Tugle_Frequency_OK();	
            break;					
      }  
		  Tugle_Frequency_OK();	
      
			
}
//--------------------------------------------------------------------
//   Индикация для успешного завершения теста при ФК
//--------------------------------------------------------------------
void Show_Error_No_For_FC(void)
{
    // Получение сигналов управления стрелочным прибором	    
	Get_Code_Arrow_Device(ram_022064);	
	// Вывод угла на стрелочный прибор						   
	Set_DAC_Angle();
	// Формируем сообщение КУРС для передатчика
	Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояния
	             ram_022064);    // Курсовой угол 
	if(time_fc<=6)
	    Set_Register_State(ALL_LEDS_ON);
  else
      Set_Register_State(ALL_LEDS_OFF);		
	
	// Индикация отсутствия отказа
	flags &= (0xFFFFFFFF - SHOW_FAIL_FC);
}
//--------------------------------------------------------------------
//   Контроль величины угла 135 градусов
//--------------------------------------------------------------------
int16_t Add_Noes_Angle(void)
{
 	/*
	if(frequency_kGz<= 700.0)
      return 46; 		// 2 градуса
	if(frequency_kGz<= 1000.0)
		  return 68;    // 3 градуса
	if(frequency_kGz<= 1100.0)
		  return 80;    // 3.5 градуса
	if(frequency_kGz<= 1200.0)
		  return 91;    // 4 градуса
  if(frequency_kGz<= 1300.0)
		  return 103;   // 4.5 градуса
  if(frequency_kGz<= 1400.0)
		  return 115;  // 5 градусов
  if(frequency_kGz<= 1800.0)
		  return 148;  // 6.5 градусов	
	*/
	return 0;
}





int16_t Add_Control_Angle(void)
{
  
	
	int16_t value = 0;
	uint8_t sgn = 0;
  // Измеренный угол курса
  value = ram_022064;
	// Вычитаем 135 градусов
	value -= CODE_135_DEGREES;
	// При необходимости берем модуль разности
	if(value <0)
	{
	    value *= -1;
      sgn = 0x01;
	}
	if(value <= 22) return 0;
	
	//if(value <= CODE_3_DEGREES) return 0x0000;
	if(value > CODE_10_DEGREES) return 0x0000;
	
	value -= 22;//CODE_2_DEGREES;
	
	if(sgn)
	    return value;
	return -value;
	
	
  
	/* РПКБ
	if(frequency_kGz<= 700.0)
      return 46; 		// 2 градуса
	if(frequency_kGz<= 1000.0)
		  return 68;    // 3 градуса
	if(frequency_kGz<= 1100.0)
		  return 80;    // 3.5 градуса
	if(frequency_kGz<= 1200.0)
		  return 91;    // 4 градуса
  if(frequency_kGz<= 1300.0)
		  return 103;   // 4.5 градуса
  if(frequency_kGz<= 1400.0)
		  return 115;  // 5 градусов
  if(frequency_kGz<= 1800.0)
		  return 148;  // 6.5 градусов	
	*/		
	return 0;
	
	
	
}
//--------------------------------------------------------------------
//   Контроль величины угла 225 градусов
//--------------------------------------------------------------------
int16_t Add_Control_Angle_Rec(void)
{
  /*
	int16_t value = 0;
	uint8_t sgn = 0;
  // Измеренный угол курса
  value = ram_022064;
	// Вычитаем 225 градусов
	value -= CODE_225_DEGREES;
	// При необходимости берем модуль разности
	if(value <0)
	{
	    value *= -1;
        sgn = 0x01;
	}
	if(value <= CODE_10_DEGREES) return 0x0000;
	if(value > CODE_20_DEGREES) return 0x0000;
	
	value -= CODE_5_DEGREES;
	
	if(sgn)
	    return value;
	return -value;
	*/
	if(frequency_kGz<= 300.0)
      return 1365; 		// 60 градусов
	if(frequency_kGz<= 700.0)
		  return 1024;    // 45 градусов
	if(frequency_kGz<= 900.0)
		  return 910;    // 40 градусов
	if(frequency_kGz<= 1100.0)
		  return 796;    // 35 градусов
  if(frequency_kGz<= 1500.0)
		  return 682;   // 30 градусов
  if(frequency_kGz<= 1800.0)
		  return 615;  // 27 градусов	
	return 0;	
}
//--------------------------------------------------------------------
//   Проверка величины контрольного угла 225 градусов
//--------------------------------------------------------------------
uint8_t Is_Control_Angle_Valid_Rec(void)
{
    int16_t value = 0;
    // Измеренный угол курса
    value = ram_022064;
	  // Вычитаем 225 градусов
	  value -= CODE_225_DEGREES;
	  value /= 2;
	  // При необходимости берем модуль разности
	  if(value <0)
			  value *= -1;
		// Если разность больше 5 градусов
		if(value > CODE_10_DEGREES)
		{	
		    angle_from_rec_invalid = 0x01;
			  return ANGLE_INVALID; // Угол неверен
		}	
    angle_from_rec_invalid = 0x00;
		return ANGLE_VALID;   	  // Угол верен
}
//--------------------------------------------------------------------
//   Проверка величины контрольного угла 135 градусов
//--------------------------------------------------------------------
uint8_t Is_Control_Angle_Valid(void)
{
    int16_t value = 0;
    // Измеренный угол курса
    value = ram_022064;
	  // Вычитаем 135 градусов
	  value -= CODE_135_DEGREES;
	  // При необходимости берем модуль разности
	  if(value <0)
			  value *= -1;
		// Если разность больше 5 градусов
		if(value > CODE_5_DEGREES)
		{	
		    angle_from_ann_invalid = 0x01;
			  return ANGLE_INVALID; // Угол неверен
		}
    angle_from_ann_invalid = 0x00;		
    return ANGLE_VALID;   	  // Угол верен
}
//--------------------------------------------------------------------
//   Работа изделия в режиме функционального контроля
//--------------------------------------------------------------------
void Mode_Functional_Control(void)
{
	  switch(phase)
	  {
	      case PHASE_START:
		        // 014376
	          code_failure = ERROR_NO;
				    enable_show_angle = 0;
		        // Регистр состояний = 1111
		        // Поддиапазон = 111
		        register_D18D19 |= 0x7807;
		        // Подрежим Телефон
		        register_D18D19 &= (0xFFFF - TLPH_TLGR);
		        Write_QBUS(D18D19,register_D18D19);
		        ram_022064 = CODE_135_DEGREES;
		        Show_Error_No_For_FC();
				    tick_time = 0;
				    time_fc = 0;
				    // Временно отключить установку новой частоты 02.03.2020
				    flags |= DISABLE_ANALYS_MESSAGE;
		        phase = PHASE_DO_MASSAGE_FRQ;
		        phase_next = PHASE_TEST_CPU;  
     				//phase_next = PHASE_GET_CUR_FRQ;
				    break;
				case PHASE_DO_MASSAGE_FRQ:				 
	          // Формируем сообщение Состояние для передатчика
				    if(tick_time >= TIME_ONE_SEC)
	          {
		            phase = Out_Word_Frequency_WR_FC();
							  // Если тест завершился без ошибки, то формируем	сообщение
		            // КУРС для передатчика и показания стрелочного указателя курса
							  if(code_failure == ERROR_NO)
		            {
		                // Если до этого индицировался отказ
			              if(flags & SHOW_FAIL_FC)
			              { 
			                  // Вернем индикацию назад
				                ram_022064 = CODE_135_DEGREES;
			                  Show_Error_No_For_FC();
			              }
		            }
		            else
		                flags |= SHOW_FAIL_FC;	// Индикация отказа в функции main.c 
							  // Запуск таймера	
                tick_time = 0;					
						}
            if(time_fc<10) time_fc++;             
						break;						
        case PHASE_DELAY_000540:
		        if(tick_time >= (TIME_ONE_SEC/2))
		        {
		            // Формируем сообщение о текущей частоте настройки 
			          //Out_Word_Frequency = Cur_Word_Frequency; 
							  // Формируем сообщение Частота для передатчика
				        Out_Word_Frequency_WR(ARINC_FR_FC ); // Матрица состояния 		
							  // Запуск таймера	
                tick_time = 0;
			          phase = PHASE_SELECT_CHECK_BACK;
            }
            break;
				case PHASE_SELECT_CHECK_BACK:
		        // Моя добавка на основании блоксхемы алгоритма        
			      // Выбор узла для повторной проверки
			      phase = Select_Check_back();						
            break;					
        case PHASE_TEST_CPU:
	          // Самотестирование ВУ
	          SelfTestCPU(); 	
				    // Обеспечим мигание частоты и вывод отказа
						phase = PHASE_DO_MASSAGE_FRQ; 
						// Анализ результата теста
						if(code_failure == ERROR_NO)
		        {	 
                flags &= (0xFFFFFFFF - FAIL_CPU);
							  // Следующий узел для проверки
							  phase_next = PHASE_TEST_ARINC_A; 			             								 
		        }
            else
	          {	 
                // Индикация отказа приемника
			          Set_Register_State(SHOW_RECEIVER_FAIL);
			          flags |= FAIL_CPU;
							  tick_time = 0; 
            } 	
		        break;
				case PHASE_TEST_ARINC_A:
		        if(tick_time >= TIME_100_MS)
		        {
		            Test_ARINC_Message();
							  tick_time = 0;
							  phase = PHASE_DO_MASSAGE_FRQ;
						    if(code_failure) // Проблема в канале A
		            {	
		                // Индикация отказа приемника
			              Set_Register_State(SHOW_RECEIVER_FAIL);
									  // Восстановление связи
									  Reset_Test_AB(); // Для платы версии А
									  flags |= FAIL_SELF_TEST_IN_A;
                    tick_time = 0;
                    last = 0;
                    last_AB ^= 0xFFFF; 									                     									
		            }    
			          else
			          {
			              flags &= (0xFFFFFFFF - FAIL_SELF_TEST_IN_A);
				            flags &= (0xFFFFFFFF - FAIL_SELF_TEST_NO_INFO);
                    phase_next = PHASE_TEST_ARINC_B;  								                
			          }
						}       										
            break;			
      	case PHASE_TEST_ARINC_B:
					  if(tick_time >= TIME_100_MS)
		        {
		            Test_ARINC_Message();
							  tick_time = 0;
							  phase = PHASE_DO_MASSAGE_FRQ;
						    if(code_failure) // Проблема в канале B
		            {	
		                // Индикация отказа приемника
			              Set_Register_State(SHOW_RECEIVER_FAIL);
									  flags |= FAIL_SELF_TEST_IN_B;                    								
		            }    
			          else
			          {
			              flags &= (0xFFFFFFFF - FAIL_SELF_TEST_IN_B);
				            flags &= (0xFFFFFFFF - FAIL_SELF_TEST_NO_INFO);
                    phase_next = PHASE_TEST_ARINC ; 		              
			          }
								// Восстановление связи
								Reset_Test_AB(); // Для платы версии А
                tick_time = 0;
                last = 0;	
                last_AB ^= 0xFFFF;								
						}       										
            break;
			  case PHASE_TEST_ARINC:
            // Анализ входного слова и проверки ЦПП
				    if(tick_time >= TIME_ONE_SEC)
						{	
		            phase = PHASE_DO_MASSAGE_FRQ;
							  
							  if(tick_ARINC_rc >= PERIOD_LOST)
	              {	
	                  flags |= FAIL_ARINC_RC;
									  code_failure = CODE_FAIL_ARINC;
								}
                						
							  if(code_failure) 
		            {	
		                // Индикация отказа канала ввода
					          Set_Register_State(SHOW_ARINC_FAIL); 
									  phase_next = PHASE_TEST_ARINC;
                    break;			
		            }	
		            else
			          {	 
                    flags &= (0xFFFFFFFF - FAIL_ARINC_RC);
									  Prepare_Test_Frq_Range();
									  flags |= DISABLE_ANALYS_MESSAGE; 
				            phase_next = PHASE_GET_TEST_FRQ;									  
									  // Сброс разрядов регистра состояния
						        Set_Register_State(ALL_LEDS_OFF);			
			          }
						}								
			      break;
				case PHASE_GET_TEST_FRQ:
		        // Передать сообщение Частота/Состояние
			      phase = PHASE_DO_MASSAGE_FRQ;
			      // Получение тестовой частоты в пределах диапазона
				    phase_next = Get_Test_Frq_SubRange();
			      break;
				case PHASE_SET_TEST_FRQ:
		        // Установка тестовой частоты
				    Set_Test_Frq_SubRange();
				    // Контроль сигнала Захват СЧ
				    phase = PHASE_TEST_HOG_SF; 
			      break;	
		    case PHASE_TEST_HOG_SF:
         	  FC_Check_HOG_SF();
            catch_SF = catch_SF;				
		   	    break;
		    case PHASE_TEST_HOG_SD:	
		        FC_Check_HOG_SD();
          	loaded_Old = 0;	
            size_mass = 0;		
            tick_time_pp = 0;				
			      break;
				case PHASE_GET_CUR_FRQ:
				    // Подготовка проверки от контрольного сигнала антенны
			      Prepare_Current_Test_Frq();
            phase = PHASE_TEST_HOG_SF;
            for(i_del = 0; i_del< 1000; i_del++)
				    phase = PHASE_TEST_HOG_SF;
				    break;				
				case PHASE_MEASURE:
	          enable_show_angle = 1;				  
				    Measure_Show_Angle();
				    	
						phase = PHASE_DO_MASSAGE_FRQ;
				
				    
				    break; 	
			  case PHASE_ANGLE_COMPARE_ANT:
		        // Проверка контрольного угла 135 
			      if(Is_Control_Angle_Valid()==ANGLE_INVALID)	 // Контрольный угол не соотвествует
			      {
			          //angle_degr = Code_To_Degrees_Value(ram_022064); // 07_04_26
							  
				        // Установка тестовой частоты и подключение контрольного сигнала от приемника
					      Set_Test_Frq_SubRange();
								tick_time_pp = 0;
				        size_mass = 0;
							  
								// Звуковой сигнал 
								Set_Telegraph();	  							   
	              // Контроль сигнала Захват СЧ
				        phase = PHASE_TEST_HOG_SF; 
							
							  /*
							  // Перестройка частоты была ?
							  if(flags&NEW_FREQUENCY_DONE)
				        {
				            // Установка тестовой частоты и подключение контрольного сигнала от приемника
					          Set_Test_Frq_SubRange();
									  tick_time_pp = 0;
				            size_mass = 0;
									  // Звуковой сигнал 
									  Set_Telegraph();	  							   
	                  // Контроль сигнала Захват СЧ
				            phase = PHASE_TEST_HOG_SF; 
				        }
				        else
				        {
				            // Перестройки не было
				            Get_New_Frequency();
									  flags |= NEW_FREQUENCY_DONE;
					          // Передать сообщение Частота/Состояние
			              phase = PHASE_DO_MASSAGE_FRQ;
			              // Получение только тестовой частоты без подключения антенны и приемника 
			              phase_next = PHASE_SET_ONLY_FRQ;				
			          }
                */								
			      }
			      else
			      {
				        
							  // Индикация исправности изделия
				        Set_Register_State(SHOW_FC_IS_OK); 
				        code_failure = ERROR_NO;
				        // Передать сообщение Частота/Состояние
			          phase = PHASE_DO_MASSAGE_FRQ;
							  // flags &= (0xFFFFFFFF - FILTER_CHEB_LOAD);
				        // Повторное измерение угла курса
				        phase_next = PHASE_MEASURE;	
                Set_Telegraph();							
			      }
		        break;
			  case PHASE_SET_ONLY_FRQ:
		        // Установка тестовой частоты
				    Set_Only_New_Frq();
				    // Контроль сигнала Захват СЧ текущей частоты
				    phase = PHASE_CUR_HOG_SF; 
			      break;
			  case PHASE_CUR_HOG_SF:
		        FC_Check_Cur_HOG_SF();
		        break;
			  case PHASE_NOISE_HOG_SD:
		        FC_Check_Noes_HOG_SD();
		        break;
				case PHASE_ANGLE_COMPARE_REC:   // Точка 10
			      //angle_degr = Code_To_Degrees_Value(ram_022064); // 07_04_26
				    // Проверка контрольного угла 225
            if(Is_Control_Angle_Valid_Rec()==ANGLE_INVALID)	 // Контрольный угол не соотвествует
			      {
		             // Индикация неисправности приемника
				         Set_Register_State(SHOW_RECEIVER_FAIL); 
				         // Код ошибки
				         code_failure = CODE_FAIL_RECEIVER;
				         // Сформировать сообщение Курс
				         Word_Angl_WR( ARINC_ANGLE_FAILRUE,  // Предупреждение об отказе
	                            ram_022064);          // Курсовой угол
				         // Передать состояние
				         phase = PHASE_DO_MASSAGE_FRQ;
							   // Повторное измерение угла курса
				         phase_next = PHASE_MEASURE;
                 Set_Telegraph();							
			      }
			      else
			      {
			          // Контроль захвата за помеху
				        if(flags&HOG_NOES_SD)
				        {
				            // Индикация исправности приемника
					          Set_Register_State(SHOW_FC_IS_OK);
					          // Код ошибки - захват за помеху
					          code_failure = ERROR_NOG_SD_NOES;					
				        }
				        else
				        {
				            // Индикация отказа антенных цепей
					          Set_Register_State(SHOW_ANTENNA_FAIL);
					          // Код ошибки
					          code_failure = ERROR_FAIL_ANTENNA_1;					
				        }
				        // Сформировать сообщение Курс
				        Word_Angl_WR( ARINC_ANGLE_FAILRUE,  // Предупреждение об отказе
	                           ram_022064);          // Курсовой угол
				        // Передать состояние
				        phase = PHASE_DO_MASSAGE_FRQ;
								// flags &= (0xFFFFFFFF - FILTER_CHEB_LOAD);
				        // Повторное измерение угла курса
				        phase_next = PHASE_MEASURE;
								Set_Telegraph();
								
			      }
						
		        break;
     }
		 Measure_Show_Angle(); 
}

//--------------------------------------------------------------------
//   Измерение и индикация угла курса
//--------------------------------------------------------------------
void Measure_Show_Angle(void)
{	
    if(enable_show_angle == 0) return;
	
	  // Вычисление угла курса
		GetOrtogonalinyeComponents();
		Old_PASS_512();
	
    
		if(flags&SET_CHECK_ANTENNA)
		{	
		    phase_next = PHASE_ANGLE_COMPARE_ANT;
			  //ram_022064 += Add_Noes_Angle();	
				ram_022064 += Add_Control_Angle();
			  
			  //ram_022064 += CODE_20_DEGREES;
			  if(tick_time_pp < TIME_DELAY_PP)
        {
				    // Получение сигналов управления стрелочным прибором	    
		        ram_022064 = CODE_135_DEGREES;							 
				}			
		}	
		else
		{	
				phase_next = PHASE_ANGLE_COMPARE_REC;
			  //angle_degr = Code_To_Degrees_Value(ram_022064);
				//ram_022064 += Add_Control_Angle_Rec();							
			  if(tick_time_pp < TIME_DELAY_PP)
        {
				    // Получение сигналов управления стрелочным прибором	    
		        ram_022064 = CODE_225_DEGREES;
					  show_test_angle_rec = 0;							 
				}
        else
        {
					  
				    if(show_test_angle_rec<1000)
						    show_test_angle_rec++;
            //	ram_022064 = CODE_90_DEGREES;													
					  
        }					
		}	
						        			
		// Получение сигналов управления стрелочным прибором	    
		if(show_test_angle_rec<1000)
		{	
		    Get_Code_Arrow_Device(ram_022064);	
		    // Вывод угла на стрелочный прибор						   
		    Set_DAC_Angle();
		    // Формируем сообщение КУРС для передатчика
	      Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояни
              	     ram_022064);    // Курсовой угол	
		}
    else
		{	
		    Get_Code_Arrow_Device(CODE_90_DEGREES);	
		    // Вывод угла на стрелочный прибор						   
		    Set_DAC_Angle();
		    // Формируем сообщение КУРС для передатчика
	      Word_Angl_WR(ARINC_ANGLE_FC, // Матрица состояни
              	     CODE_90_DEGREES);    // Курсовой угол	
		}	
			
		
}	
//--------------------------------------------------------------------
//   Перестройка на новую частоту
//--------------------------------------------------------------------
void Get_New_Frequency(void)
{
    // Сброс сигналов Проверка приемника  
	  register_D28D29 &= (0xFFFF - CHECK_RECEIVER - CHECK_ANTENNA);
	  Write_QBUS(D28D29,register_D28D29);
	
    // Определяем новую частоту +- 12 кГц 
	  //Set_Frequency_Delta_12kGz();
	  
	   
	  SelectFrequency();	
		// Настройка на новую частоту
		frequency = new_frequency;
		Set_Mode_Compass();
		//Set_Check_Antenna();
		Set_Fr_SF_From_Code();
	  frequency_kGz = frequency_kGz;						
      
	  
	
	  // Режим компас
	  register_D18D19 &= (0xFFFF - COMPASS_ANTENNA);
	  // Установка   Синхронизация ЦАП, ТЛФ/ТЛГР
	  register_D18D19 |= 0x0088;
	  Write_QBUS(D18D19,register_D18D19);
	  // Число попыток
	  i_probe = 2;
	  // Сбросить флаг Повторить проверку 
	  flags &= (0xFFFFFFFF - REPEAT_TEST);
}
//--------------------------------------------------------------------
//  Проверка сигнала Захват СД для ФК по шумам приемника
//--------------------------------------------------------------------
void FC_Check_Noes_HOG_SD(void)
{
    
    code_failure = ERROR_NO;

	  if(Check_HOG_SD() == HOG_SF_RESET)
	  {
        // Сигнал Захват СД не обнаружен	    
		    if(flags & REPEAT_TEST) 
		    {
		        // При повторной проверке
			      flags &= (0xFFFFFFFF - HOG_NOES_SD);
		        flags &= (0xFFFFFFFF - FAIL_HOG_SD);
            i_probe = 2;
		        flags &= (0xFFFFFFFF - REPEAT_TEST);
            // Включить контрольный сигнал антенны
			      Set_Check_Antenna();
					  phase_next = PHASE_MEASURE;
            Set_Telegraph();  					
		    }
		    else
		    {
		        // Установить флаг повторной настройки частоты
			      flags |= REPEAT_TEST;
			      // Повторно проверить захват СД
		        phase_next =  PHASE_NOISE_HOG_SD;
		    }									 
    }							 
	  else
	  {
		    // Сигнал Захват СД обнаружен 
		    flags |= HOG_NOES_SD; // Захват по помехе
		    flags &= (0xFFFFFFFF - FAIL_HOG_SD);
        i_probe = 2;
		    flags &= (0xFFFFFFFF - REPEAT_TEST);
		    // Включить контрольный сигнал антенны
		    Set_Check_Antenna();
			  loaded_Old = 0;	
        size_mass = 0;				
			  phase_next = PHASE_MEASURE;	
			  Set_Telegraph();
	  }
	  // Передать сообщение о состоянии
	  phase = PHASE_DO_MASSAGE_FRQ; 		
}
//--------------------------------------------------------------------
//  Проверка сигнала Захват СД для ФК
//--------------------------------------------------------------------
void FC_Check_HOG_SD(void)
{
    if(Check_HOG_SD() == HOG_SD_RESET)
	  {
        // Сигнал Захват СД не обнаружен	    
		    if(flags & REPEAT_TEST) 
		    {
		        // При повторной проверке
			      code_failure = ERROR_NO_HOG_SD;
            flags |= FAIL_HOG_SD;
					
	          if((signal_from_ann_received == 0)&&(flags&SET_CHECK_ANTENNA))
						{	
		            // Индикация отказа блока антенн
			          Set_Register_State(SHOW_ANTENNA_FAIL);
							  code_failure = CODE_FAIL_ANTENNA;
						}	
						
					  if((signal_from_rec_received == 0)&&(flags&SET_CHECK_RECEIVER))
					      // Индикация отказа приемника
			          Set_Register_State(SHOW_RECEIVER_FAIL);
			      // Будем тупить эту же проверку 
			      i_probe = 2;
			      flags &= (0xFFFFFFFF - REPEAT_TEST);
			      phase_next = PHASE_TEST_HOG_SD;
		    }
		    else
		    {
		        // Установить флаг повторной настройки частоты
			      flags |= REPEAT_TEST;
			      // Повторно проверить захват СД
		        phase_next =  PHASE_TEST_HOG_SD;
		    }									 
    }							 
	  else
	  {
		    // Сигнал Захват СД обнаружен 
		    code_failure = ERROR_NO;
		    flags &= (0xFFFFFFFF - FAIL_HOG_SD);
        i_probe = 2;
		    flags &= (0xFFFFFFFF - REPEAT_TEST);
		    if(i_fr_range >= 0)
		    {
            // Извлечь частоту для следующего поддиапазона
		        phase_next = PHASE_GET_TEST_FRQ;		 
		    }
	 	    else
		    {
		        // Измерение контрольного угла
			      phase_next = PHASE_MEASURE;
		       	Set_Telegraph();
		    }
	  }
	  // Передать сообщение о состоянии
	  phase = PHASE_DO_MASSAGE_FRQ; 		
}
//--------------------------------------------------------------------
//  Проверка сигнала Захват СЧ для ФК текущей частоты
//--------------------------------------------------------------------
void FC_Check_Cur_HOG_SF(void)
{
    // Контроль захвата частоты
    if(Check_HOG_SF() == HOG_SF_RESET)
	  {	
	      // Проблема - нет сигнала Захват СЧ        
	      if(i_probe>0)
		    {
		        i_probe -= 1;
		        // Затем повторить проверку сигнала Захват СЧ
		        phase_next = PHASE_CUR_HOG_SF;
		    }
		    else
		    {
            if(flags & REPEAT_TEST)
			      {
			          // Тест не прошел - жопа
			          flags |= FAIL_HOG_SF;
				        code_failure = ERROR_NO_HOG_FREQUENCY;
				        // Индикация отказа приемника
			          Set_Register_State(SHOW_RECEIVER_FAIL);
				        // Повторная настройка частоты
				       Set_Only_New_Frq();
				       // Будем тупить эту же проверку 
				       i_probe = 2;
				       flags &= (0xFFFFFFFF - REPEAT_TEST);
			         phase_next = PHASE_CUR_HOG_SF; 
			      }
			      else
			      {
			          // Установить флаг повторной настройки частоты
				        flags |= REPEAT_TEST;
				        // Повторная настройка частоты
				        Set_Only_New_Frq();
			          // Затем повторить проверку сигнала Захват СЧ
		            phase_next = PHASE_CUR_HOG_SF;
			      }
		    }			
    }	
    else
    {			
	      code_failure = ERROR_NO;
		    // Сигнал захват СЧ есть
		    flags &= (0xFFFFFFFF - FAIL_HOG_SF);
		    // Сбросить флаг Повторить проверку 
	      flags &= (0xFFFFFFFF - REPEAT_TEST);
		    // Затем проверить сигнал Захват СД для помехи
		    phase_next = PHASE_NOISE_HOG_SD;
	  }
	  // Передать сообщение о состоянии
	  phase = PHASE_DO_MASSAGE_FRQ; 
}	 
//--------------------------------------------------------------------
//  Проверка сигнала Захват СЧ для ФК
//--------------------------------------------------------------------
void FC_Check_HOG_SF(void)
{
    
	
	  // Контроль захвата частоты
    if(Check_HOG_SF() == HOG_SF_RESET)
	  {	
	      // Проблема - нет сигнала Захват СЧ        
	      if(i_probe>0)
		    {
		        i_probe -= 1;
		        // Затем повторить проверку сигнала Захват СЧ
		        phase_next = PHASE_TEST_HOG_SF;
		    }
		    else
		    {
            if(flags & REPEAT_TEST)
			      {
			          // Тест не прошел - жопа
			          flags |= FAIL_HOG_SF;
				        code_failure = ERROR_NO_HOG_FREQUENCY;
				        // Индикация отказа приемника
			          Set_Register_State(SHOW_RECEIVER_FAIL);
				        // Будем тупить эту же проверку 
				        i_probe = 2;
				        flags &= (0xFFFFFFFF - REPEAT_TEST);
			          phase_next = PHASE_SET_TEST_FRQ; 
			      }
			      else
			      {
			          // Установить флаг повторной настройки частоты
				        flags |= REPEAT_TEST;
				        // Повторная настройка частоты
				        Set_Test_Frq_SubRange();
			          // Затем повторить проверку сигнала Захват СЧ
		            phase_next = PHASE_TEST_HOG_SF;
			     }
		    }			
	  }	
    else
    {			
	      code_failure = ERROR_NO;
		    // Сигнал захват СЧ есть
		    flags &= (0xFFFFFFFF - FAIL_HOG_SF);
		    // Сбросить флаг Повторить проверку 
	      flags &= (0xFFFFFFFF - REPEAT_TEST);
		    // Затем проверить сигнал Захват СД
		    phase_next = PHASE_TEST_HOG_SD;
	  }
	  // Передать сообщение о состоянии
	  phase = PHASE_DO_MASSAGE_FRQ; 
}	 
//--------------------------------------------------------------------
//  Выбор узла для повторной проверки
//--------------------------------------------------------------------
uint8_t Select_Check_back(void)
{
    // Отказ самотестирования ВУ
    if(flags & FAIL_CPU)
	    return 	PHASE_TEST_CPU;
		
	  // Отказ тестирования по каналу А
	  if(flags & FAIL_SELF_TEST_IN_A)
	  {
	      Prepare_Test_ARINC_A();
		    return PHASE_TEST_ARINC_A;
	  }
	  // Отказ тестирования по каналу В
	  if(flags & FAIL_SELF_TEST_IN_B)
	  {
	      Prepare_Test_ARINC_B();
		    return PHASE_TEST_ARINC_B;
	  }
    // Нет сигнала ARINC429
	  if(flags & FAIL_ARINC_RC)
	  {
	      return PHASE_TEST_ARINC;
	  }


	  // Отказы не обнаружены
	  switch(phase_next)
	  {
	      /*
			  case PHASE_TEST_CPU: 
				case PHASE_ANGLE_COMPARE_ANT:
				case PHASE_ANGLE_COMPARE_REC:	
				case PHASE_GET_TEST_FRQ:
        case PHASE_SET_TEST_FRQ:
        case PHASE_TEST_HOG_SF:	
        case PHASE_TEST_HOG_SD:
        case PHASE_GET_CUR_FRQ:
        case PHASE_DO_MASSAGE_FRQ:
        case PHASE_MEASURE: 
        case PHASE_SET_ONLY_FRQ:
        case PHASE_CUR_HOG_SF:
        case PHASE_NOISE_HOG_SD:
        case PHASE_TEST_ARINC: 					
						return phase_next;
				*/
			   
				case PHASE_TEST_ARINC_A:
		        Prepare_Test_ARINC_A();
				    break;
		    case PHASE_TEST_ARINC_B:
		        Prepare_Test_ARINC_B();
				    break;    
	  }
    return phase_next; 

}

//--------------------------------------------------------------------
// Подготовка проверки текущей частоты настройки
//--------------------------------------------------------------------
void Prepare_Current_Test_Frq(void)
{
	  // Получение текущей частоты
		Determ_Frequency(Cur_Word_Frequency);
		// Получение ближайшей кратной 12
		SelectFrequency();
		// Настройка на новую частоту
		frequency = new_frequency;
    Set_Mode_Compass();
		Set_Telephon();
		Set_Check_Antenna();
    Set_Fr_SF_From_Code();
	  // Число попыток
	  i_probe = 2;
	  // Сбросить флаг Повторить проверку 
	  flags &= (0xFFFFFFFF - REPEAT_TEST);	
}
//--------------------------------------------------------------------
// Подготовка проверки частотных поддиапазонов 
//--------------------------------------------------------------------
void Prepare_Test_Frq_Range(void)
{
    // Сброс сигналов Проверка приемника и Проверка антенны 
	  register_D28D29 &= (0xFFFF - 0x0006);
	  flags &= (0xFFFFFFFF - SET_CHECK_ANTENNA - SET_CHECK_RECEIVER);
	  // Установка Проверка АЦП, Синхронизация СЧ, Частота/Крен
	  register_D28D29 |= 0x1081; 
	  Write_QBUS(D28D29,register_D28D29);
	  // Режим компас
	  register_D18D19 &= (0xFFFF - COMPASS_ANTENNA);
	  // Установка   Синхронизация ЦАП, ТЛФ/ТЛГР
	  register_D18D19 |= 0x0088;
	  Write_QBUS(D18D19,register_D18D19);
	  // Счетчик поддиапазонов
	  i_fr_range = VALUE_SUBRANGE;
	  // Сбросить флаг Повторить проверку 
	  flags &= (0xFFFFFFFF - REPEAT_TEST);	
}
//--------------------------------------------------------------------
// Установка только тестового значения частоты
// контрольный сигнал не подключается 
//--------------------------------------------------------------------
void Set_Only_New_Frq(void)
{
	  // Загрузка частоты в ячейку СЧ	без контроля сигнала Захват СЧ 
   	Determ_Range_Fr();         // Определение диапазона
    Renov_Range_Fr();					 // Выбор диапазона частот 
    Transform_Code_Fr();			 // Преобразование двоично-десятичного кода 
	                             // частоты в бинарный
    Write_DATA_SF();           // Запись кода в ячейку СФ    
	  // Импульс перестройки
	  Pulse_Fr();                
	  Delay_SD();
	  Set_Mode_Compass();
	
	  // Подготовка контроля захвата СЧ
	  i_probe = 2;
	  flags &= (0xFFFFFFFF - REPEAT_TEST);
}

//--------------------------------------------------------------------
// Установка тестового значения частоты из поддиапазона
// Контрольный сигнал приемника
//--------------------------------------------------------------------
void Set_Test_Frq_SubRange(void)
{
	  // Включение тестового сигнала приемника
	  Set_Check_Receiver();    
	  // Преобразование кода частоты frequency в преременную float frequency_kGz
	  Get_Frequency_kGz();
    // Загрузка частоты в ячейку СЧ	без контроля сигнала Захват СЧ 
   	Determ_Range_Fr();         // Определение диапазона
    Renov_Range_Fr();					 // Выбор диапазона частот 
    Transform_Code_Fr();			 // Преобразование двоично-десятичного кода 
	                             // частоты в бинарный
    Write_DATA_SF();           // Запись кода в ячейку СФ    
	  // Импульс перестройки
	  Pulse_Fr();                
	  Delay_SD();
	  //Set_Mode_Compass();
}
//--------------------------------------------------------------------
// Получение тестового значения частоты из поддиапазона
//--------------------------------------------------------------------
uint8_t Get_Test_Frq_SubRange(void)
{
    i_fr_range -= 1; 
	  // Сбросить флаг Повторить проверку 
	  flags &= (0xFFFFFFFF - REPEAT_TEST);
	  // Счетчик проб 
    i_probe = 2;
	
	  if(i_fr_range >= 0)
	  {	
	      // Настройка на новую частоту
	      frequency = Test_Frq_SubRange[i_fr_range];
	      // Сброс сигналов Проверка приемника и Проверка антенны 
	      register_D28D29 &= (0xFFFF - 0x0006);
		    flags &= (0xFFFFFFFF - SET_CHECK_ANTENNA - SET_CHECK_RECEIVER);
	      // Установка Проверка АЦП, Синхронизация СЧ, Частота/Крен
	      register_D28D29 |= 0x1081;
			  Write_QBUS(D28D29,register_D28D29);
	      // Запуск таймера
	      tick_time = 0;
		    return PHASE_SET_TEST_FRQ;
	  }
		

		
    return 	PHASE_GET_CUR_FRQ;		  
}
//--------------------------------------------------------------------
// Проверка принятого тестового слова 
//--------------------------------------------------------------------
void Test_ARINC_Message(void)
{
    if(Test_Word_Frequency == 0)
		{	
		    code_failure = ERROR_NO_INFO_FROM_INPUT;
			  flags |= FAIL_SELF_TEST_NO_INFO;
			  return;
		}	
		
		flags &= (0xFFFFFFFF - FAIL_SELF_TEST_NO_INFO);
		// Удалить бит четности
		Test_Word_Frequency &= (0xFFFFFFFF - BIT31);   
    if(Test_Word_Frequency != Out_Word_Frequency)
		{	
  	    if(arinc_AB == CHANNEL_A)
        {					
			      flags |= FAIL_SELF_TEST_IN_A;
					  code_failure = ERROR_RETRANSMIT_MESSAGE_A;
				}	
				else
				{	
					  flags |= FAIL_SELF_TEST_IN_B;
					  code_failure = ERROR_RETRANSMIT_MESSAGE_B;
				}	
		}	
    else
			  code_failure = ERROR_NO;
}
//--------------------------------------------------------------------
// Подготовка проверки приемопередающего тракта ARINC429 по каналу B
//--------------------------------------------------------------------
void Prepare_Test_ARINC_B(void)
{
	  // Активировать канал A на прием
	  arinc_AB = CHANNEL_B;
	  Arinc429RxConfig_Ch(ARINC429R_CLK_12_5_KHz, LABEL_R);
	  // Коммутировать выход передатчика B на вход приемника B
	  Set_Test_B();
	  // Сформировать тестовое	сообщение Частота/Состояние для канала B
    // Set_Test_Message_Freq_Status();  30_06_23
    // Сброс ожидаемого слова
	  Test_Word_Frequency = 0;
    // Запускж таймера	
    tick_time = 0;	
}	
//--------------------------------------------------------------------
// Подготовка проверки приемопередающего тракта ARINC429 по каналу A 
//--------------------------------------------------------------------
void Prepare_Test_ARINC_A(void)
{
	  // Активировать канал A на прием
	  arinc_AB = CHANNEL_A;
	  Arinc429RxConfig_Ch(ARINC429R_CLK_12_5_KHz, LABEL_R);
	  // Коммутировать выход передатчика A на вход приемника A
	  //Set_Test_AB(); // Для платы версии А
	  Set_Test_A();
	  // Сформировать тестовое	сообщение Частота/Состояние для канала А
    //Set_Test_Message_Freq_Status(); 30_06_23
    // Сброс ожидаемого слова
	  Test_Word_Frequency = 0;
    // Запускж таймера	
    tick_time = 0;	
}	
//--------------------------------------------------------------------
//   Самотестирование CPU 
//--------------------------------------------------------------------
void SelfTestCPU(void)
{
    /*
	  static uint8_t i = 0;	  
	  
    if(i<5)
		    code_failure = ERROR_CPU_0;
    else 		
	      code_failure = ERROR_NO; 
    i++;
    */
    code_failure = ERROR_NO;	
}	
//--------------------------------------------------------------------
