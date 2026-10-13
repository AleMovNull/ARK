#include "APK25_SF.h"
#include "APK25_FC.h"
#include "APK25_Chassis.h"
#include "APK25_QBUS.h"
#include "Init_APK25.h"
#include "APK25_Angle.h"
#include "APK25_TransRec.h"
//--------------------------------------------------------------------

extern uint16_t register_D18D19; //enable
extern uint16_t register_D28D29; // enable
extern uint32_t flags;
extern uint8_t  phase;               // Фаза выполнения ФК
extern uint16_t load_f;              // Загрузка ФНЧ оценок угла
extern uint16_t tick_time;
extern uint16_t last;                // Состоянии переферии
extern uint16_t code_failure;        // Код ошибки

extern _REGISTR R0, R1, R2, R3, R4, R5;
uint16_t new_frequency;              // Новый код частоты в двоично-десятичном коде
uint16_t frequency;	                 // Код частоты в двоично-десятичном коде
uint16_t input_frequency;            // Входная частота от ЦВС
float    frequency_kGz;              // Частота в кГц 

uint32_t code_frequency;			       // Код частоты в 24 битном двоичном коде для записи в СЧ
uint8_t  range_frequency;            // Диапазон частот
uint8_t  state_working;              // Режим работы АРК-25

// новые переменные, предотвращают перестройку частоты при смене режима АРК
uint16_t frequency_0 = 1;	           // Код частоты первое значение
uint16_t frequency_00 = 0;	         // Код частоты последующее значение

uint32_t Out_Word_Frequency = 0;  // Выходное слово Частота/Состояние
uint32_t Cur_Word_Frequency = 0;  // Текущее  слово Частота


uint32_t Test_Word_Frequency = 0; // Входное тестовое слово Частота/Состояние
extern uint32_t Out_Word_Angl;    // Выходное слово КУРС
uint32_t In_Word_ARINC_0 = 0;     // Предыдущее значение сообщения "Частота/Функция"
extern uint8_t freq_fail;
extern float fr_kHz_real;

uint8_t catch_SF = 0;
uint8_t catch_SD = 0;
uint8_t signal_from_rec_received = 0;
uint8_t signal_from_ann_received = 0;

// Обработка сообщения "Частота/Функция"
void Process_In_Word_ARINC(uint32_t In_Word_ARINC_)
{	
     	
    if(flags & ENABLE_FUNC_CONTR) 
	  {	
	      // Блокировка анализа в фазах ФК
		    if((phase == PHASE_TEST_ARINC_A)||
		       (phase == PHASE_TEST_ARINC_B))
		    {
		       // Запомним принятое сообщение
			     Test_Word_Frequency = In_Word_ARINC_; 
			     return;
		    }
           				
    }

		// Блокировка отработки установки частоты при проверке поддиапазонов
    if(flags & DISABLE_ANALYS_MESSAGE)
	    return; 
	
    // Копируем входное слово 
	  Cur_Word_Frequency = In_Word_ARINC_;
				
	  // Если один из параметров входного слова поменялся, производим обработку слова
	  if(In_Word_ARINC_ != In_Word_ARINC_0)
	  {  
	      flags = 0;
		    load_f = 0;
        tick_time = 0;			
			
		   if(frequency_0 != frequency_00)
			    {
			      // Запись частоты в ячейку СЧ
               Set_Fr_SF(In_Word_ARINC_);
				 
			         if(range_frequency == 0x07) 
			            return;
						               
					     // Режим Антенна
		           Set_Mode_Ant();			     				 
		           // Импульс перестройки
		           Pulse_Fr();                
	             Delay_SD();
               // Контроль захвата частоты
		           if(Check_HOG_SF() == HOG_SF_RESET)
		               flags |= FAIL_HOG_SF;
               else				 
		               flags &= (0xFFFFFFFF - FAIL_HOG_SF);
				
				       flags |= NEW_FREQUENCY_DONE;
						 
						  // if(state_working != CONTROL)
							//		 frequency_00 = frequency_0;
			  	  }						 
         				 
		    // Отработка режима работы АРК
		    switch(state_working)
	      {
	          case COMPASS:
			          Set_Mode_Compass();
			          register_D28D29 = 0x0081; // 1
	              Write_QBUS(D28D29,register_D28D29);
		            // Копируем входное слово  в выходное
	              Out_Word_Frequency = In_Word_ARINC_; 
	              Delay_SD();		   
	              break;
		       case ANTENNA_TLF:
						 
		       case ANTENNA_TLG: 
						 
						    register_D28D29 = 0x0081; // 1
	              Write_QBUS(D28D29,register_D28D29);
					 				 
						   tick_time = 0;
						   // Копируем входное слово  в выходное
					     Out_Word_Frequency = In_Word_ARINC_;
		           // Установить режим антенна
			         Set_Mode_Ant();
			         Delay_SD();
					     break; 		
					 case CONTROL: 
					     break; 
					 
	      }
		   // In_Word_ARINC_0 = In_Word_ARINC_;
        
       // if((state_working != CONTROL)|| (state_working != FUNC_CONTROL))
				
				/*
				if(state_working != CONTROL)
				    {			
				     frequency_00 = frequency_0;
				     In_Word_ARINC_0 = In_Word_ARINC_;
						}
        else 
				    { 
							frequency_00 = 0;
							In_Word_ARINC_0 = 0;
						}
       */ 
						
				switch(state_working)
    {
	      case FUNC_CONTROL:
			                frequency_00 = 0;
							        In_Word_ARINC_0 = 0;
			      break;
		    
        case CONTROL:
					            frequency_00 = 0;
							        In_Word_ARINC_0 = 0;
            break;					
				default:
					            frequency_00 = frequency_0;
				              In_Word_ARINC_0 = In_Word_ARINC_;
				    break;
			}












						
						
	  }
}

//--------------------------------------------------------------------
// Настроить на новую частоту отличную от предыдущей на 12 кГц
// в пределах диапазона. На выходе новая частота настройки в
// двоично-десятичном коде
//--------------------------------------------------------------------
void Set_Frequency_Delta_12kGz(void)
{
    uint8_t  old_range_frequency = range_frequency;	  
	  uint16_t fr_int = 0;
	
	  // Получение значения частоты в кГц
	  Get_Frequency_kGz();
	  // Добавляем 12 кГц к текущей частоте
    frequency_kGz += 12.0;
	  // Контроль выхода за пределы диапазона
	  if(frequency_kGz > 1750.0)  
	      frequency_kGz -= 24.0;
    // Проверяем кратность 12
    fr_int = frequency_kGz/12;
    frequency_kGz = fr_int * 12.0; 	
	  // Получаем двоично-десятичный код новой частоты
	  Get_Binary_Decimal_Code_Fr();
	  // Определяем новый диапазон
	  Determ_Range_Fr();
	  if(old_range_frequency == range_frequency)
	      return;
    // Добавляем 12 кГц к текущей частоте
    frequency_kGz -= 12.0;
    // Контроль выхода за пределы диапазона
	  if(frequency_kGz < 150.0)
   	    frequency_kGz += 24.0;
		// Проверяем кратность 12
    fr_int = frequency_kGz/12;
    frequency_kGz = fr_int * 12.0; 	
	  // Получаем двоично-десятичный код новой частоты
	  Get_Binary_Decimal_Code_Fr();
	  // Определяем новый диапазон
	  Determ_Range_Fr();	 	
}	
//--------------------------------------------------------------------
// Получение двоично-десятичного кода частоты 
//--------------------------------------------------------------------
void Get_Binary_Decimal_Code_Fr(void)
{
    uint8_t tetrad = 0;
	  float fr = frequency_kGz;
	
	  // Единицы МГц
	  tetrad = fr/1000.0;
	  frequency = tetrad;
 	  frequency <<= 4;
	  fr -= (tetrad*1000.0);
	  // Сотни кГц
	  tetrad = fr/100.0;
	  frequency += tetrad;
 	  frequency <<= 4;
	  fr -= (tetrad*100.0);
	  // Десятки кГц
	  tetrad = fr/10.0;
	  frequency += tetrad;
 	  frequency <<= 4;
	  fr -= (tetrad*10.0);
	  // Единицы кГц
	  tetrad = fr;
	  frequency += tetrad;
 	  frequency <<= 1;
	  fr -= tetrad;
		// 0,5 кГц
		if(fr>= 0.5)
		    frequency |= BIT0;		
}	

//--------------------------------------------------------------------
// Получение значения частоты в кГц
//--------------------------------------------------------------------
void Get_Frequency_kGz(void)
{
    uint16_t code_fr = frequency;
	  frequency_kGz = 0.0;
	  if(code_fr & BIT0)
			frequency_kGz = 0.5;
	  code_fr >>= 1;
		// ЕдиницымкГц
		frequency_kGz += (code_fr&0x0F); 
		code_fr >>= 4;
		// Десятки кГц
		frequency_kGz += (code_fr&0x0F)*10;
		code_fr >>= 4;
		// Сотни кГц
		frequency_kGz += (code_fr&0x0F)*100;
		code_fr >>= 4;
		// Единицы МГц
		frequency_kGz += (code_fr&0x07)*1000;
}	
//--------------------------------------------------------------------
//     Контроль захвата частоты
//--------------------------------------------------------------------
uint8_t Check_HOG_SF(void)
{
    uint8_t i = 0;
	   
	  catch_SF = 0x00;
	  for(i = 0; i < 10; i++)
	  {
	      Delay_SD();
		    if(Check_SF() == HOG_SF_SET)
		    {	
		        catch_SF = 0x01;
			      return HOG_SF_SET;
				}	
		}	
		return HOG_SF_RESET; 
}	
//--------------------------------------------------------------------
//     Контроль захвата частоты
//--------------------------------------------------------------------
uint8_t Check_HOG_SD(void)
{
    uint8_t i = 0;
	
	  catch_SD = 0;	
	  for(i = 0; i < 10; i++)
	  {
			  Delay_SD();
		    if(Check_SD() == HOG_SD_SET)
				{	
				    catch_SD = 1;
            if(flags&SET_CHECK_RECEIVER) signal_from_rec_received = 0x01;
					  if(flags&SET_CHECK_ANTENNA)  signal_from_ann_received = 0x01;
					  return HOG_SD_SET;
				}	
		}
		if(flags&SET_CHECK_RECEIVER) signal_from_rec_received = 0x00;
		if(flags&SET_CHECK_ANTENNA)  signal_from_ann_received = 0x00;
	  return HOG_SD_RESET; 
}


//--------------------------------------------------------------------
//     Определение кода частоты
//--------------------------------------------------------------------
void Determ_Frequency(uint32_t In_Word_Frequency_)
{
  	In_Word_Frequency_ &= 0x1FFFE000;
	  In_Word_Frequency_ >>= 13;  
	  frequency = In_Word_Frequency_; 
    input_frequency = frequency;	  
}

//--------------------------------------------------------------------
//     Определение кода частоты на входе
//--------------------------------------------------------------------
void Determ_Frequency_Input(uint32_t In_Word_Frequency_)
{
  	In_Word_Frequency_ &= 0x1FFFE000;
	  In_Word_Frequency_ >>= 13;  
	  input_frequency = In_Word_Frequency_;
    frequency_0 = input_frequency; // новое		
}


//--------------------------------------------------------------------
//     Определение диапазона частоты
//--------------------------------------------------------------------
void Determ_Range_Fr(void)
{
   	/*
	  static const uint16_t Levels_Range_Fr[6] =	  
    {
        0x02A0, 	  // 150  kHz
	      0x0460, 	  // 230  kHz
	      0x06C0,     // 360  kHz
        0x0AC0,     // 560  kHz
	      0x10C0,     // 860  kHz
	      0x2600      // 1300 kHz		
    };
    */	
	
	  range_frequency = 0x7; // range no determin
	  if(frequency >= F_150_kHz) range_frequency = 0;
	  if(frequency >= F_230_kHz) range_frequency = 1;
	  if(frequency >= F_360_kHz) range_frequency = 2;
	  if(frequency >= F_560_kHz) range_frequency = 3;
	  if(frequency >= F_860_kHz) range_frequency = 4;
	  //if((frequency >= F_1300_kHz)&&(frequency <=F_1750_kHz)) 07_04_26
	  if(frequency >= F_1300_kHz)	range_frequency = 5;
}
//--------------------------------------------------------------------
//     Выбор диапазона частот
//-------------------------------------------------------------------- 
void Renov_Range_Fr(void)
{
    register_D18D19 &= MASK_RANGE_FR;
    register_D18D19 |= range_frequency;
    Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
// Преобразование двоично-десятичного кода в 24 битный код частоты
//--------------------------------------------------------------------
void Transform_Code_Fr(void)
{
	  uint32_t transform = 0;
    code_frequency = 0x200000;

    // 0,5 kHz
    if(frequency&BIT0)  code_frequency |= BIT4;

	// amount 1 kHz
	 transform = (frequency>>1)&0xF;
	 if(transform >= 5)
		code_frequency = code_frequency | ((transform - 5)<<5) | BIT8;
	 else 	 
		code_frequency |= (transform << 5);

	 // amount 10 kHz
	 transform = 0;
	 transform = (frequency>>5)&0xF;
	 if(transform >= 5)
	    code_frequency = code_frequency | ((transform - 5)<< 9) | BIT12;
	 else 	 
		code_frequency |= (transform << 9);

	 // amount 100 kHz
	 transform = 0;
	 transform = (frequency>>9)&0xF;
	 if(transform >= 5)
        code_frequency = code_frequency | ((transform - 5)<< 13) | BIT16;
	 else 	 
		code_frequency |= (transform << 13);

	 /*
	 // amount 1 MHz
	 if(frequency&BIT13)  code_frequency |= BIT20;
	 else  code_frequency |= BIT19;
	 */
	 
	 // к-во MHz
	 if((frequency&BIT13)||(frequency&BIT14)) code_frequency |= BIT20; // к-во MHz = 1 или 2 
	 else code_frequency |= BIT19; // к-во MHz = 0
	 if(frequency&BIT14) code_frequency |= BIT17; // к-во MHz = 2 
	 //(2.182)
}
//--------------------------------------------------------------------
//     Импульс синхронизации СФ 
//--------------------------------------------------------------------
void Pulse_SYN_SF(void)
{
    Clear_SYN_SF();
    Set_SYN_SF();   
} 
//--------------------------------------------------------------------
//     Запись кода в ячейку СФ
//--------------------------------------------------------------------
 void Write_DATA_SF(void)
{
    uint8_t i = 0;
    uint32_t data_code = 0;
   
    data_code = code_frequency;
      
    for(i=0; i<24; i++)
    { 
	      Clear_CLOCK_SF();
	      if(data_code&1) 
					  Set_DATA_SF();
	   	  else 
					  Clear_DATA_SF(); 
        Set_CLOCK_SF();
		    Clear_CLOCK_SF();
        Clear_DATA_SF();
		    data_code >>= 1;
	  }
	  Pulse_SYN_SF();  
}
//--------------------------------------------------------------------
//     Установка частоты ячейки СЧ
//--------------------------------------------------------------------
void Set_Fr_SF(uint32_t In_Word_Frequency)
{
    Determ_Frequency(In_Word_Frequency); // Выделение кода частоты	 			
    Determ_Range_Fr();                   // Определение диапазона
	  if(range_frequency == 0x07) 
			return;
    Renov_Range_Fr();					           // Выбор диапазона частот 
    Transform_Code_Fr();				         // Преобразование двоично-десятичного кода 
	                                       // частоты в бинарный
	  Get_Frequency_kGz(); 
	
    Write_DATA_SF();                     // Запись кода в ячейку СФ     		
}
//--------------------------------------------------------------------
//     Установка частоты ячейки СЧ из двоично-десятичного кода
//--------------------------------------------------------------------
void Set_Fr_SF_From_Code(void)
{
    Determ_Range_Fr();         // Определение диапазона
    Renov_Range_Fr();					 // Выбор диапазона частот 
    Transform_Code_Fr();			 // Преобразование двоично-десятичного кода 
	                             // частоты в бинарный
	  Get_Frequency_kGz(); 
    Write_DATA_SF();           // Запись кода в ячейку СФ    
	  // Импульс перестройки
	  Pulse_Fr();                
	  Delay_SD();
	  // Контроль захвата частоты
	  if(Check_HOG_SF() == HOG_SF_RESET)
	  {	
	      flags |= FAIL_HOG_SF;
	  }	
    else
    {			
	      flags &= (0xFFFFFFFF - FAIL_HOG_SF);
	  }	
}
//--------------------------------------------------------------------
//     Выбор частоты ячейки СЧ
//  Вход - R0, содержит полный двоично-десятичный код частоты
//  Выход - new_frequency содержит новый двоично-десятичный код частоты
//--------------------------------------------------------------------
void SelectFrequency(void)
{
    uint8_t  _byte = 0;
	  static const uint16_t t_freq_0[7] =  
	  {
		   0x2EA0,     // 1750
		   0x2600,     // 1300
		   0x10C0,     // 860
		   0x0AC0,     // 560
		   0x06C0,     // 360
		   0x0460,     // 230
		   0x02A0      // 150
	  };
	  static const uint16_t t_freq_1[7] =
	 {
		 0x2E80,     // 1740
		 0x2508,     // 1284
		 0x1080,     // 840
		 0x0A80,     // 540
		 0x0690,     // 348
		 0x042C,     // 216
		 0x02AC      // 156
	};

  if(fr_kHz_real<150.0) 
	    frequency = F_150_kHz;
	else
	{ 
      if(fr_kHz_real>1750.0)
      {				
          //frequency = F_1750_kHz;				
				  new_frequency = 0x2E80;
				  return;
			}	
	}	
	
	
	R0._uword = frequency;
	R0._uword &= (0xFFFF - 0xC000);

	R2._uword = R0._uword;
	R2._uword &= (0xFFFF - 0xC1FF);

	R1._uword = R0._uword;
	R1._uword &= (0xFFFF - 0xE1FF);
	
	_byte = (R1._uword>>8)&0xFF;
	R1._uword <<= 8;
	R1._uword  += _byte;
	R1._uword  >>= 1;

	if(R0._uword & 0x2000)
		R1._uword += 10; 

	while(R1._uword > 3) R1._uword -= 3;

	
	if(R1._uword == 1)  
		R2._uword |= 0x0010;   
	else
	{
		if(R1._uword == 2) 
			R2._uword |= 0x0008;  
	}

m_014172:
	{
		  R1._uword = R2._uword;
		  R1._word -= R0._word;

		  if(R1._word > 0)
		  {
			  if(R1._word >= 32)
				  goto m_014310;
		  }

		  R2._uword += 36;  

		  if((R2._uword&0x0010)==0)
			  goto m_014236;
		  if((R2._uword&0x0004)==0)
			  goto m_014236;
		  R2._uword &= (0xFFFF - 0x001C);
		  R2._uword += 32;

m_014236:
		  if((R2._uword & 0x0100)==0)
			 goto m_014172;
		  if((R2._uword & 0x0040)==0)
			 goto m_014172;

		  R2._uword &= (0xFFFF - 0x01C0);
		  R2._word += 0x0200;

		  if((R2._uword & 0x1000)==0)
			  goto m_014172;
		  if((R2._uword & 0x0400)==0)
			  goto m_014172;

		  R2._uword &= (0xFFFF - 0x1C00);
		  R2._word += 0x2000;
		  goto m_014172;
	}

m_014310:
		 if(R0._uword >= 0x2EA1)
		 {
			new_frequency = 0x2E80;
			return;
		 }

//m_014330:

		_byte = 0;
		 do
		 {
			 if(R2._uword >= t_freq_0[_byte])
			 {
				 _byte++;
				 goto m_014342;
			 }
			_byte++;
		 }
		 while(_byte < 7);

m_014342:
		 if(R0._uword >= t_freq_0[_byte-1])
			new_frequency = R2._uword;
	else
			new_frequency = t_freq_1[_byte-1];

}




























