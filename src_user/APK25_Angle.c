#include "Init_APK25.h"
#include "APK25_Angle.h"
#include "MDR32F9Qx_port.h"
#include "APK25_SF.h"
#include "APK25_Chassis.h"
#include "APK25_Filtr.h"
#include "APK25_FC.h"
#include "APK25_TransRec.h"
#include "math.h"
#include "MDR32F9Qx_uart.h"

//--------------------------------------------------------------------

void Eight_periods_to_half_period(int16_t* dac); // Procedure_005174()
void Sin_Cos_Comp(int16_t* dac);

void ProcessOrtogonalinyeComponents(void);       // Procedure_004610()


void Procedure_004110(void);
void Procedure_005122(void);
void Procedure_005460(void);
void Correct_Quadrant_Mistake(uint16_t correct_quadrant_mistake); 
void Set_BDPRS(int16_t angle); 

int16_t Check_Posit_Antenna(int16_t angle);
void Set_Code_Arrow_Device(void);
void Mode_Antenna(uint8_t mode);

void Code_To_Degrees(void);
//float Code_To_Degrees_Value(uint16_t value);
int16_t my_abs(int16_t value);
void Degrees_To_Code(void);
uint16_t Degrees_To_Code_Value(float value);




//--------------------------------------------------------------------
extern uint8_t state_working;       // Режим работы АРК-25
extern DATA switch_sin_0;
extern DATA switch_sin_1;

extern uint8_t enable_angle;
extern uint16_t ID;
extern uint32_t flags;
extern uint16_t tick_time;
extern uint16_t load_f;             // Загрузка ФНЧ оценок угла
extern uint16_t load_fv;            // Загрузка ФНЧ оценок угла
extern uint16_t code_angle_180;     // Код угла с поворотом на 180 градусов 
extern uint16_t code_angle_0;       // Код угла с поворотом на 0   градусов
extern uint8_t  en_trans;
uint32_t code_DAC = 0; 
extern float    frequency_kGz;              // Частота в кГц

// Calculation of the heading angle
_REGISTR R0, R1, R2, R3, R4, R5;
_REGISTR  D0, D1;

_TWO_WORD component_cos, component_sin;
_TWO_WORD _cos, _sin;
_TWO_WORD R0R1, R2R3, R4R5;
int16_t half_period[16];


uint16_t
	ram_022060,  
	ram_022064;  // Содержит код угла курса
	
int16_t mass1[64];
int16_t mass2[32];
//int16_t mass3[32];
int16_t correct_quadrant_mistake;
int16_t antenna_h;
uint8_t loaded;
uint32_t code_angle;

//AM angle_mass;


//int delta_ch_0 = 0;
//int delta_ch_180 = 0;

ANALYS_ANGLE analys;



uint32_t Out_Word_Angl; // enable
extern uint32_t In_Word_ARINC; // enable
extern uint16_t size_mass;    // Размер прочитанного массива
extern uint16_t tick_hog_sd; // Счетчик временных интервалов захвата СД

float angle_degrees = 0.0;
float angle_degrees_fltr = 0.0;
uint8_t flags_angle = 0;

uint8_t chenge  = 0;



//--------------------------------------------------------------------

static const uint16_t tabl[257] =
{
	0x0000, 0x0005, 0x000A, 0x000F, 0x0014, 0x0019, 0x001F, 0x0024,
	0x0029, 0x002E, 0x0033, 0x0038, 0x003D, 0x0042, 0x0047, 0x004C,	
	0x0051, 0x0056, 0x005C, 0x0061, 0x0066, 0x006B, 0x0070, 0x0075,
	0x007A, 0x007F, 0x0084, 0x0089, 0x008E, 0x0093, 0x0098, 0x009D,
	0x00A2, 0x00A7, 0x00AC, 0x00B1, 0x00B6, 0x00BB, 0x00C0, 0x00C5,	
	0x00CA, 0x00CF, 0x00D4, 0x00D9, 0x00DE, 0x00E3, 0x00E8, 0x00ED,
	0x00F2, 0x00F7, 0x00FB, 0x0100, 0x0105, 0x010A, 0x010F, 0x0114,
	0x0119, 0x011E, 0x0122, 0x0127, 0x012C, 0x0131, 0x0136, 0x013B,
	0x013F, 0x0144, 0x0149, 0x014E, 0x0153, 0x0157, 0x015C, 0x0161,
	0x0165, 0x016A, 0x016F, 0x0174, 0x0178, 0x017D, 0x0182, 0x0186,
	0x018B, 0x0190, 0x0194, 0x0199, 0x019D, 0x01A2, 0x01A7, 0x01AB,
	0x01B0, 0x01B4, 0x01B9, 0x01BD, 0x01C2, 0x01C6, 0x01CB, 0x01CF,
	0x01D4, 0x01D8, 0x01DD, 0x01E1, 0x01E6, 0x01EA, 0x01EE, 0x01F3,
	0x01F7, 0x01FB, 0x0200, 0x0204, 0x0209, 0x020D, 0x0211, 0x0215,
	0x021A, 0x021E, 0x0222, 0x0226, 0x022B, 0x022F, 0x0233, 0x0237,
	0x023C, 0x0240, 0x0244, 0x0248, 0x024C, 0x0250, 0x0254, 0x0258,
	0x025D, 0x0261, 0x0265, 0x0269, 0x026D, 0x0271, 0x0275, 0x0279,
	0x027D, 0x0281, 0x0285, 0x0289, 0x028C, 0x0290, 0x0294, 0x0298,
	0x029C, 0x02A0, 0x02A4, 0x02A8, 0x02AB, 0x02AF, 0x02B3, 0x02B7,
	0x02BB, 0x02BE, 0x02C2, 0x02C6, 0x02CA, 0x02CD, 0x02D1, 0x02D5,
	0x02D8, 0x02DC, 0x02E0, 0x02E3, 0x02E7, 0x02EA, 0x02EE, 0x02F2,
	0x02F5, 0x02F9, 0x02FC, 0x0300, 0x0303, 0x0307, 0x030A, 0x030E,
	0x0311, 0x0315, 0x0318, 0x031C, 0x031F, 0x0322, 0x0326, 0x0329,
	0x032D, 0x0330, 0x0333, 0x0337, 0x033A, 0x033D, 0x0340, 0x0344,
	0x0347, 0x034A, 0x034D, 0x0351, 0x0354, 0x0357, 0x035A, 0x035E,
	0x0361, 0x0364, 0x0367, 0x036A, 0x036D, 0x0370, 0x0373, 0x0377,
	0x037A, 0x037D, 0x0380, 0x0383, 0x0386, 0x0389, 0x038C, 0x038F,
	0x0392, 0x0395, 0x0398, 0x039B, 0x039E, 0x03A1, 0x03A3, 0x03A6,
	0x03A9, 0x03AC, 0x03AF, 0x03B2, 0x03B5, 0x03B7, 0x03BA, 0x03BD,
	0x03C0, 0x03C3, 0x03C6, 0x03C8, 0x03CB, 0x03CE, 0x03D1, 0x03D3,
	0x03D6, 0x03D9, 0x03DB, 0x03DE, 0x03E1, 0x03E3, 0x03E6, 0x03E9,
	0x03EB, 0x03EE, 0x03F1, 0x03F3, 0x03F6, 0x03F8, 0x03FB, 0x03FD,
	0x03FF
};


static const int16_t t_cos[24] =
{
	0x0000, 0x18F8, 0x30FB, 0x471C, 0x5A81, 0x6A6C, 0x7640, 0x7D89,
	0x7FFF, 0x7D89, 0x7640, 0x6A6C, 0x5A81, 0x471C, 0x30FB, 0x18F8,
	0x0000, 0xE708, 0xCF05, 0xB8E4, 0xA57F, 0x9594, 0x89C0, 0x8277
};

static const float t_quadrant_degrees[27] = 
{
    -26.0, -24.0, -22.0, -20.0, -18.0, -16.0, -14.0, -12.0,
	  -10.0, -8.0, -6.0, -4.0, -2.0,
	  0.0, // 13 центральная точка таблицы коррекции
	  2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0,
	  18.0, 20.0, 22.0, 24.0, 26.0		
};

static const uint16_t tabl_D[129] =
{
	0x0000, 0x0006, 0x000D, 0x0013, 0x0019, 0x0020, 0x0026, 0x002C,
	0x0032, 0x0039, 0x003F, 0x0045, 0x004C, 0x0052, 0x0058, 0x005E,
	0x0064, 0x006B, 0x0071, 0x0077, 0x007D, 0x0083, 0x0089, 0x008F,
	0x0095, 0x009C, 0x00A2, 0x00A7, 0x00AD, 0x00B3, 0x00B9, 0x00BF, 
	0x00C5, 0x00CB, 0x00D1, 0x00D6, 0x00DC, 0x00E2, 0x00E7, 0x00ED, 
	0x00F3, 0x00F8, 0x00FE, 0x0103, 0x0109, 0x010E, 0x0113, 0x0119, 
	0x011E, 0x0123, 0x0128, 0x012D, 0x0132, 0x0137, 0x013C, 0x0141, 
	0x0146, 0x014B, 0x0150, 0x0155, 0x0159, 0x015E, 0x0163, 0x0167,
	0x016C, 0x0170, 0x0174, 0x0179, 0x017D, 0x0181, 0x0185, 0x0189,
	0x018D, 0x0191, 0x0195, 0x0199, 0x019D, 0x01A0, 0x01A4, 0x01A8,
	0x01AB, 0x01AF, 0x01B2, 0x01B5, 0x01B8, 0x01BC, 0x01BF, 0x01C2,
	0x01C5, 0x01C8, 0x01CA, 0x01CD, 0x01D0, 0x01D2, 0x01D5, 0x01D7,
	0x01DA, 0x01DC, 0x01DE, 0x01E1, 0x01E3, 0x01E5, 0x01E7, 0x01E9,
	0x01EA, 0x01EC, 0x01EE, 0x01EF, 0x01F1, 0x01F2, 0x01F4, 0x01F5,
	0x01F6, 0x01F7, 0x01F8, 0x01F9, 0x01FA, 0x01FB, 0x01FC, 0x01FD,
	0x01FD, 0x01FE, 0x01FE, 0x01FE, 0x01FF, 0x01FF, 0x01FF, 0x01FF,
	0x01FF}; 

//--------------------------------------------------------------------------
//                    Отработка режима "Компас"
//--------------------------------------------------------------------------
void Work_Mode_Compass(void)
{
    
	  tick_hog_sd = 0;
		// Контроль сигнала "Захват СД"
		if(Message_No_Data_Calc())
		{	
        flags_angle = 0;
				size_mass = 0;
				load_f = 0;
        load_fv = 0;
        loaded = 0;					
				tick_time = 0;   
				return;					
		}
   
		if((flags&MESSAGE_WARNING_ABOUT_FAIL))
        return;	
    
		if(size_mass < SIZE_MASS_SAMPLE)
			  return;
		else
        size_mass = SIZE_MASS_SAMPLE;			
    
		// Вычисление угла курса
		GetOrtogonalinyeComponents();	
		
		/*
		if(en_trans==0x01)
		{	
		    UART_SendWord(MDR_UART1, ram_022064);
        
		}	
		*/
		Correction_Low_Freq();
		Old_PASS_512();
		//Correction_Low_Freq();
		
		/*
		if(en_trans==0x01)
		{	
		    UART_SendWord(MDR_UART1, ram_022064);
        
		}	
		*/
		
		/*
		chenge = !chenge;
		if(chenge == 0)
		    Set_Bit_D18D19(T_DATA_DISABLE);
		else
		    Clear_Bit_D18D19(T_DATA_DISABLE);
		*/
		
				
				
		
		if(tick_time < TIME_DELAY)
		    return;
		else
			  tick_time = TIME_DELAY;
		
		
		if(flags & NEW_FREQUENCY_DONE)
		{
		    tick_time = 0;
        flags &= (0xFFFFFFFF -  NEW_FREQUENCY_DONE);
        return;			
    }			
    
		
		// Коррекция квадрантной ошибки
		Correct_Quadrant_Mistake(correct_quadrant_mistake);
		// Учет положения антенны
		ram_022064 = Check_Posit_Antenna(ram_022064);	
		// Контроль сигнала БПРС/ДПРС
    Set_BDPRS(ram_022064);
    // Получение сигналов управления стрелочным прибором	    
		Get_Code_Arrow_Device(ram_022064);	
		// Вывод угла на стрелочный прибор						   
		Set_DAC_Angle();
		// Формируем сообщение КУРС для передатчика
	  Word_Angl_WR( ARINC_ANGLE_COMPAS_OK,   // Матрица состояния
	                ram_022064);             // Курсовой угол 
		//Set_Bit_D18D19(T_DATA_DISABLE);
}
//--------------------------------------------------------------------
//    Функция 
//--------------------------------------------------------------------
void Correction_Low_Freq(void)
{
    int i = 0;
	
	  if(frequency_kGz<200.0)
		{
		    
			  // Преобразование кода угла в градусы
	      Code_To_Degrees();
        analys.angle_mass[analys.i_angle_mass] = angle_degrees;
			  analys.i_angle_mass++;
        if(analys.i_angle_mass == 0)
        {
					  analys.loaded = 0x01;
					  //analys.i_angle_mass = 0;
        }
        if(analys.loaded)
				{
				    analys.m_angle_mass = 0;
					  for(i=0; i<SYZE_ANGLE_MASS; i++)
            analys.m_angle_mass +=  analys.angle_mass[i];
            analys.m_angle_mass /= (float)SYZE_ANGLE_MASS; 
            
					
					  analys.d_angle_mass[analys.i_d] = 0;
     				for(i=0; i<SYZE_ANGLE_MASS; i++)
					  {
					      analys.delta = analys.m_angle_mass - analys.angle_mass[i];
							  if(analys.delta<0)
									analys.delta *= -1.0;
								analys.d_angle_mass[analys.i_d] += analys.delta;
						}	
						analys.d_angle_mass[analys.i_d] /= (float)SYZE_ANGLE_MASS; 
						
					  analys.i_d++;
						if(analys.i_d >= SYZE_D_MASS)
						{
						    analys.i_d = 0;
							  analys.ready = 0x01;
						}	  
            if(analys.ready)
            {							
						    analys.d_angle = 0;
							  for(i=0; i<SYZE_D_MASS; i++)	
							  analys.d_angle += analys.d_angle_mass[i];
							  analys.d_angle /= (float) SYZE_D_MASS; 							  
            }							
												
            analys.e_angle = 270.0 - analys.m_angle_mass;	
            analys.e_angle = 0.0;						
				}
			
			  
			  if(analys.ready)
				{
           if(analys.d_angle>1.0) 
					 angle_degrees += 3.0; 					
						 			
			  }
				// Преобразование градусов в код угла
		    Degrees_To_Code();					
		}	
}
//--------------------------------------------------------------------
//    Функция определения модуля числа int16_t
//--------------------------------------------------------------------
int16_t my_abs(int16_t value)
{
    if(value<0)
			  value *= -1;
		return value;
}
//--------------------------------------------------------------------
//    Помещение в массив отсчетов угла курса
//--------------------------------------------------------------------
void Set_To_Angle_Mass(void)
{
    /*
	  uint8_t i = 0;
	  for(i = 0; i<255; i++)
	  angle_mass.angle[i] = angle_mass.angle[i+1]; 
    angle_mass.angle[255] = ram_022064;
	  */
}


//--------------------------------------------------------------------
//    Работа в режиме Антенна
//--------------------------------------------------------------------
void Work_Mode_Antenna(void)
{
    if(tick_time < (TIME_DELAY))
		    return;
		else
			  tick_time = (TIME_DELAY);
	
	  Message_No_Data_Calc_Antenna();	  
}	
//--------------------------------------------------------------------
//    Задержка 1 секунда
//--------------------------------------------------------------------
void Delay_One_Second(void)
{
	  tick_time = 0;
	  while(tick_time<=TIME_ONE_SEC);
}	
//--------------------------------------------------------------------
// Check position antenna and correct code angle D27_D30:5-p
//--------------------------------------------------------------------
int16_t Check_Posit_Antenna(int16_t angle)
{
    if(antenna_h == 0)
	{
	    if(angle > 0)
		    angle -= 4096;
		else
		    angle += 4096;
		angle *= -1;			
	}
	return angle;	
}
//--------------------------------------------------------------------
//      Check and set signals BPRS or DPRS reg D18_D19:9-p
//--------------------------------------------------------------------
void Set_BDPRS(int16_t angle)
{
	  static uint8_t prev_marker = 0;  
	  
	  uint8_t cur_marker;
	  if (angle < 0) 
			  angle *= -1;     // Берем модуль
	  if (angle <= 910) 
			  cur_marker = 1;     // Меньше 40 градусов 
	  else 
			  cur_marker = 0;
	  if(cur_marker != prev_marker)
    {
		    if(cur_marker) 
					  Set_BPRS(); 
				else 
					  Set_DPRS();
			  prev_marker = cur_marker;				
		}	
}
//--------------------------------------------------------------------
//       Сообщение "90 градусов, Нет данных для вычисления"
//--------------------------------------------------------------------
uint8_t Message_No_Data_Calc(void)
{
	  
	  if(Check_SD() == HOG_SF_RESET) 
		{	
   	    if((flags&MESSAGE_NO_DATA_CALC) == 0)
				{	
			      // Получение сигналов управления стрелочным прибором	    
			      Get_Code_Arrow_Device(2048);
					  flags |= MESSAGE_NO_DATA_CALC;
            // Указатель на 90 градусов
            Set_DAC_Angle(); 
            //Init_L_PASS_Cheb_64();					
				    // Формируем сообщение КУРС для передатчика
				    Word_Angl_WR( ARINC_ANGLE_NO_DATA,       // Матрица состояния
	                        2048);                      // Курсовой угол
				    
				}
        return 0x01;				
		}	
		else
		{	
				flags &=  (0xFFFFFFFF - MESSAGE_NO_DATA_CALC);
        return 0;			
		}	
}	
//--------------------------------------------------------------------
//   Коммутатор сообщений
//--------------------------------------------------------------------
void Switch_Message(void)
{
    if((flags & ENABLE_FUNC_CONTR )||(flags & ENABLE_OPER_CONTR))
		{
		    // Передача сообщений об отказе
			  if((flags & FAIL_ARINC_RC)||          // Потеря сигнала по ARINC429 
			     (flags & FAIL_SELF_TEST_IN_A)||    // Отказ самотест вход А 
			     (flags & FAIL_SELF_TEST_IN_B)||    // Отказ самотест вход B
			     (flags & FAIL_SELF_TEST_NO_INFO)|| // Отказ самотест нет информации на входе
			     (flags & FAIL_CPU)||               // Отказ контроллера 
			     (flags & FAIL_MEASURE_ANGLE)||     // Отказ измерения контрольного угла
           (flags & FAIL_HOG_SD)||  		      // Нет захвата СД
			     (flags & FAIL_HOG_SF))             // Нет захвата СЧ
			  {	
			      Message_Warning_about_fail();
			  }
			  else
			  {
          flags &= (0xFFFFFFFF - MESSAGE_WARNING_ABOUT_FAIL);
			  }		  
    }
    else
    {
			  // Передача сообщения 90 градусов Нет Данных АРК
			  if(flags & FAIL_ARINC_RC)
			  {
			      Message_No_Data_Calc_Antenna();
            state_working = NOTHING;				
        }	
        else
        {
				    flags &= (0xFFFFFFFF - MESSAGE_NO_DATA_CALC);
        }			
    }
}	
//--------------------------------------------------------------------
//       Сообщение "90 градусов, Нет данных для вычисления"  в режиме АНТЕННА
//--------------------------------------------------------------------
void Message_No_Data_Calc_Antenna(void)
{
	  if((flags&MESSAGE_NO_DATA_CALC) == 0)
		{	
			   // Получение сигналов управления стрелочным прибором	    
			   Get_Code_Arrow_Device(2048);	
         // Указатель на 90 градусов
         Set_DAC_Angle(); 					
				 // Формируем сообщение КУРС для передатчика
				 Word_Angl_WR( ARINC_ANGLE_NO_DATA,       // Матрица состояния
	                     2048);                     // Курсовой угол 
			   // Формируем сообщение Частота для передатчика
				 //Out_Word_Frequency_WR(ARINC_FR_OK ); // Матрица состояния 
				 flags |= MESSAGE_NO_DATA_CALC;
		}    	
}
//--------------------------------------------------------------------
//       Сообщение "90 градусов, Предупреждение об отказе"
//--------------------------------------------------------------------
void Message_Warning_about_fail(void)
{
	 	if((flags&MESSAGE_WARNING_ABOUT_FAIL)==0)
		{	
	      // Получение сигналов управления стрелочным прибором	    
	      Get_Code_Arrow_Device(2048);	
        // Указатель на 90 градусов
        Set_DAC_Angle(); 					
		    // Формируем сообщение КУРС для передатчика
		    Word_Angl_WR( ARINC_ANGLE_FAILRUE,       // Матрица состояния
	                    2048);                     // Курсовой угол
			  flags |= MESSAGE_WARNING_ABOUT_FAIL;
		}
}	

//--------------------------------------------------------------------
//  Управление стрелочным прибором
//--------------------------------------------------------------------
uint32_t Get_Code_Arrow_Device(int16_t code_angl)    // Procedure_005624()
{
	_REGISTR  D2, D4, D5;
	_TWO_WORD D2D3;	
	
	D5._word = code_angl;
	
	if(D5._word < 0) D5._word ^= 0xFFFF; 
	if(D5._word >= 2048) // >90 grad
	   {
		  D5._word -= 4095; // -180 grad
		  D5._word *= -1; 
	   }
	D4._word = D5._word;
	D5._word &= 0x7F0;
	D4._word -= D5._word;
 	D5._word /= 2;	 
  D5._word /= 2;
	D5._word /= 2;
	D5._word /= 2; // add my	 
		 
	D1._word = tabl_D[D5._word];	 
	D2._word = tabl_D[D5._word+1];
	D2._word -= D1._word;	

	D2D3._dword = D2._word * D4._word;
	D2D3.words.r_lo._word /= 2;
	D2D3.words.r_lo._word /= 2;
    D2D3.words.r_lo._word /= 2;
	D2D3.words.r_lo._word /= 2;
		 
	D1._word +=	D2D3.words.r_lo._word;  // unsigned code control X 
	
  D0._word = tabl_D[128 - D5._word]; // попробовать поменять 128 и 127
  D2._word = tabl_D[127 - D5._word];
	D2._word -= D0._word;
	
	D2D3._dword = D2._word * D4._word;
	D2D3.words.r_lo._word /= 2;
	D2D3.words.r_lo._word /= 2;
    D2D3.words.r_lo._word /= 2;
	D2D3.words.r_lo._word /= 2;
	
	D0._word +=	D2D3.words.r_lo._word;  // unsigned code control Y

  D0._word *= 2;   // Глобальные переменные управления стрелочным
	D1._word *= 2;	 // прибором
	
	if(code_angl >= 0)
	//if((code_angl >= 0) && (code_angl <= 4095))
  		{
				if(code_angl >= 2048) // >= 90 grad 
				   D0._word++;
      }
	else 
      {
		   D1._word++;
		   if(code_angl <= -2048) // >= 270 grad // >
				  D0._word++;
	    }
    // D0._word - signed code control Y
	// D1._word - signed code control X
	
	// Shaping code control for DAC
	code_DAC = 0;
    code_DAC =  D1._uword; 
    code_DAC <<= 10;
	D1._uword &= 0x03FF;		
    code_DAC |= D0._uword; 
    return code_DAC;			
}

//--------------------------------------------------------------------------
//    Преобразование угла курса в градусах в код
//--------------------------------------------------------------------------	
uint16_t Degrees_To_Code_Value(float value)
{
    float weit_bit = 0.5;
	  uint16_t n_bit = BIT11;
    uint16_t code_angle = 0;
	  uint8_t i = 0;
	  float value_in = value; 
	
    
    if(value_in >= 180.0)
		{
			  value_in = (360.0 - value_in)/180.0;
		    
			  weit_bit = 0.5;
			  n_bit = BIT11;
		    for(i = 0; i<12; i++)
		    {
			      if((value_in / weit_bit)>= 1.0)
            {
						    value_in -= weit_bit;
							  code_angle += n_bit;
						}
            weit_bit /= 2.0;
			      n_bit >>= 1;
	      }	 
				code_angle = ~code_angle +1;				
    }
    else
    {
			  value_in /= 180.0;
		    
			  weit_bit = 0.5;
			  n_bit = BIT11;
		    for(i = 0; i<12; i++)
		    {
			      if((value_in / weit_bit)>= 1.0)
            {
						    value_in -= weit_bit;
							  code_angle += n_bit;
						}
            weit_bit /= 2.0;
			      n_bit >>= 1;
	      }	 
				code_angle &= 0x0FFF;				
    }			
		return  code_angle;	
}	
//--------------------------------------------------------------------------
//    Преобразование угла курса в градусах в код
//--------------------------------------------------------------------------	
void Degrees_To_Code(void)
{
    float weit_bit = 0.5;
	  uint16_t n_bit = BIT11;
    uint16_t code_angle = 0;
	  uint8_t i = 0;
	
	  if(angle_degrees >= 180.0)
		{
			  angle_degrees = (360.0 - angle_degrees)/180.0;
		    
			  weit_bit = 0.5;
			  n_bit = BIT11;
		    for(i = 0; i<12; i++)
		    {
			      if((angle_degrees / weit_bit)>= 1.0)
            {
						    angle_degrees -= weit_bit;
							  code_angle += n_bit;
						}
            weit_bit /= 2.0;
			      n_bit >>= 1;
	      }	 
				code_angle = ~code_angle +1;				
    }
    else
    {
			  angle_degrees /= 180.0;
		    
			  weit_bit = 0.5;
			  n_bit = BIT11;
		    for(i = 0; i<12; i++)
		    {
			      if((angle_degrees / weit_bit)>= 1.0)
            {
						    angle_degrees -= weit_bit;
							  code_angle += n_bit;
						}
            weit_bit /= 2.0;
			      n_bit >>= 1;
	      }	 
				code_angle &= 0x0FFF;				
    }			
		ram_022064 = code_angle;	
}	
//--------------------------------------------------------------------------
//    Преобразование кода угла курса в градусы
//--------------------------------------------------------------------------
float Code_To_Degrees_Value(uint16_t value)
{
    uint8_t bit_sign = 0;
	  float weit_bit = 0.5;
	  uint16_t n_bit = BIT11;
    uint16_t code_angle = value;
	  uint8_t i = 0;

	
 	  float ret = 0.0;	
	
	  bit_sign = (code_angle & BIT12) == BIT12;
    code_angle &= 0x1FFF;
	
	  if(bit_sign)
	  {
		    code_angle = ~code_angle;
		    weit_bit = 0.5;
		    for(i = 0; i<12; i++)
		    {
			      if(code_angle & n_bit)
				        ret += weit_bit;
			      weit_bit /= 2.0;
			      n_bit >>= 1;
		    }
		    ret = 360.0 - ret*180.0;
	}
	else
	{

		    weit_bit = 0.5;
		    for(i = 0; i<12; i++)
		    {
			      if(code_angle & n_bit)
				        ret += weit_bit;
			      weit_bit /= 2.0;
			      n_bit >>= 1;
		    }
		    ret *= 180.0;
	}	
	
	if(ret == 360.0)
		  ret = 0.0;
	return ret;
}	
//--------------------------------------------------------------------------
//    Преобразование кода угла курса в градусы
//--------------------------------------------------------------------------
void Code_To_Degrees(void)
{
    
	
	  uint8_t bit_sign = 0;
	  float weit_bit = 0.5;
	  uint16_t n_bit = BIT11;
    uint16_t code_angle = ram_022064;
	  uint8_t i = 0;

	
 	  angle_degrees = 0.0;	
	
	  bit_sign = (code_angle & BIT12) == BIT12;
    code_angle &= 0x1FFF;
	
	  if(bit_sign)
	  {
		    code_angle = ~code_angle;
		    weit_bit = 0.5;
		    for(i = 0; i<12; i++)
		    {
			      if(code_angle & n_bit)
				        angle_degrees += weit_bit;
			      weit_bit /= 2.0;
			      n_bit >>= 1;
		    }
		    angle_degrees = 360.0 - angle_degrees*180.0;
	}
	else
	{

		    weit_bit = 0.5;
		    for(i = 0; i<12; i++)
		    {
			      if(code_angle & n_bit)
				    angle_degrees += weit_bit;
			      weit_bit /= 2.0;
			      n_bit >>= 1;
		    }
		    angle_degrees *= 180.0;
	}
  if(angle_degrees == 360.0)
	    angle_degrees = 0.0;	
	
}
//--------------------------------------------------------------------
//  Функция коррекции квадрантной ошибки
//--------------------------------------------------------------------
void Correct_Quadrant_Mistake(uint16_t correct_quadrant_mistake)
{
    float Ampl = 0.0;  
	  _REGISTR r4;
	
	  // Преобразование кода угла в градусы
	  Code_To_Degrees(); 
	  // Выбор амплитуды коррекции
	  r4._word = correct_quadrant_mistake;
	  // Выделение значения сигналов коррекция квадрантной ошибки <0..4>
	  r4._uword &= 0x1F;
	  if (r4._uword&0x10)    // <4>  Бит знака, при  1 означает  "-"
	      r4._uword |= 0xFFF0; 
	  if (r4._word > 13)
		    r4._word = 13; // + 13    // Переполнение сверху
	  else
	  {
	      if (r4._word < (-13))
			  r4._word = -13;       // Переполнение снизу
	  }
	  // Выборка амплитуды коррекции из таблицы 
		Ampl = t_quadrant_degrees[r4._word+13]; 
		// Расчет величины коррекции
		Ampl *= sin(angle_degrees*RADIAN_PER_DEGREE_2);
		// Получение откорректированной величины
	  angle_degrees += Ampl;
		// Преобразование градусов в код угла
		Degrees_To_Code();	  
}


//--------------------------------------------------------------------
// Reception and processing ortogonal components
// Entry - two arrays estimation AD for switching sin = 1 and 0
// Out - code of heading angle in ram_022064
//--------------------------------------------------------------------
void GetOrtogonalinyeComponents(void)    // Procedure_004210()
{	  
	   // Сжатие массива 512 в полупериод, вычисление компонент
	   Sin_Cos_Comp((int16_t*)switch_sin_0._word);		 
	   _cos._dword = component_cos._dword;
	   _sin._dword = component_sin._dword;	
	   Sin_Cos_Comp((int16_t*)switch_sin_1._word);		 
	   _sin._dword -= component_sin._dword;
	   _cos._dword += component_cos._dword;
	   _cos._udword ^= 0xFFFFFFFF;	
	   // Issue for processing in ProcessOrtogonalinyeComponents()
	   R0R1._udword = _sin._udword;
	   R2R3._udword = _cos._udword;
	   ProcessOrtogonalinyeComponents();  // Procedure_004610() out in R1
	   // Restore senior words a component in register
	   R2._word = _sin.words.r_hi._word;
	   R3._word = _cos.words.r_hi._word;
	   R2._uword ^= R3._uword;
	   // Correction r1 on r2 and ram_/022050
	   if (R2._word < 0)
		     goto m_004404;
	   if (_sin.words.r_hi._word >= 0)
		     goto m_004424;
	   R1._word -= 4096;       // yes
	   goto m_004424;
m_004404:
	   if (_sin.words.r_hi._word < 0) 
		     goto m_004422;
	   R1._word -= 4095;  // 61 gr -> 241 gr 
	   R1._word *= -1;    // 241 gr -> 118 gr
	   goto m_004424;
m_004422:
	   R1._uword ^= 0xFFFF; // Inversion
m_004424:
	   R1._word += 2048; // 12-p // 118 gr -> 208  // yes
	   // Correction accouting heading angle data in r1 on border importances
	   Procedure_005460(); // 208 gr ->  // no
	   // Save accouting headin3g angle data in RAM
	   ram_022064 = R1._uword;		 
}
//--------------------------------------------------------------------
void Filter(void)
{
	int i = 0;
	
	if(loaded==0)
	{
		// Preliminary loading the array 1
		for(i=0; i <64; i++)
		mass1[i] = ram_022064;   // Estimation of the angle
		loaded = 0xFF;
	}
	else
	{
		
      		
		// Rerecording the array 1 with new importance of the angle in last element
		for(i=0; i <63; i++)
		mass1[i] = mass1[i+1];
		mass1[63] = ram_022064;   // Estimation of the angle

		// Writing the array 2
		for(i=0; i<32; i++)
		{
			R0._word = mass1[i];
			R1._word = mass1[i+32];
			Procedure_004110();
			mass2[i] = R0._word;
		}

		for(i=0; i<31; i++)
		{
			R0._word = mass2[i];
			R1._word = mass2[i+1];
      Procedure_004110();
			//mass3[i] = R0._word;
    }
		ram_022064 = R0._uword;
				
	} 
}
//--------------------------------------------------------------------
// Processing two namber R0 and R1
void Procedure_004110(void)
{
	R2._word = R1._word;
	R2._uword ^= R0._uword;
	if (R2._word >= 0)
	{
		R0._word += R1._word;
		R0._word /= 2;
		return;
	}
	R2._word = R1._word;
	if (R2._word < 0)
		R2._word *= -1;
	if (R0._word < 0)
		R2._word -= R0._word;
	else
		R2._word += R0._word;
	if (R2._word < 4096)
	{
		R0._word += R1._word;
		R0._word /= 2;
		return;
	}
	if (R0._word >= 0)
	{
		if (R1._word < 0)
			R1._word += 0x2000;
	}
	else
		R0._word += 0x2000;
	R1._word += R0._word;
	R1._word /= 2;
	// Correction accounting heading angle data in r1 on border importances
	if (R1._word >= 4096) // 4096                               
		R1._word -= 0x2000;
	else
	{
		if(R1._word < (-4096))
			R1._word += 0x2000;
	}
  R0._word = R1._word;
}
//--------------------------------------------------------------------
// Entry R0R1 and R2R3, output in R1
void ProcessOrtogonalinyeComponents(void)  // Procedure_004610()
{
	ram_022060 = 0;
	if(R0R1.words.r_hi._word < 0)
		R0R1._udword ^= 0xFFFFFFFF;
	if(R2R3.words.r_hi._word < 0)
		R2R3._udword ^= 0xFFFFFFFF;
	R4R5._udword = R2R3._udword;
	R4R5._dword -=  R0R1._dword;
	if (R4R5.words.r_hi._word > 0)
		goto m_004704; // r4r5 = cos(i)-sin(i) > 0
	if (R4R5.words.r_hi._word < 0)
		goto m_004664; // r4r5 = cos(i)-sin(i) < 0
	// r4 = 0 (senior word in r4r5 = cos(i)-sin(i)
	if (R4R5.words.r_lo._uword != 0) // (younger word in r4r5 = cos(i)-sin(i) !=0
			goto m_004664;
	// (younger word in r4r5 = cos(i)-sin(i) ==0
	R1._word = 1024; // Correction younger word sin copnent r0r1
	return;
m_004664:  // r4r5 = cos(i)-sin(i) < 0
	// Inversion ram_022060
	ram_022060 ^= 0xFFFF; // result ram_022060 = 0xFFFF
	// Copying cos component r2r3 in r4r5
	R4R5._udword = R2R3._udword;
	// Copying sin component r0r1 in r2r3
	R2R3._udword = R0R1._udword;
	// Copying cos component r4r5 in r0r1
	R0R1._udword = R4R5._udword;
m_004704:
	// r4r5 = cos(i)-sin(i) > 0
	// Shift (<<=8) components sin and cos at condition Hi(sin)<000200
	Procedure_005122();
	Procedure_005122();
	// Operations with senior words ortogonal components
	if (R2R3.words.r_hi._word<= R0R1.words.r_hi._word)
	{
		if (R2R3.words.r_hi._uword == 0x7FFF)
			R0R1.words.r_hi._uword -=1; // Decrement of the senior word
		else
			R2R3.words.r_hi._uword++; // Increment of the senior word
	}
	R0R1._dword /= 4;
	// Attitude ortogonal component (/)
	R0._uword = R0R1._udword/R2R3.words.r_hi._uword; 
	// Save attitude in r4
	R4._uword = R0._uword;
	// Select group a bit in r0
	R0._uword &= 0x3FC0;
	// Correction relations (/) component with provision for these bit
	R4._word -= R0._word;
	// Shift with provision for sign
	R0._word /= 2;
	R0._word /= 2;
	R0._word /= 2;
	R0._word /= 2;
	R0._word /= 2;
	R0._word /= 2;
	// r0 allows to position two nearby counting outs in table
	// trigonometric function from 256 elements
	R1._uword = tabl[R0._word];
	R2._uword = tabl[R0._word +1];
	// Incrementation between two nearby elements in table
	R2._word -= R1._word;
	// Multiply incrementation on attitude component
	R2R3._dword= R2._word * R4._word;
	// Younger part of product shift with provision sign
	R3._uword = R2R3.words.r_lo._uword;
	R3._word /= 2;
	R3._word /= 2;
	R3._word /= 2;
	R3._word /= 2;
	R3._word /= 2;
	// Summiruem with first element from table with provision r0
	R1._word += R3._word;	
	
	// Checking the cell RAM
	if (ram_022060 != 0)
	{
		R1._word -= 2047;
		R1._word *= -1;
	}	
	 
}
//--------------------------------------------------------------------
void Procedure_005122(void)
{
	// Entry sin component in r2r3, cos component in r0r1
	if (R2R3.words.r_hi._word >= 128)
		return;
	// Shift cos component in r0r1
	R0R1._udword <<= 8;
	// Shift sin component in r2r3
	R2R3._udword <<= 8;	
}
//--------------------------------------------------------------------
// Correction R1 on border condition
void Procedure_005460(void)
{
	if(R1._word >= 4096)
		R1._word -= 8192; // - 0 grad
	else
	{
		if(R1._word < (-4096)) // -4096
			R1._word += 8192;
	}	
}
//--------------------------------------------------------------------
void Eight_periods_to_half_period(int16_t* dac) // Procedure_005174()
{
  uint8_t r2 = 0;        // counter of the array of the compressed halftime (0,5T)
	uint8_t r3 = 0;        // counter
	uint16_t r4 = 0;       // current offset in array
	uint16_t r5 = PERIOD;  // address of the first point in array (32)
	_REGISTR r0;

	// Unset components (32 bit word)
	component_cos._udword = 0;
	component_sin._udword = 0;
	r0._uword = 0;
   
	// Compression of the sample before halftime (16 counting out)
	for( r2 = 0; r2 < 16; r2++)
	{
		r0._word = dac[r5]-dac[r5+HALF_PERIOD]; // HALF_PERIOD = 16

		r4 = 0;
		for( r3 = 0; r3 < 6; r3++)
		{
			r4 += PERIOD;
			r0._word += dac[r5+r4]- dac[r5+r4+HALF_PERIOD]; // HALF_PERIOD = 16
		}
		r0._word /= 8;

		half_period[r2] = r0._word;
		// Following point
		r5++;
	}
	// Calculation sin and cos components
	for(r2 = 0; r2 <16; r2++)
	{
		component_sin._dword += half_period[r2]*t_cos[r2+OFFSET_SIN];
		component_cos._dword += half_period[r2]*t_cos[r2];
	}  
}
//--------------------------------------------------------------------

void Sin_Cos_Comp(int16_t* dac)
{  
	
  int32_t compress[16] = {0};	   
	uint16_t i, j;
	
	// Unset components (32 bit word)
	component_cos._udword = 0;
	component_sin._udword = 0;
	
	// Сжатие 512 отсчетов (16 периодов) до 16 (одного полупериода)
	for(j=0; j<16; j++) // 16 точек полупериод
		  {
			  for(i=1; i<16; i++) // 16 периодов  0..16 1..7
			    {
				    compress[j] += dac[32*i+j] - dac[32*i+16+j];						
			    }
			  compress[j] /= 16; //16;
		  }			
			
			// Вычисление sin и cos компонент
	for(i = 0; i <16; i++)
	{
		component_sin._dword += compress[i]*t_cos[i+8];
		component_cos._dword += compress[i]*t_cos[i];
	}
	
	/*
	uint8_t r2 = 0;        // counter of the array of the compressed halftime (0,5T)
	uint8_t r3 = 0;        // counter
	uint16_t r4 = 0;       // current offset in array
	uint16_t r5 = PERIOD;  // address of the first point in array (32)
	_REGISTR r0;

	// Unset components (32 bit word)
	component_cos._udword = 0;
	component_sin._udword = 0;
	r0._uword = 0;
   
	// Compression of the sample before halftime (16 counting out)
	for( r2 = 0; r2 < 16; r2++)
	{
		r0._word = dac[r5]-dac[r5+HALF_PERIOD]; // HALF_PERIOD = 16

		r4 = 0;
		for( r3 = 0; r3 < 6; r3++)
		{
			r4 += PERIOD;
			r0._word += dac[r5+r4]- dac[r5+r4+HALF_PERIOD]; // HALF_PERIOD = 16
		}
		r0._word /= 8;

		half_period[r2] = r0._word;
		// Following point
		r5++;
	}
	// Calculation sin and cos components
	for(r2 = 0; r2 <16; r2++)
	{
		component_sin._dword += half_period[r2]*t_cos[r2+OFFSET_SIN];
		component_cos._dword += half_period[r2]*t_cos[r2];
	}
*/  
}

