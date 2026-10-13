//  Этот файл содержит функции измерения угла курса
//--------------------------------------------------------------------
#ifndef __APK25_ANGLE_H
#define __APK25_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_lib.h"


#define PERIOD             32            // Период - 32 отсчета
#define HALF_PERIOD        16            // Полупериод - 16 отсчетов
#define OFFSET_SIN          8
#define SIZE_MASS_SAMPLE    1024         // Размер объема выборки	
	
	
#define INVALID_ANGLE      0x01
#define VALID_ANGLE	       0x00 
	

	
// Флаги состояний 
#define FAIL_ARINC_RC                   0x00000001
#define FAIL_HOG_SF                     0x00000002
#define FAIL_HOG_SD                     0x00000004
#define HOG_SD_EXIST                    0x00000008
#define ENABLE_FUNC_CONTR               0x00000010   // Разрешение функционального контроля 
#define ENABLE_OPER_CONTR               0x00000020   // Разрешение оперативного контроля 	
#define FAIL_INPUT_MESSAGE              0x00000040   // Дефект входного сообщения 
#define FAIL_CPU                        0x00000080   // Дефект контроллера	
#define NEW_FREQUENCY_DONE              0x00000100   // Перестройка приемника была 
#define FAIL_MEASURE_ANGLE              0x00000200   // Отказ измерения угла 
#define FAIL_SELF_TEST_IN_A             0x00000400   // Отказ самотест вход А
#define FAIL_SELF_TEST_IN_B             0x00000800   // Отказ самотест вход B
#define FAIL_SELF_TEST_NO_INFO          0x00001000   // Отказ самотест нет информации на входе
#define SHOW_FAIL_FC                    0x00002000   // Индикация отказа при ФК
#define REPEAT_TEST                     0x00004000   // Повторить проверку
#define DISABLE_ANALYS_MESSAGE          0x00008000   // Блокировать анализ сообщения Частота/Функция
#define HOG_NOES_SD			          			0x00010000   // Захват СД по помехе
//#define HOG_NOES_SD			          			0x00000000   // Захват СД по помехе
#define SET_CHECK_ANTENNA               0x00020000   // Включение контрольного сигнала антенны
#define SET_CHECK_RECEIVER				      0x00040000   // Включение контрольного сигнала приемника
#define SET_MODE_TLPH                   0x00080000   // Установлен режим телефон
#define SET_MODE_TLGR                   0x00100000   // Установлен режим телеграф


#define MESSAGE_WARNING_ABOUT_FAIL      0x01000000   // Сообщение о предупреждении об отказе передано	
#define MESSAGE_NO_DATA_CALC            0x02000000   // Сообщение Нет данных для расчета передано  	


#define RADIAN_PER_DEGREE_2             0.034906585  // Радиан в градусе удвоенное


#define SELECT_FILTER                   0x01         // Выбор фильтра по дисперсии оценки угла
#define SELECTED_FILTER_0               0x02         // Выбран фильтр без сдвига  
#define SELECTED_FILTER_180             0x04         // Выбран фильтр со сдвигом 180 градусов 
#define CHANGE_FILTER                   0x08         // Смена фильтра Чебышева
#define DISPERS                         0x10

#define LEVEL   40


//--------------------------------------------------------------------
typedef union
{
	int16_t _word;
	uint16_t _uword;
} _REGISTR;

typedef union
{
	struct
	{
		_REGISTR r_lo;
		_REGISTR r_hi;
	}words;
	int32_t   _dword;
	uint32_t _udword;
} _TWO_WORD;



/*
typedef union 
{
	unsigned char _byte[512];
	uint16_t      angle[256];
} AM;
*/
#define SYZE_ANGLE_MASS  256
#define SYZE_D_MASS      100

typedef  struct
{
    float angle_mass[SYZE_ANGLE_MASS];
    float m_angle_mass;
    float d_angle_mass[100];
	  float delta; 
    uint8_t i_angle_mass;
    uint8_t i_d;
    float d_angle;	
	  uint8_t loaded;
	  float e_angle;
	  uint8_t ready;
} ANALYS_ANGLE;
	




//--------------------------------------------------------------------




//--------------------------------------------------------------------
void Filter(void);
uint32_t Get_Code_Arrow_Device(int16_t code_angl);
uint8_t Message_No_Data_Calc(void);
void Message_Warning_about_fail(void);
void Message_No_Data_Calc_Antenna(void);
void Switch_Message(void);
void Delay_One_Second(void);
void Work_Mode_Compass(void);
void Work_Mode_Antenna(void);
void GetOrtogonalinyeComponents(void); 
void Set_To_Angle_Mass(void);
void Correction_Low_Freq(void);
float Code_To_Degrees_Value(uint16_t value);

#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 


