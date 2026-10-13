//  Этот файл содержит функции работы шасси
//--------------------------------------------------------------------
#ifndef __APK25_CHASSIS_H
#define __APK25_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MDR1986VE1T.h"


//--------------------------------------------------------------------
// Матрица состояния в выходном сообщении КУРС
#define ARINC_ANGLE_COMPAS_OK  3<<29
#define ARINC_ANGLE_FC         1<<30
#define ARINC_ANGLE_NO_DATA    1<<29
#define ARINC_ANGLE_FAILRUE	   0 
	
// Матрица состояния в выходном сообщении Частота/Состояние
#define ARINC_FR_OK            0
#define ARINC_FR_SET_NEW       1<<29
#define ARINC_FR_UNDEFINE      3<<29
#define ARINC_FR_FC            1<<30

// Режимы и подрежимы АРК
#define ARINC_MODE_COMPAS      0
#define ARINC_MODE_ANTENNA     1<<11 	
#define ARINC_MODE_TELEPHON    0
#define ARINC_MODE_TELEGRAPH   1<<10


// USE ??????
#define ERROR_HOG_SF		       0x0001
#define ERROR_HOG_SD  		     0x0002	
#define ERROR_HOG_SF_1		     0x0004   // 
#define ERROR_HOG_SF_2	       0x0008
#define ERROR_HOG_SF_3	       0x0010
#define ERROR_HOG_SF_4	       0x0020
#define ERROR_HOG_SF_5	       0x0040
#define ERROR_HOG_SF_6	       0x0080
#define ERROR_HOG_SD_RC1		   0x01000
#define ERROR_HOG_SD_RC2		   0x02000
#define ERROR_HOG_SD_RC3		   0x04000
#define ERROR_HOG_SD_RC4		   0x08000
#define ERROR_HOG_SD_RC5		   0x010000
#define ERROR_HOG_SD_RC6		   0x020000
#define ERROR_HOG_SD_AN1		   0x040000
#define ERROR_HOG_SD_AN2		   0x080000
#define ERROR_HOG_SD_AN3		   0x0100000
#define ERROR_HOG_SD_AN4		   0x0200000
#define ERROR_HOG_SD_AN5		   0x0400000
#define ERROR_HOG_SD_AN6		   0x0800000

// Биты приемника D27D30
#define BITS_CORR_QUADRANT_ERR	   0x001F   // Биты коррекции квадрантной ошибки <0..4>
#define LOC_ANTENNA                0x0020   // Размещение антенны
#define HOG_SF				             0x0040	  // Захват СЧ	 
#define HOG_SD				             0x0080	  // Захват СД
#define ID_DEVICE_0                0x0100	  // Идентификатор приемника <0>
#define ID_DEVICE_1                0x0200	  // Идентификатор приемника <1>
#define SELECT_AB			             0x0400	  // Выбор данных A/B 
#define FUNC_CHECK_ON              0x0800	  // Включение функционального контроля
#define NO_INPUT_INFO              0x1000	  // Нет входной информации
#define LAND_AIR			             0x2000   // Земля/Воздух
#define RECORD_ENABLE              0x4000   // Разрешение записи NC
#define KPA_TURN				           0x8000   // Подключение КПА

#define CHANNEL_A                  0x0400   // Канал A
#define CHANNEL_B                  0x0000   // Канал B 


#define HOG_SF_SET                 0x01
#define HOG_SF_RESET               0x00
#define HOG_SD_SET                 0x01
#define HOG_SD_RESET               0x00


// Бит приемника D26
#define DISABLE_FC                 0x0080  // Запрет ФК


// Биты регистров  D18D19
#define SELECT_F_RANGE_0           0x0001	// Выбор поддиапазона
#define SELECT_F_RANGE_1           0x0002 
#define SELECT_F_RANGE_2           0x0004
#define SYN_DAC					           0x0008   // Синхронизация ЦАП
#define DATA_DAC                   0x0010   // Информация ЦАП
#define CLOCK_DAC                  0x0020   // Такт ЦАП
#define COMPASS_ANTENNA            0x0040   // Компас/Антенна
#define TLPH_TLGR				           0x0080   // Телефон/Телеграф
#define PULSE_F					           0x0100   // Импульс перестройки
#define DPRS                       0x0200   // ДПРС
#define T_DATA_DISABLE			       0x0400   // Запрет передачи данных
#define REGISTR_STATE_2            0x0800 	// Биты регистра состояния
#define REGISTR_STATE_3            0x1000	
#define REGISTR_STATE_1            0x2000
#define REGISTR_STATE_0            0x4000


// Биты регистров D28D29
#define CHECK_ADC                  0x0001 	// Проверка АЦП
#define CHECK_RECEIVER             0x0002   // Проверка приемника
#define CHECK_ANTENNA              0x0004   // Проверка антенны
#define SELECT_IN_ARINC_0          0x0008   // Выбор входа ЦПП <0> NC
#define SELECT_IN_ARINC_1          0x0010   // Выбор входа ЦПП <1> NC
#define CLOCK_SF       	     		   0x0020   // Такт СЧ
#define DATA_SF                    0x0040   // Код настройки СЧ
#define SYN_SF					           0x0080   // Синхронизация СЧ
#define INPUT_WORD_DISABLE         0x0100   // Блокировка выходного слова
#define SET_RT                     0x0200   // Установка ЦПП
#define PREPAR_RECORD              0x0400   // Подготовка записи
#define ANGLE_FR				           0x0800   // Курс/Частота
#define FR_ROLL					           0x1000	// Частота/Крен 


#define MASK_SELECT_IN_ARINC	   0xFFE7     // MASKS
#define MASK_REGISTR_STATE         0x87FF	    
#define MASK_DATA_DAC			   0xFFEF
#define MASK_CLOCK_DAC			   0xFFDF
#define MASK_SYN_DAC			   0xFFF7

#define MASK_DATA_SF			      0xFFBF
#define MASK_SYN_SF			        0xFF7F
#define MASK_CLOCK_SF			      0xFFDF
#define MASK_SF_0			          0xFF1F
#define MASK_DPRS			          0xFDFF
#define MASK_IDENTIFIER         0x0300 
#define LABEL_R                 0x1A   
#define LABEL_T                 0x72


// Индикация светодиодов
#define ALL_LEDS_OFF             0x0  // Светодиоды не горят
#define ALL_LEDS_ON              0xF  // Cветодиоды горят
#define SHOW_FC_IS_OK            0xE  // Радиокомпаc исправен
#define SHOW_ARINC_FAIL          0xD  // Отказ канала ввода
#define SHOW_ANTENNA_FAIL        0xB  // Отказ антенных цепей
#define SHOW_RECEIVER_FAIL       0x7  // Отказ приемника
//--------------------------------------------------------------------

void Init_Chassis(void);
void Set_Register_State(uint8_t state);
void Set_DATA_DAC(void);
void Clear_DATA_DAC(void);
void Set_CLOCK_DAC(void);
void Clear_CLOCK_DAC(void);
void Set_SYN_DAC(void);
void Clear_SYN_DAC(void);
void Set_DAC_Angle(void);
void Clear_CLOCK_SF(void);
void Set_CLOCK_SF(void);
void Clear_SYN_SF(void);
void Set_SYN_SF(void);
void Set_DATA_SF(void);
void Clear_DATA_SF(void);

void Set_Mode_Ant(void);
void Set_Mode_Compass(void);
void Set_Check_Receiver(void);
void Set_Check_Antenna(void);
void Clear_Check_RA(void);

uint8_t Check_SF(void);
uint8_t Check_SD(void);
void Clear_Pulse_Fr(void);
void Set_Pulse_Fr(void);
void Pulse_Fr(void);
void Delay_SD(void);

void Set_DPRS(void);
void Set_BPRS(void);

uint16_t Read_ID(void);
uint16_t Read_Antenna(void);
uint16_t Read_CQM(void);
uint16_t Read_ARINC_AB(void);
void Set_Telephon(void);
void Set_Telegraph(void);

void Set_Test_A(void);
void Set_Test_B(void);
void Reset_Test_A(void);
void Reset_Test_B(void);
void Set_Test_AB(void);
void Reset_Test_AB(void);

void Clear_FR_ROLL(void);
void Set_FR_ROLL(void);


void Clear_Bit_D18D19(uint16_t _bit);
void Set_Bit_D18D19(uint16_t _bit);

void Check_Switch_ARC(void);
//--------------------------------------------------------------------



#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

