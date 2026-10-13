//  Этот файл содержит функции приемопередатчика ARINC429
//--------------------------------------------------------------------
#ifndef __APK25_TRANSREC_H
#define __APK25_TRANSREC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MDR1986VE1T.h"
	


//--------------------------------------------------------------------
#define MESSAGE_OK           0x00
#define MESSAGE_FAIL	     0x01 

#define ANTENNA_COMPASS	     0x00000800
#define TELEPHON_TELEGRAPH   0x00000400
#define BIT_CONTROL          0x40000000
#define UNUSED_MESSAGE	     0x60000000
#define NORMAL_WORK          0x00000000
#define DO_LAST_MESSAGE      0x20000000 
	
#define MASK_TETRADA         0x0000000F	
#define TELEPHON             0x00000000
#define TELEGRAPH            0x00000400	
	

#define CLEAR_INDICATOR      0x03FFC01A     // Стереть индикатор  

#define START_FRQ_MESSAGE    0x7FFF001A     // Стартовое сообщение Частота/Состояние
#define ZERO_POINT_ZERO      0x03FC001A     // Сообщение 0.0
#define ZERO_POINT_5         0x03FC201A     // Сообщение 0.5
#define TEST_FRQ_MESSAGE_A   0x0599A01A     // Тестовое сообщение для канала A
#define TEST_FRQ_MESSAGE_B   0x0266501A     // Тестовое сообщение для канала B

#define ERROR_NO                   0x0000   // Ошибок нет
#define ERROR_NOG_SD_NOES          0x0005   // Захват за помеху  
#define ERROR_NO_INFO_FROM_INPUT   0x0008   // Нет входной информации 
#define ERROR_CODE_FREQUENCY	     0x000A   // Ошибки кодирования частоты
#define ERROR_UP_RANGE_FREQUENCY   0x000B   // Выход за пределы диапазона сверху
#define ERROR_DOWN_RANGE_FREQUENCY 0x000B   // Выход за пределы диапазона снизу
#define ERROR_NOT_DATA_FROM_PULT   0x000C   // Отказ пульта управления 
#define ERROR_ENABLE_INPUT_MESSAGE 0x000D   // Отказ канала ввода(не используется)
#define ERROR_RETRANSMIT_MESSAGE_A 0x000E   // Ошибка в ретранслируемом сообщении канала A
#define ERROR_RETRANSMIT_MESSAGE_B 0x000F   // Ошибка в ретранслируемом сообщении канала B
#define ERROR_NO_HOG_SD            0x001A   // Нет сигнала Захват СД
#define ERROR_NO_HOG_FREQUENCY     0x001F   // Нет сигнала Захват СЧ
#define ERROR_FAIL_ANTENNA_0       CODE_FAIL_ANTENNA //0x0018   // Отказ антенных цепей
#define ERROR_FAIL_ANTENNA_1       CODE_FAIL_ANTENNA //0x0019   // Отказ антенных цепей
#define ERROR_CPU_0                0x0020   // Отказ контроллера 0(Не используется)
#define ERROR_CPU_1                0x0021   // Отказ контроллера 1(Не используется)
#define ERROR_CPU_2                0x0022   // Отказ контроллера 2(Не используется)	
#define ERROR_DATA                 0x0040   // Данные не верны

#define CODE_FAIL_ARINC            0x0001   // Ввод в сообщение отказа канала ввода
#define CODE_FAIL_ANTENNA          0x0002   // Ввод в сообщение отказа антенных цепей
#define CODE_FAIL_RECEIVER	       0x0003   // Ввод в сообщение отказа приемника
	

//--------------------------------------------------------------------
uint8_t Is_Input_Message_Fail(uint32_t message);
uint32_t Check_Freq_Range(uint32_t message);
void Set_Start_Message_Freq_Status(void);
void Word_Angl_WR( uint32_t state,          // Матрица состояния
	                 uint16_t code_angle);    // Курсовой угол
void Out_Word_Frequency_WR(uint32_t state); // Матрица состояния
uint8_t Out_Word_Frequency_WR_FC(void); 	                          
void Set_Test_Message_Freq_Status(void);
void Out_Word_Frequency_WR_OK(void);
void Is_Telephon(uint32_t message);
void Out_Word_Frequency_WR_5( uint8_t code); 

//--------------------------------------------------------------------





#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

