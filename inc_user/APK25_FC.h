//  Этот файл содержит функции режима Функциональный контроль
//--------------------------------------------------------------------
#ifndef __APK25_FC_H
#define __APK25_FC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MDR1986VE1T.h"
//--------------------------------------------------------------------

#define PHASE_START             0x00	
#define PHASE_TEST_CPU          0x01 
#define PHASE_TEST_ARINC        0x02
#define PHASE_GET_FREQUENCY     0x03	
#define PHASE_TEST_HOG_SF       0x04
#define PHASE_TEST_HOG_SD       0x05	
#define PHASE_MEASURE           0x06 
#define PHASE_DELAY_007616      0x07	
#define PHASE_DELAY_000540      0x08
#define PHASE_TEST_ARINC_A      0x09
#define PHASE_TEST_ARINC_B      0x0A 	
#define PHASE_AFTER_TEST_AB	    0x0B
#define PHASE_DO_MASSAGE_FRQ    0x0C
#define PHASE_GET_TEST_FRQ      0x0D
#define PHASE_SET_TEST_FRQ      0x0E
#define PHASE_GET_CUR_FRQ       0x0F
#define PHASE_SELECT_CHECK_BACK 0x10
#define PHASE_ANGLE_COMPARE_ANT 0x11
#define PHASE_SET_ONLY_FRQ      0x12
#define PHASE_CUR_HOG_SF        0x13
#define PHASE_NOISE_HOG_SD      0x14
#define PHASE_ANGLE_COMPARE_REC 0x15
#define PHASE_OPER_TEST_OK      0x16
#define PHASE_OUT_225           0x17




#define PHASE_UNUSED            0xFF	
	
#define CODE_225_DEGREES        0xF400   // Код 225 градусов	
#define CODE_135_DEGREES        0x0C00   // Код 135 градусов
#define CODE_90_DEGREES         2048
#define CODE_5_DEGREES          0x0072   // Код 5 градусов	
#define CODE_2_DEGREES          45 
#define CODE_3_DEGREES          70 
#define CODE_4_DEGREES          90
#define CODE_7_DEGREES          160
#define CODE_10_DEGREES         228 
#define CODE_20_DEGREES         456 
#define CODE_75_DEGREES         1700


#define ANGLE_INVALID           0x00
#define ANGLE_VALID             0x01	

#define VALUE_SUBRANGE          6        // Число поддиапазонов 





//--------------------------------------------------------------------
void Mode_Functional_Control(void);	
void Mode_Operative_Control(void);
void Is_Functional_Control_Enable(void);
void Measure_Show_Angle(void);
void Tugle_Frequency_OK(void);
//--------------------------------------------------------------------





#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

