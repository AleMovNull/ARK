//  Этот файл содержит функции работы ячейки СЧ (синтезатора частоты)
//--------------------------------------------------------------------
#ifndef __APK25_SF_H
#define __APK25_SF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MDR1986VE1T.h"


//--------------------------------------------------------------------
#define MASK_RANGE_FR  0xFFF8 // enable
#define COMPASS      0x01
#define ANTENNA_TLF  0x02
#define ANTENNA_TLG  0x03
#define CONTROL      0x04
#define FUNC_CONTROL 0x05	
#define NOTHING      0x06	
	

#define MASK_STATUS_APK 3<<29
#define MASK_FREQUENCY 0x1FFFE000	
	
#define F_150_kHz    0x02A0
#define F_230_kHz	   0x0460
#define F_360_kHz	   0x06C0
#define F_560_kHz    0x0AC0
#define F_860_kHz	   0x10C0
#define F_1300_kHz	 0x2600 		
#define F_1750_kHz   0x2EA0
	
	

//--------------------------------------------------------------------
void Process_In_Word_ARINC(uint32_t In_Word_ARINC_);
void Determ_Frequency_Input(uint32_t In_Word_Frequency_);	
void Determ_Frequency(uint32_t In_Word_Frequency); 
void Determ_Range_Fr(void);
void Renov_Range_Fr(void);
void Transform_Code_Fr(void);
void Pulse_SYN_SF(void);
void Write_DATA_SF(void);
void Set_Fr_SF(uint32_t In_Word_Frequency);
void Set_Fr_SF_From_Code(void);
uint8_t Check_HOG_SF(void);
uint8_t Check_HOG_SD(void);
void Get_Frequency_kGz(void);
void Set_Frequency_Delta_12kGz(void);
void Get_Binary_Decimal_Code_Fr(void);

void SelectFrequency(void);

//--------------------------------------------------------------------





#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

