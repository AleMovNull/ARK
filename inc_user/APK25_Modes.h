//  Этот файл содержит функции режимов работы АРК-25
//--------------------------------------------------------------------
#ifndef __APK25_MODES_H
#define __APK25_MODES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "MDR1986VE1T.h"
	


//--------------------------------------------------------------------
#define PHASE_MAIN    0x00
#define PHASE_DELAY   0x01 
#define PHASE_DONE    0x02	
	
	

//--------------------------------------------------------------------
void Work_Mode_APK(void);
//--------------------------------------------------------------------





#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

