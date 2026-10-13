//  Этот файл содержит функции работы с отладчиком
//--------------------------------------------------------------------
#ifndef __APK25_DEBUG_H
#define __APK25_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_lib.h"
#include "MDR1986VE1T.h"
#include "MDR32F9Qx_port.h"


#define SIZE_DEBUG_PACKET      6

#define CHECK_LEDS             0x00
#define WRITE_REGISTER         0x01
#define READ_MPP               0x02 
#define SET_ANGLE              0x03
#define CHECK_UART             0x04
#define CHECK_RC               0x05
#define CHECK_AD               0x06
#define	CHECK_ANGLE            0x07
#define GET_SIN_0              0x08
#define GET_SIN_1              0x09	
#define GET_MASS_ANGLE         0x0A



typedef union
{
	uint8_t _byte[SIZE_DEBUG_PACKET];
	struct
	{
	  uint8_t command;
		uint8_t len;
		uint16_t addr;
		uint16_t data;
		//uint16_t delya;
		//uint32_t wword;		
	};

	
} _DEBUGGER_DATA;


//--------------------------------------------------------------------



//--------------------------------------------------------------------
void Receive_Debug_Packet(uint8_t byte);
void Work_Debugger(void);


//--------------------------------------------------------------------



#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

