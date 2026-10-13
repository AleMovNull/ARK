//  Этот файл содержит функции работы с шиной QBUS
//--------------------------------------------------------------------
#ifndef __APK25_QBUS_H
#define __APK25_QBUS_H

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
//--------------------------------------------------------------------
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_lib.h"
#include "MDR1986VE1T.h"
#include "MDR32F9Qx_port.h"




#define SET_QBUS_OFF      0x00
#define SET_QBUS_READ     0x01
#define SET_QBUS_WRITE    0x02

  
// Hardware interface
// Bus AD 
#define  AD_00        PORT_Pin_0   	 // PORT C
#define  AD_01        PORT_Pin_1	   // PORT C
#define  AD_02        PORT_Pin_2	   // PORT C
#define  AD_03        PORT_Pin_3	   // PORT A 
#define  AD_04        PORT_Pin_4	   // PORT A
#define  AD_05        PORT_Pin_5 	   // PORT C
#define  AD_06        PORT_Pin_6	   // PORT C
#define  AD_07        PORT_Pin_7 	   // PORT C
#define  AD_08        PORT_Pin_8	   // PORT C
#define  AD_09        PORT_Pin_9	   // PORT C
#define  AD_10        PORT_Pin_10 	 // PORT C
#define  AD_11        PORT_Pin_11	   // PORT C
#define  AD_12        PORT_Pin_12	   // PORT C
#define  AD_13        PORT_Pin_13	   // PORT C
#define  AD_14        PORT_Pin_14 	 // PORT C
#define  AD_15        PORT_Pin_15	   // PORT C

#define  MASK_AD_PC	  0xFFE7
#define  MASK_AD_PA   0x0018

// Bus of control
#define SYNC_SA       PORT_Pin_0     // PORT D
#define DATA_READ     PORT_Pin_1   	 // PORT D
#define DATA_WRITE    PORT_Pin_2   	 // PORT D
#define ANSWER        PORT_Pin_3  	 // PORT D

#define READ_QBUS     PORT_Pin_8     // PORT F
#define WRITE_QBUS    PORT_Pin_7     // PORT F

#define GENERATOR_0   PORT_Pin_10    // PORT B
#define GENERATOR_1   PORT_Pin_11    // PORT B 

    

#define D18D19        0xF006		     // Address register
#define D28D29        0xF008
#define D27D30        D18D19
#define D26           D28D29

#define DELAY_SA      1
#define VALUE_TEST	  1000


//--------------------------------------------------------------------
void  Init_QBUS(void);
void Write_QBUS(uint16_t addr, uint16_t value);
uint16_t Read_QBUS(uint16_t addr);
void Device_Work(void);
//--------------------------------------------------------------------



#ifdef __cplusplus
} // extern "C" block end
#endif

#endif 

