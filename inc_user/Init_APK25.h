/**
  ******************************************************************************
  * @file    Init_APK25.h
  * @author  Haemorrhoid Application Team
  * @version V1.0.0
  * @date    22/03/2017
  * @brief   Этот файл содержит функции инициализации переферии котроллера
  ******************************************************************************
   * FILE Init_APK25.h
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INIT_APK25_H
#define __INIT_APK25_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_lib.h"

#define ALL_PORTS_CLK   (RST_CLK_PCLK_PORTA | RST_CLK_PCLK_PORTB | \
                         RST_CLK_PCLK_PORTC | RST_CLK_PCLK_PORTD | \
                         RST_CLK_PCLK_PORTE | RST_CLK_PCLK_PORTF)



#define BIT0   0x00000001
#define BIT1   0x00000002
#define BIT2   0x00000004
#define BIT3   0x00000008
#define BIT4   0x00000010
#define BIT5   0x00000020
#define BIT6   0x00000040
#define BIT7   0x00000080
#define BIT8   0x00000100
#define BIT9   0x00000200
#define BIT10  0x00000400
#define BIT11  0x00000800
#define BIT12  0x00001000
#define BIT13  0x00002000
#define BIT14  0x00004000
#define BIT15  0x00008000
#define BIT16  0x00010000
#define BIT17  0x00020000
#define BIT18  0x00040000
#define BIT19  0x00080000
#define BIT20  0x00100000
#define BIT21  0x00200000
#define BIT22  0x00400000
#define BIT23  0x00800000
#define BIT24  0x01000000
#define BIT25  0x02000000
#define BIT26  0x04000000
#define BIT27  0x08000000
#define BIT28  0x10000000
#define BIT29  0x20000000
#define BIT30  0x40000000
#define BIT31  0x80000000

#define PERIOD_250_MS      600
#define PERIOD_LOST        500//2000
#define PERIOD_MORE_LOST   2050
#define PERIOD_54_MS       128

#define TIME_ONE_SEC       2600 // 2000    //2360
#define TIME_DELAY_FC      3000 
#define TIME_DELAY_PP      6000 
#define TIME_100_MS        236
#define TIME_10_MS         24
#define TIME_HALF_SEC      1300//1180 
#define TIME_TWO_SEC       3000//4800
#define TIME_FOUR_SEC      19200 
#define TIME_DELAY         3000 


// Mistakes of the execution  operation
#define ERROR_READ_QBUS      BIT0 
#define ERROR_WRITE_QBUS     BIT1


#define SIZE_BUFFER_BYTE  0x0400
#define SIZE_BUFFER_WORD  0x0200           // 512 отсчетов
#define BUFFER_1          0x8000
#define BIT_SWITCH_SIN    PORT_Pin_5
#define BIT_WRITE_ARRAY   PORT_Pin_15

typedef union
{
	volatile uint8_t  _byte[SIZE_BUFFER_BYTE];
	volatile uint16_t _word[SIZE_BUFFER_WORD]; //  Буфер отсчетов АЦП
} DATA;


void ClockConfig(void); // enable
void Delay(unsigned int val); // enable
void PortsConfig(void); // enable
void InitARINC(void); // enable
void Timer1Config(unsigned char n_ch); // enable
void Timer2Config(void);
void ConfigUART1(void); // enable
void SysTickInit(void); // enable

void ClockConfig_Low(void);

void Init_ADC(void);

void Arinc429TxConfig(uint32_t ARINC429T_CLK);
void Arinc429RxConfig(uint32_t ARINC429R_CLK, uint16_t ID0, uint8_t Label_R);
void Arinc429RxConfig_Ch(uint32_t ARINC429R_CLK, uint8_t Label_R);
void Arinc429RxConfig_Ch_Disable(uint32_t ARINC429R_CLK);

#ifdef __cplusplus
} // extern "C" block end
#endif

#endif /* __MDR32F9Qx_PORT_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE MDR32F9Qx_port.h */
