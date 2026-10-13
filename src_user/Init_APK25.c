
// Процедуры и функции инициализации переферии контроллера

#include "MDR1986VE1T.h"
#include "MDR32F9Qx_rst_clk.h"
#include "MDR32F9Qx_bkp.h"
#include "MDR32F9Qx_eeprom.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_timer.h"
#include "MDR32F9Qx_ebc.h"
#include "MDR32F9Qx_adc.h"
#include "Init_APK25.h"
#include "MDR32F9Qx_uart.h"
#include "MDR32F9Qx_arinc429t.h"
#include "MDR32F9Qx_arinc429r.h"
#include "APK25_Chassis.h"





extern uint16_t ID; // enable
extern uint16_t arinc_AB; // enable


uint16_t errors = 0;


//--------------------------------------------------------------------------------------------------------------------
void ClockConfig_Low(void)
{
   
	
	RST_CLK_WarmDeInit();
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
	while(RST_CLK_HSEstatus() != SUCCESS){}
	
	// Включение тактирования всех портов, таймера 1, UART1, приемника и педатчика ARINC429, АЦП 
	RST_CLK_PCLKcmd(ALL_PORTS_CLK | RST_CLK_PCLK_TIMER1 | RST_CLK_PCLK_UART1 | RST_CLK_PCLK_ADC    
	                | RST_CLK_PCLK_ARINC429R | RST_CLK_PCLK_ARINC429T, ENABLE);
   
	MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
}



//--------------------------------------------------------------------------------------------------------------------
void ClockConfig(void)
{
	uint32_t temp;
	
	RST_CLK_WarmDeInit();
	RST_CLK_HSEconfig(RST_CLK_HSE_ON);
	while(RST_CLK_HSEstatus() != SUCCESS){}
	
	RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLLsrcHSEdiv1, RST_CLK_CPU_PLLmul10);	//PLL frequency: 10 * 8 MHz = 80 MHz  
  
	RST_CLK_CPU_PLLcmd(ENABLE);		//PLL enable
   	if (RST_CLK_CPU_PLLstatus() == ERROR){ while (1); }
		
	RST_CLK_PCLKcmd(RST_CLK_PCLK_EEPROM|RST_CLK_PCLK_BKP, ENABLE);

	EEPROM_SetLatency (EEPROM_Latency_3);	//EEPROM_Latency_0 because core frequency below 100 MHz
	
	
	temp = MDR_BKP->REG_0E;
	MDR_BKP->REG_0E = ((temp & 0xFFFFFFC0) | ((5 << BKP_REG_0E_SELECTRI_Pos)|(5 << BKP_REG_0E_LOW_Pos)));	
	RST_CLK_PCLKcmd(RST_CLK_PCLK_EEPROM|RST_CLK_PCLK_BKP, DISABLE);
	while(RST_CLK_CPU_PLLstatus() != SUCCESS){}
	
		
		
	RST_CLK_CPU_PLLuse(ENABLE);

	RST_CLK_CPUclkPrescaler(RST_CLK_CPUclkDIV1);	

	RST_CLK_CPUclkSelection(RST_CLK_CPUclkCPU_C3);	//Core frequency 80 MHz
	// Включение тактирования всех портов, таймера 1, UART1, приемника и педатчика ARINC429, АЦП 
	RST_CLK_PCLKcmd(ALL_PORTS_CLK | RST_CLK_PCLK_TIMER1 | RST_CLK_PCLK_UART1 | RST_CLK_PCLK_ADC    
	                | RST_CLK_PCLK_ARINC429R | RST_CLK_PCLK_ARINC429T, ENABLE);
   
	MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
}
//-----------------------------------------------------------------------------------------------------------------
void Init_ADC(void)
{
    MDR_ADC->ADC1_CFG |= 0xFEC21021;
	  MDR_ADC->ADC1_STATUS = 0;		  
}	
//-----------------------------------------------------------------------------------------------------------------
void PortsConfig(void)
{
	PORT_InitTypeDef	Port;
	
	PORT_DeInit(MDR_PORTA);
	PORT_DeInit(MDR_PORTB);
	PORT_DeInit(MDR_PORTD);
	PORT_DeInit(MDR_PORTC);
	PORT_DeInit(MDR_PORTE);	
	PORT_DeInit(MDR_PORTF);	

	// PORTA
	// PA5 - Сигнал переключения SIN
	// PA7..PA15 - Адресная шина АЦП 9 бит  (ADDR_ADC0...ADDR_ADC8)                                          
  PORT_StructInit(&Port);	
  Port.PORT_Pin   = (PORT_Pin_5 
					   | PORT_Pin_7 | PORT_Pin_8 | PORT_Pin_9 |PORT_Pin_10 | PORT_Pin_11        
             | PORT_Pin_12 |  PORT_Pin_13 | PORT_Pin_14 | PORT_Pin_15);
  Port.PORT_OE    = PORT_OE_IN;
  Port.PORT_FUNC  = PORT_FUNC_PORT;
  Port.PORT_SPEED = PORT_OUTPUT_OFF;
	Port.PORT_MODE  = PORT_MODE_DIGITAL;  
	PORT_Init(MDR_PORTA, &Port);
  
  // PORTB
	// PB4..PB15 - Шина данных АЦП 12 бит  (ADC_0...ADC_11)                                          
  PORT_StructInit(&Port);	
  Port.PORT_Pin   = (PORT_Pin_4 | PORT_Pin_5 | PORT_Pin_6 |  PORT_Pin_7 |
					   PORT_Pin_8 | PORT_Pin_9 | PORT_Pin_10 |  PORT_Pin_11 |
					   PORT_Pin_12 | PORT_Pin_13 | PORT_Pin_14 | PORT_Pin_15);
  Port.PORT_OE    = PORT_OE_IN;
  Port.PORT_FUNC  = PORT_FUNC_PORT;
  Port.PORT_SPEED = PORT_OUTPUT_OFF;
	Port.PORT_MODE  = PORT_MODE_DIGITAL;  
	PORT_Init(MDR_PORTB, &Port);	
	
	// PORTB
  // PB0,PB1 - первый приемный канал ARINC429
	// PB3,PB4 - второй приемный канал ARINC429
	PORT_StructInit(&Port);
	Port.PORT_Pin   = PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3; 
	Port.PORT_FUNC = PORT_FUNC_ALTER;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	Port.PORT_PD = PORT_PD_DRIVER;
	PORT_Init(MDR_PORTB, &Port);	
	
	////////////////////////////////////////////
	// PORTB
	// PB10,PB11 выход генерации для узла перезапуска
	PORT_StructInit(&Port);	
	Port.PORT_Pin = PORT_Pin_10 | PORT_Pin_11;
	Port.PORT_OE = PORT_OE_OUT;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	PORT_Init(MDR_PORTB, &Port);	
	/////////////////////////////////////////////
	
	
	

	// PORTC
	// Отладочный канал UART1
  Port.PORT_PULL_UP = PORT_PULL_UP_OFF;
	Port.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
	Port.PORT_PD_SHM = PORT_PD_SHM_OFF;
	Port.PORT_PD = PORT_PD_DRIVER;
	Port.PORT_GFEN = PORT_GFEN_OFF;
	Port.PORT_FUNC = PORT_FUNC_MAIN;
	Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	// UART1_TX
	Port.PORT_OE = PORT_OE_OUT;
	Port.PORT_Pin = PORT_Pin_3;
	PORT_Init(MDR_PORTC, &Port);
	// UART1_RX
	Port.PORT_OE = PORT_OE_IN;
	Port.PORT_Pin = PORT_Pin_4;
	PORT_Init(MDR_PORTC, &Port);
  	
	
	// PORTD
	// PD13,PD14 передатчик второго канала ARINC429
	PORT_StructInit(&Port);	
	Port.PORT_Pin = PORT_Pin_13 | PORT_Pin_14;
	Port.PORT_FUNC = PORT_FUNC_ALTER;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	Port.PORT_PD = PORT_PD_DRIVER;
	PORT_Init(MDR_PORTD, &Port);	

  // PORTD
	// PD9,PD10,PD11 - входы АЦП ADC2, ADC3, ADC4
	PORT_StructInit(&Port);	
	Port.PORT_Pin = PORT_Pin_9 | PORT_Pin_10 | PORT_Pin_11;
	Port.PORT_OE = PORT_OE_IN;
	Port.PORT_MODE = PORT_MODE_ANALOG;
	PORT_Init(MDR_PORTD, &Port);  				
  	
	// PORTE
	// PE3 - контакт захвата сигнала ПРЕОБРАЗОВАНИЕ АЦП ВЫПОЛНЕНО  (ADC_DONE0) для таймера 1 
 	PORT_StructInit(&Port);
	Port.PORT_Pin =  PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5;	 
	Port.PORT_FUNC = PORT_FUNC_MAIN;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_PULL_DOWN = PORT_PULL_DOWN_ON; // OFF
	PORT_Init(MDR_PORTE, &Port);
		
	// PORT F
	// PF13,PF14 -  передатчик первого канала ARINC429
	PORT_StructInit(&Port);
	Port.PORT_Pin = PORT_Pin_13 | PORT_Pin_14;
	Port.PORT_FUNC = PORT_FUNC_MAIN;
	Port.PORT_OE = PORT_OE_OUT;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	Port.PORT_PD = PORT_PD_DRIVER;
	PORT_Init(MDR_PORTF, &Port);	
	
	// PORT F
	// PF7, PF8 - сигналы READ_QBUS и WRITE_QBUS
	PORT_StructInit(&Port);
	Port.PORT_Pin = PORT_Pin_7 | PORT_Pin_8;
	Port.PORT_FUNC = PORT_FUNC_PORT;
	Port.PORT_MODE = PORT_MODE_DIGITAL;
	Port.PORT_OE = PORT_OE_OUT;
	Port.PORT_SPEED = PORT_SPEED_FAST;
	PORT_Init(MDR_PORTF, &Port);  
	
	errors = 0;
}


//--------------------------------------------------------------------------------------------------------------------
void Arinc429TxConfig(uint32_t ARINC429T_CLK)
{
  ARINC429T_InitChannelTypeDef	ARINC429T_InitChanelStruct;
	
	ARINC429T_DeInit();

	// Настройка передатчика для частоты 80 MГц 
	//ARINC429T_BRG_Init(80);
	ARINC429T_BRG_Init(72);  // Частота 13.9

	// Ввод настроек передатчиков 1 и 2 каналов ARINC429  
	ARINC429T_InitChanelStruct.ARINC429T_CLK = ARINC429T_CLK; //ARINC429T_CLK_100_KHz; //ARINC429T_CLK_12_5_KHz;
	ARINC429T_InitChanelStruct.ARINC429T_DIV = 0;
	ARINC429T_InitChanelStruct.ARINC429T_EN_PAR = ENABLE;
	ARINC429T_InitChanelStruct.ARINC429T_ODD = ARINC429T_ODD_ADD_TO_ODD;
  
	ARINC429T_ChannelInit(ARINC429T_CHANNEL1, &ARINC429T_InitChanelStruct);
	// Включение передатчика  1
	ARINC429T_ChannelCmd(ARINC429T_CHANNEL1, ENABLE);
		
	ARINC429T_ChannelInit(ARINC429T_CHANNEL2, &ARINC429T_InitChanelStruct); 
	// Включение передатчика  2 	
	ARINC429T_ChannelCmd(ARINC429T_CHANNEL2, ENABLE);		 
}
//--------------------------------------------------------------------------------------------------------------------
void Arinc429RxConfig(uint32_t ARINC429R_CLK, uint16_t ID0, uint8_t Label_R)
{	
	ARINC429R_InitChannelTypeDef ARINC429R_InitChannelStruct;
	
	ARINC429R_DeInit();

	// Настройка приемника для частоты 80 MГц
	ARINC429R_BRG_Init(80);	
  
	// Ввод настроек примников 1 и 2 каналов ARINC429 
	ARINC429R_InitChannelStruct.ARINC429R_CLK = ARINC429R_CLK; //ARINC429R_CLK_100_KHz; //ARINC429R_CLK_12_5_KHz;	
	//ARINC429R_InitChannelStruct.ARINC429R_SD = DISABLE;	      // DISABLE of the decoding bit given 9 and 10 in message
	ARINC429R_InitChannelStruct.ARINC429R_SD = ENABLE;	  // DISABLE of the decoding bit given 9 and 10 in message
	//ARINC429R_InitChannelStruct.ARINC429R_LB = DISABLE;	  // DISABLE of the decoding the address (mark) of the message
	ARINC429R_InitChannelStruct.ARINC429R_LB = ENABLE;    // Filtering the address (mark) for channe l   ARINC429R_CONTROL2_LB_EN1;	
	 
	if(ID0 & ID_DEVICE_0)	
	    ARINC429R_InitChannelStruct.ARINC429R_SDI1 = SET;
	else
	    ARINC429R_InitChannelStruct.ARINC429R_SDI1 = RESET;
	if(ID0 & ID_DEVICE_1)
	    ARINC429R_InitChannelStruct.ARINC429R_SDI2 = SET;
	else
	    ARINC429R_InitChannelStruct.ARINC429R_SDI2 = RESET;	
	
	ARINC429R_InitChannelStruct.ARINC429R_DIV = 0;
	
	ARINC429R_ChannelInit(ARINC429R_CHANNEL1, &ARINC429R_InitChannelStruct);
	ARINC429R_ChannelInit(ARINC429R_CHANNEL2, &ARINC429R_InitChannelStruct);	  

	ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, ENABLE);
	// Enabling the receiving channe2	
	ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, ENABLE);		 
	
	
	ARINC429R_SetChannel(ARINC429R_CHANNEL1);
	ARINC429R_SetLabel(Label_R); //Label CHANNEL1
	//Set Label Channe2	
	ARINC429R_SetChannel(ARINC429R_CHANNEL2); 
	ARINC429R_SetLabel(Label_R); //Label CHANNEL2
		 		
	// ENABLE of the interruption on receiving the message and discovered mistake
	ARINC429R_ITConfig((ARINC429R_IT_INT_DR | ARINC429R_IT_INT_ER), ENABLE);
  NVIC_EnableIRQ(ARINC429R_IRQn);		
}
//------------------------------------------------------------------------------------------
void Arinc429RxConfig_Ch_Disable(uint32_t ARINC429R_CLK)
{	
	ARINC429R_InitChannelTypeDef ARINC429R_InitChannelStruct;
	
	ARINC429R_DeInit();

	// Настройка приемника для частоты 80 MГц
	ARINC429R_BRG_Init(80);	
  
	// Ввод настроек примников 1 и 2 каналов ARINC429 
	ARINC429R_InitChannelStruct.ARINC429R_CLK = ARINC429R_CLK; //ARINC429R_CLK_100_KHz; //ARINC429R_CLK_12_5_KHz;	
	ARINC429R_InitChannelStruct.ARINC429R_SD = DISABLE;	      // DISABLE of the decoding bit given 9 and 10 in message
	ARINC429R_InitChannelStruct.ARINC429R_LB = DISABLE;	  // DISABLE of the decoding the address (mark) of the message
	  	
	ARINC429R_InitChannelStruct.ARINC429R_DIV = 0;
	
	// Настройка на прием выбранного канала
	switch(arinc_AB)
	{	
		case CHANNEL_A:	
	      // Блокировка канала В
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, DISABLE);
		    // Инициализация канала А
		    ARINC429R_ChannelInit(ARINC429R_CHANNEL1, &ARINC429R_InitChannelStruct);
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, ENABLE);
		    break;
		case CHANNEL_B:
		    // Блокировка канала A
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, DISABLE);
		    // Инициализация канала А	
		    ARINC429R_ChannelInit(ARINC429R_CHANNEL2, &ARINC429R_InitChannelStruct);	  
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, ENABLE);
		    break;
	}
  // Разрешение прерываний по приему сообщения и ошибке при приеме
	ARINC429R_ITConfig((ARINC429R_IT_INT_DR | ARINC429R_IT_INT_ER), ENABLE);
  NVIC_EnableIRQ(ARINC429R_IRQn);		
}


void Arinc429RxConfig_Ch(uint32_t ARINC429R_CLK, uint8_t Label_R)
{	
		
	ARINC429R_InitChannelTypeDef ARINC429R_InitChannelStruct;
	
	ARINC429R_DeInit();

	// Настройка приемника для частоты 80 MГц
	ARINC429R_BRG_Init(80);	
  
	// Ввод настроек примников 1 и 2 каналов ARINC429 
	ARINC429R_InitChannelStruct.ARINC429R_CLK = ARINC429R_CLK; //ARINC429R_CLK_100_KHz; //ARINC429R_CLK_12_5_KHz;	
	ARINC429R_InitChannelStruct.ARINC429R_SD = DISABLE;	      // DISABLE of the decoding bit given 9 and 10 in message
	//ARINC429R_InitChannelStruct.ARINC429R_SD = ENABLE;	  // DISABLE of the decoding bit given 9 and 10 in message
	
	//ARINC429R_InitChannelStruct.ARINC429R_LB = DISABLE;	  // DISABLE of the decoding the address (mark) of the message
	
	ARINC429R_InitChannelStruct.ARINC429R_LB = ENABLE;    // Filtering the address (mark) for channe l   ARINC429R_CONTROL2_LB_EN1;	
	
  ARINC429R_InitChannelStruct.ARINC429R_DIV = 0;
	
	// Настройка на прием выбранного канала
	switch(arinc_AB)
	{	
		case CHANNEL_A:	
	      // Блокировка канала В
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, DISABLE);
		    // Инициализация канала А
		    ARINC429R_ChannelInit(ARINC429R_CHANNEL1, &ARINC429R_InitChannelStruct);
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, ENABLE);
		    // Ввод метки (адреса) канала А
		    ARINC429R_SetChannel(ARINC429R_CHANNEL1);
	      ARINC429R_SetLabel(Label_R); 
	      break;
		case CHANNEL_B:
		    // Блокировка канала A
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL1, DISABLE);
		    // Инициализация канала А	
		    ARINC429R_ChannelInit(ARINC429R_CHANNEL2, &ARINC429R_InitChannelStruct);	  
		    ARINC429R_ChannelCmd(ARINC429R_CHANNEL2, ENABLE);
		    // Ввод метки (адреса) канала B	
	      ARINC429R_SetChannel(ARINC429R_CHANNEL2); 
	      ARINC429R_SetLabel(Label_R); 
		    break;
	}
  // Разрешение прерываний по приему сообщения и ошибке при приеме
	ARINC429R_ITConfig((ARINC429R_IT_INT_DR | ARINC429R_IT_INT_ER), ENABLE);
  NVIC_EnableIRQ(ARINC429R_IRQn);		
}

//-------------------------------------------------------------------------------------
void Timer1Config(unsigned char n_ch)
{
	TIMER_CntInitTypeDef	Timer_Count;
	TIMER_ChnInitTypeDef	Timer_Channel;
	
	TIMER_BRGInit(MDR_TIMER1, TIMER_HCLKdiv1);
	TIMER_DeInit(MDR_TIMER1);
	
	TIMER_CntStructInit(&Timer_Count);
	TIMER_ChnStructInit(&Timer_Channel);
	
	Timer_Count.TIMER_CounterDirection = TIMER_CntDir_Up;
	Timer_Count.TIMER_ARR_UpdateMode = TIMER_ARR_Update_Immediately;
	Timer_Count.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
	Timer_Count.TIMER_EventSource = TIMER_EvSrc_None;
	Timer_Count.TIMER_Prescaler = 0;
	Timer_Count.TIMER_Period = 0xFFFFFFFF;
	Timer_Count.TIMER_IniCounter = 0;
	
	TIMER_CntInit(MDR_TIMER1, &Timer_Count);
	
	switch(n_ch)
	{
	    case 1: Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL1; break;
		  case 2: Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL2; break;
		  case 3: Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL3; break;
		  case 4: Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL4; break;
		 default: Timer_Channel.TIMER_CH_Number = TIMER_CHANNEL1; break;
	}
	Timer_Channel.TIMER_CH_Mode = TIMER_CH_MODE_CAPTURE;
	// Timer_Channel.TIMER_CH_EventSource = TIMER_CH_EvSrc_PE;
	Timer_Channel.TIMER_CH_EventSource = TIMER_CH_EvSrc_NE;
	Timer_Channel.TIMER_CH_CCR1_Ena = DISABLE;
	
	TIMER_ChnInit(MDR_TIMER1, &Timer_Channel);
	
	switch(n_ch)
	{
	     case 1:  TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH1 , ENABLE); break; // |TIMER_STATUS_CNT_ZERO
		   case 2:  TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH2 , ENABLE); break;
		   case 3:  TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH3 , ENABLE); break;
		   case 4:  TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH4 , ENABLE); break;
		  default: TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CCR_CAP_CH1 , ENABLE); break;
	}
	TIMER_Cmd(MDR_TIMER1, ENABLE);
}
//-------------------------------------------------------------------------------------

void ConfigUART1(void) 
{
   
	UART_InitTypeDef UARTInitStructure;
   
    UART_BRGInit(MDR_UART1, UART_HCLKdiv1);

    UART_DeInit(MDR_UART1);
	  // Disable interruption from UART1
    // NVIC_DisableIRQ(UART1_IRQn);
    // Enable interruption from UART1
	  NVIC_EnableIRQ(UART1_IRQn);
	  // Config UART1
	  UARTInitStructure.UART_BaudRate         = 115200;
    UARTInitStructure.UART_WordLength       = UART_WordLength8b;
    UARTInitStructure.UART_StopBits         = UART_StopBits1;
    UARTInitStructure.UART_Parity           = UART_Parity_No;
    UARTInitStructure.UART_FIFOMode         = UART_FIFO_OFF;
    UARTInitStructure.UART_HardwareFlowControl     = UART_HardwareFlowControl_RXE | \
                                                   UART_HardwareFlowControl_TXE;
	  UART_Init (MDR_UART1,&UARTInitStructure);
    
    // Enable interruption receiver
	  UART_ITConfig(MDR_UART1, UART_IT_RX, ENABLE);	
	  // On UART1
	  UART_Cmd(MDR_UART1,ENABLE);
}
//---------------------------------------------------------------------------------
void InitARINC(void)
{
    // Arinc429RxConfig(ARINC429R_CLK_12_5_KHz, ID, LABEL_R);
	  Arinc429RxConfig_Ch(ARINC429R_CLK_12_5_KHz, LABEL_R);
	  // Активированы два передатчика
    Arinc429TxConfig(ARINC429T_CLK_12_5_KHz);
}
//---------------------------------------------------------------------------------
//         Init SysTick
//---------------------------------------------------------------------------------
void SysTickInit()
{
	SysTick->LOAD = 0x004248C0;	//Pause 54,3 ms; (Pause 100 ms 0x007A1200) (HCLK = 80MHz) 
	SysTick->CTRL = 0x00000003;	//Enabel SysTick, Enable Interrupt!!!!!!
}
//---------------------------------------------------------------------------------
void Delay(unsigned int val)
{
  unsigned char b = 0;
	unsigned int i=0;
	for(i=0; i<val; i++)  b++; 
}

