#include "Init_APK25.h"
#include "APK25_QBUS.h"

//--------------------------------------------------------------------

extern uint16_t errors;
PORT_InitTypeDef  Port;

//--------------------------------------------------------------------
void Set_QBUS_Input(void);
void Set_QBUS_Output(void);
void QBUS_Output_AD(uint16_t AD);
uint16_t QBUS_Input_AD(void);
void QBUS_Set_Ctrl(uint16_t pin);
void QBUS_Reset_Ctrl(uint16_t pin);
void Mode_QBUS(uint8_t mode);
//--------------------------------------------------------------------
void  Init_QBUS(void)
{
    // Конфигурация контактов управления шиной QBUS 
	  // Сигналы READ_QBUS и WRITE_QBUS 
	  Port.PORT_Pin = READ_QBUS | WRITE_QBUS;
	  Port.PORT_FUNC = PORT_FUNC_PORT;
	  Port.PORT_MODE = PORT_MODE_DIGITAL;
	  Port.PORT_OE = PORT_OE_OUT;
	  Port.PORT_SPEED = PORT_SPEED_FAST;
	  PORT_Init(MDR_PORTF, &Port);
	  // Выходные сигналы
	  Port.PORT_Pin    = SYNC_SA | DATA_READ | DATA_WRITE;  
	  Port.PORT_OE     = PORT_OE_OUT;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;
	  Port.PORT_FUNC   = PORT_FUNC_PORT;
	  Port.PORT_SPEED  = PORT_SPEED_FAST;
	  PORT_Init(MDR_PORTD, &Port);
	  // Входные сигналы
	  Port.PORT_Pin    = ANSWER;
	  Port.PORT_OE     = PORT_OE_IN;
    Port.PORT_FUNC   = PORT_FUNC_PORT;
    Port.PORT_SPEED  = PORT_OUTPUT_OFF;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;  
	  PORT_Init(MDR_PORTD, &Port);
	  // Выключение QBUS
	  Mode_QBUS(SET_QBUS_OFF);
	  // Установка сигналов управления в 1
	  QBUS_Set_Ctrl(SYNC_SA);
	  QBUS_Set_Ctrl(DATA_WRITE);
	  QBUS_Set_Ctrl(DATA_READ);
	  // Конфигурация цины на передачу данных		 
    Set_QBUS_Output();
	  // Все сигналы шины AD в 1
	  QBUS_Output_AD(0xFFFF);
}
//--------------------------------------------------------------------
void Device_Work(void)
{
    PORT_ResetBits(MDR_PORTB, GENERATOR_0);	
		PORT_ResetBits(MDR_PORTB, GENERATOR_1);	
}
//--------------------------------------------------------------------
void Mode_QBUS(uint8_t mode)
{
    switch(mode)
	  {
	      case SET_QBUS_OFF:
		        PORT_SetBits(MDR_PORTF, READ_QBUS);
			      PORT_SetBits(MDR_PORTF, WRITE_QBUS);
			      break;
		    case SET_QBUS_READ:
			      PORT_ResetBits(MDR_PORTF, READ_QBUS);
			      PORT_SetBits(MDR_PORTF, WRITE_QBUS);
			      break;
		    case SET_QBUS_WRITE:
		        PORT_SetBits(MDR_PORTF, READ_QBUS);
			      PORT_ResetBits(MDR_PORTF, WRITE_QBUS);
			      break;
	  }
}
//--------------------------------------------------------------------
void Set_QBUS_Input(void)
{
    // PORT C - на вход
	  Port.PORT_Pin    = MASK_AD_PC;
    Port.PORT_OE     = PORT_OE_IN;
    Port.PORT_FUNC   = PORT_FUNC_PORT;
    Port.PORT_SPEED  = PORT_OUTPUT_OFF;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;
	  PORT_Init(MDR_PORTC, &Port);
	  // PORT A - на вход
	  Port.PORT_Pin    = MASK_AD_PA;
    Port.PORT_OE     = PORT_OE_IN;
    Port.PORT_FUNC   = PORT_FUNC_PORT;
    Port.PORT_SPEED  = PORT_OUTPUT_OFF;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;
	  PORT_Init(MDR_PORTA, &Port);
}
//--------------------------------------------------------------------
void Set_QBUS_Output(void)
{
    // PORT C - на выход 
	  Port.PORT_Pin    = MASK_AD_PC;
	  Port.PORT_OE     = PORT_OE_OUT;
    Port.PORT_FUNC   = PORT_FUNC_PORT;
    Port.PORT_SPEED  = PORT_SPEED_FAST;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;  
	  PORT_Init(MDR_PORTC, &Port);
	  // PORT A - на выход 
	  Port.PORT_Pin    = MASK_AD_PA;
	  Port.PORT_OE     = PORT_OE_OUT;
    Port.PORT_FUNC   = PORT_FUNC_PORT;
    Port.PORT_SPEED  = PORT_SPEED_FAST;
	  Port.PORT_MODE   = PORT_MODE_DIGITAL;  
	  PORT_Init(MDR_PORTA, &Port);
}
//--------------------------------------------------------------------
void QBUS_Output_AD(uint16_t AD)
{
    PORT_Write(MDR_PORTC, AD&(0xFFFF-BIT3-BIT4));
    /*
	  if(AD&BIT0)
	      PORT_SetBits(MDR_PORTC, AD_02);
    if(AD&BIT2)
	      PORT_SetBits(MDR_PORTC, AD_00);
	  */
	  if(AD&BIT3)
	      PORT_SetBits(MDR_PORTA, AD_03);
	  else
	      PORT_ResetBits(MDR_PORTA, AD_03);
	  if(AD&BIT4)
	      PORT_SetBits(MDR_PORTA, AD_04);
	  else
	      PORT_ResetBits(MDR_PORTA, AD_04);
	  Delay(DELAY_SA);
}
//--------------------------------------------------------------------
uint16_t QBUS_Input_AD(void)
{
    uint16_t ret = 0;

    ret = PORT_ReadInputData(MDR_PORTC)&0xFFFF; 
	  
	  // Сброс BIT3,BIT4
	  ret &= (0xFFFF - BIT3 - BIT4);
	  /*
	  if(PORT_ReadInputDataBit(MDR_PORTC, BIT0)==Bit_SET)
	      ret |= BIT2;
	  if(PORT_ReadInputDataBit(MDR_PORTC, BIT2)==Bit_SET)
	      ret |= BIT0;
	  */ 
	  if(PORT_ReadInputDataBit(MDR_PORTA, BIT3)==Bit_SET)
	      ret |= BIT3;
	  if(PORT_ReadInputDataBit(MDR_PORTA, BIT4)==Bit_SET)
	      ret |= BIT4;
	  return ret;	
}
//--------------------------------------------------------------------
void QBUS_Set_Ctrl(uint16_t pin)
{
    PORT_SetBits(MDR_PORTD, pin);
	  Delay(DELAY_SA);
}
//--------------------------------------------------------------------
void QBUS_Reset_Ctrl(uint16_t pin)
{
	  PORT_ResetBits(MDR_PORTD, pin);
	  Delay(DELAY_SA);
}
//--------------------------------------------------------------------
uint8_t Get_ANSWER(void)
{
    return PORT_ReadInputDataBit(MDR_PORTD, ANSWER);
}
//--------------------------------------------------------------------
void Write_QBUS(uint16_t addr, uint16_t value)
{
    // uint16_t k = 0;
	  
   	Set_QBUS_Output(); 
	  // Установка адреса
	  QBUS_Output_AD(addr); 
	  // МПП на выход
	  Mode_QBUS(SET_QBUS_WRITE);
    // Отрицательный фронт SYNC
	  QBUS_Reset_Ctrl(SYNC_SA);
    // Установка данных 
	  QBUS_Output_AD(value);
    // Режим записи 
	  QBUS_Reset_Ctrl(DATA_WRITE);
	  /*
	  errors &= (0xFFFF - ERROR_WRITE_QBUS);
	  // Ожидание 0 ANSWER
	  for(k = 0; k< VALUE_TEST; k++)
	  {
        if(Get_ANSWER()==Bit_SET)
		    break;
		    Delay(DELAY_SA);
	  }
	  // Превышение времени ожидания QBUS
	  if(k == VALUE_TEST) 
	       errors |= ERROR_WRITE_QBUS; 
		*/
		Delay(DELAY_SA);
	  // Режим записи 
	  QBUS_Set_Ctrl(DATA_WRITE);
	  // Положительный фронт SYNC
	  QBUS_Set_Ctrl(SYNC_SA);
	  // Отключение МПП
	  Mode_QBUS(SET_QBUS_OFF);
}
//--------------------------------------------------------------------
uint16_t Read_QBUS(uint16_t addr)
{
    uint16_t ret = 0;
	  uint16_t k = 0;
    
	  Set_QBUS_Output(); 
	  // Вывод адреса
	  QBUS_Output_AD(addr); 
	  // МПП на выход
	  Mode_QBUS(SET_QBUS_WRITE);
	  // Отрицательный фронт  SYNC
	  QBUS_Reset_Ctrl(SYNC_SA);
	  // Режим чтения
	  QBUS_Reset_Ctrl(DATA_READ);
	  // Шина QBUS на ввод
	  Set_QBUS_Input();
	  // Режим чтения
	  Mode_QBUS(SET_QBUS_READ);
	  /*
	  errors &= (0xFFFF - ERROR_READ_QBUS);
	  // Ожидание 0  "ANSWER"
	  for(k = 0; k< VALUE_TEST; k++)
	  {
        if(Get_ANSWER()==Bit_SET)
		       break;
		    Delay(DELAY_SA);
	  }
		*/
		Delay(DELAY_SA);
	  // Чтение данных
    ret = QBUS_Input_AD();
	  // Превышение времени ожидания QBUS
	  if(k == VALUE_TEST) 
	      errors |= ERROR_READ_QBUS;
	  // Отключение МПП
	  Mode_QBUS(SET_QBUS_OFF);	 
    // Режим вывода
	  Set_QBUS_Output();
	  // Режим чтения 
	  QBUS_Set_Ctrl(DATA_READ);
    // Положительный фронт  SYNC
	  QBUS_Set_Ctrl(SYNC_SA);
	  return ret;
}
//--------------------------------------------------------------------







