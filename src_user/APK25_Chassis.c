#include "APK25_Chassis.h"

#include "APK25_TransRec.h"
#include "APK25_Angle.h"
#include "APK25_QBUS.h"
#include "Init_APK25.h"
#include "MDR32F9Qx_arinc429t.h"
#include "MDR32F9Qx_arinc429r.h"

//--------------------------------------------------------------------

extern uint16_t errors;

uint32_t errors_1;

uint16_t r_word;
uint16_t receiver_D27D30; 
uint16_t register_D18D19; 
uint16_t register_D28D29; 
extern uint16_t receiver_D26; 
extern uint32_t code_DAC;
extern uint32_t In_Word_ARINC_0;
uint16_t ID = 0;          // Идентификатор приемника
extern uint16_t correct_quadrant_mistake; // enable
extern int16_t antenna_h; // enable
uint16_t arinc_AB = 0;  // enable
extern uint32_t flags;         // Регистр состояний 
uint16_t last = 0;
uint16_t last_AB = 0xFFFF;
uint16_t fc_on = 0;





//--------------------------------------------------------------------
// 0 - Есть сигнал "Захват СД"
// 1 - Нет сигнала "Захват СД"
uint8_t Check_SD(void)
{
   if(Read_QBUS(D27D30)& HOG_SD) 
	     return HOG_SD_RESET; 
	 else
       return HOG_SD_SET; 		 
}
//--------------------------------------------------------------------------
//   Опрос сигналов управления внешнего интерфейса
//--------------------------------------------------------------------------
void Check_Switch_ARC(void)
{
    	
	  receiver_D27D30 =  Read_QBUS(D27D30);
	  receiver_D26 = Read_QBUS(D26);
	  
	  if(last != receiver_D27D30)
	  {
		    // Коррекция квадрантной ошибки
			  correct_quadrant_mistake = Read_CQM();
			  // Положение антены
			  antenna_h = Read_Antenna();
			  // Идентификатор приемника 
			  ID = Read_ID();
			  // Канал А/B приемника
			  arinc_AB = Read_ARINC_AB();
			  if(last_AB!=(receiver_D27D30&SELECT_AB)) 
				{	
			      // Конфигурация приемного канала ARINC429		
			      Arinc429RxConfig_Ch(ARINC429R_CLK_12_5_KHz, LABEL_R);					  
				  //	  In_Word_ARINC_0 = 0;
					last_AB = receiver_D27D30&SELECT_AB;
			  }
				last = receiver_D27D30;
		}
		
}
//--------------------------------------------------------------------
//         Чтение положения антенны
//--------------------------------------------------------------------
uint16_t Read_Antenna(void)
{
    return receiver_D27D30 & LOC_ANTENNA;
}
//--------------------------------------------------------------------
//         Чтение идентификатора приемника
//--------------------------------------------------------------------
uint16_t Read_ID(void)
{
    return receiver_D27D30 & MASK_IDENTIFIER;
}
//--------------------------------------------------------------------
//         Чтение значения коррекции квадрантной ошибки
//--------------------------------------------------------------------
uint16_t Read_CQM(void)
{
    return receiver_D27D30 & BITS_CORR_QUADRANT_ERR;
}
//--------------------------------------------------------------------
//          Чтение выбора канала А/B
//--------------------------------------------------------------------
uint16_t Read_ARINC_AB(void)
{
  	return receiver_D27D30 & SELECT_AB;
}
//--------------------------------------------------------------------
//          Установка режима Телефон
//--------------------------------------------------------------------
void Set_Telephon(void)
{
    flags &= (0xFFFFFFFF - SET_MODE_TLGR);
    flags |= SET_MODE_TLPH;
	  register_D18D19 &= (0xFFFF - TLPH_TLGR);
    Write_QBUS(D18D19,register_D18D19);	
}
//--------------------------------------------------------------------
//          Установка режима Tелеграф
//--------------------------------------------------------------------
void Set_Telegraph(void)
{
    flags &= (0xFFFFFFFF - SET_MODE_TLPH);
    flags |= SET_MODE_TLGR;
	  register_D18D19 |= TLPH_TLGR;
    Write_QBUS(D18D19,register_D18D19);	
}
//--------------------------------------------------------------------
//          Установить сигнал TEST_A  
//--------------------------------------------------------------------
void Set_Test_A(void)
{
	  register_D28D29 |= SET_RT;
	  Write_QBUS(D28D29,register_D28D29);	
}	
//--------------------------------------------------------------------
//          Сбросить сигнал TEST_A  
//--------------------------------------------------------------------
void Reset_Test_A(void)
{
	  register_D28D29 &= (0xFFFF - SET_RT);
	  Write_QBUS(D28D29,register_D28D29);	
}	
//--------------------------------------------------------------------
//          Установить сигнал TEST_B  
//--------------------------------------------------------------------
void Set_Test_B(void)
{
	  register_D28D29 |= INPUT_WORD_DISABLE;
	  Write_QBUS(D28D29,register_D28D29);	
}	
//--------------------------------------------------------------------
//          Сбросить сигнал TEST_B  
//--------------------------------------------------------------------
void Reset_Test_B(void)
{
	  register_D28D29 &= (0xFFFF - INPUT_WORD_DISABLE);
	  Write_QBUS(D28D29,register_D28D29);	
}
//--------------------------------------------------------------------
//          Сбросить сигналы TEST_B  и TEST_A 
//--------------------------------------------------------------------
void Reset_Test_AB(void)
{
	  register_D28D29 &= (0xFFFF - INPUT_WORD_DISABLE - SET_RT);
	  Write_QBUS(D28D29,register_D28D29);	
}	
//--------------------------------------------------------------------
//          Установить сигналы TEST_B  и TEST_A 
//--------------------------------------------------------------------
void Set_Test_AB(void)
{
	  register_D28D29 |= INPUT_WORD_DISABLE;
	  register_D28D29 |= SET_RT;
	  Write_QBUS(D28D29,register_D28D29);	
}	
//--------------------------------------------------------------------
//          Запись регистра статуса  
//--------------------------------------------------------------------
void Set_Register_State(uint8_t state)
{
	register_D18D19 &= MASK_REGISTR_STATE;
	if(state&BIT0)
	    register_D18D19 |= REGISTR_STATE_0;
	if(state&BIT1)
	    register_D18D19 |= REGISTR_STATE_1;	
	if(state&BIT2)
	    register_D18D19 |= REGISTR_STATE_2;	 
	if(state&BIT3)
	    register_D18D19 |= REGISTR_STATE_3;	
  Write_QBUS(D18D19,register_D18D19);	
}
//--------------------------------------------------------------------
void Init_Chassis(void)
{
  
	  //Чтение данных с микросхем D27D30
	  receiver_D27D30 =  Read_QBUS(D27D30);
    // Получение информации	
	  ID = Read_ID();
	  correct_quadrant_mistake = Read_CQM();
	  antenna_h = Read_Antenna();
	  arinc_AB = Read_ARINC_AB();
	
	  register_D28D29 = 0x0081; // 1
	  Write_QBUS(D28D29,register_D28D29);
	
	  // Предварительная загрузка курса
	  Get_Code_Arrow_Device(2048);	
    // Указатель на 90 градусов
    Set_DAC_Angle(); 					
	  // Формируем сообщение КУРС для передатчика
	  Word_Angl_WR( ARINC_ANGLE_NO_DATA,       // Матрица состояния
	                2048);                     // Курсовой угол
	  // Предварительная загрузка частоты
    Set_Start_Message_Freq_Status();
   	
	  register_D18D19 = 0x010F;
	  Write_QBUS(D18D19,register_D18D19); 
}

//--------------------------------------------------------------------
void Set_Mode_Ant(void)
{
  register_D18D19 |= COMPASS_ANTENNA;
  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_Mode_Compass(void)
{
  register_D18D19 &= (0xFFFF - COMPASS_ANTENNA);
  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_Check_Receiver(void)
{
   register_D28D29 |= CHECK_RECEIVER;
   register_D28D29 &= (0xFFFF - CHECK_ANTENNA);
   flags &= (0xFFFFFFFF - SET_CHECK_ANTENNA);
   flags |= SET_CHECK_RECEIVER;
   Write_QBUS(D28D29,register_D28D29);	
}
//--------------------------------------------------------------------
void Clear_Check_RA(void)
{
   register_D28D29 &= (0xFFFF - CHECK_RECEIVER - CHECK_ANTENNA);
   flags &= (0xFFFFFFFF - SET_CHECK_ANTENNA - SET_CHECK_RECEIVER); 
   Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Set_Check_Antenna(void)
{
   register_D28D29 |= CHECK_ANTENNA;
   register_D28D29 &= (0xFFFF - CHECK_RECEIVER);
   flags &= (0xFFFFFFFF - SET_CHECK_RECEIVER);
   flags |= SET_CHECK_ANTENNA;
   Write_QBUS(D28D29,register_D28D29);
}

//--------------------------------------------------------------------
// 0 - Есть сигнал "Захват частоты"
// 1 - Нет сигнала "Захват частоты"
uint8_t Check_SF(void)
{  
   if(Read_QBUS(D27D30)& HOG_SF) 
		   return HOG_SF_RESET;
   else 
		   return HOG_SF_SET;       
}
//--------------------------------------------------------------------
void Clear_FR_ROLL(void)
{
   register_D18D19 &= (0xFFFF - FR_ROLL);
   Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_FR_ROLL(void)
{
   register_D18D19 |= FR_ROLL;
   Write_QBUS(D18D19,register_D18D19);
}


//--------------------------------------------------------------------
void Clear_Pulse_Fr(void)
{
   register_D18D19 &= (0xFFFF - PULSE_F);
   Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_Pulse_Fr(void)
{
   register_D18D19 |= PULSE_F;
   Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Pulse_Fr(void)
{
     Clear_Pulse_Fr();
	 Delay(500);
	 Set_Pulse_Fr();
}
//--------------------------------------------------------------------
void Delay_SD(void)
{
  uint32_t i_32 = 0;
	for(i_32=0; i_32<50; i_32++)	 //25
  Delay(200000);
 
}
//--------------------------------------------------------------------
void Set_DAC_Angle(void)
{
    uint8_t i = 0;
	  // Write 20-bit code of the heading angle
    Set_SYN_DAC();	 // Reset of the signal "Synchronizing DAC" (inversion signal)
	  for(i = 0; i<20; i++)
    {
        if(code_DAC&0x0001)
            Set_DATA_DAC();
        else
            Clear_DATA_DAC();
        code_DAC >>= 1;
		    // Pulse of the tact
        Set_CLOCK_DAC();
        Clear_CLOCK_DAC();
    }
	  // Write in parallel register
    Clear_SYN_DAC();
    Set_SYN_DAC();		
}
//--------------------------------------------------------------------
void Clear_Bit_D18D19(uint16_t _bit)
{
    register_D18D19 &= (0xFFFF - _bit);
	  Write_QBUS(D18D19,register_D18D19);
}//--------------------------------------------------------------------
void Set_Bit_D18D19(uint16_t _bit)
{
	  register_D18D19 |= _bit;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Clear_SYN_DAC(void)
{
    register_D18D19 &= MASK_SYN_DAC;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_SYN_DAC(void)
{
    register_D18D19 |= SYN_DAC;
    Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Clear_CLOCK_DAC(void)
{
    register_D18D19 &= MASK_CLOCK_DAC;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_CLOCK_DAC(void)
{
    register_D18D19 |= CLOCK_DAC;
    Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_DATA_DAC(void)
{
    register_D18D19 |= DATA_DAC;
    Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Clear_DATA_DAC(void)
{
    register_D18D19 &= MASK_DATA_DAC;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
//                    Cell SCH
//--------------------------------------------------------------------
void Clear_CLOCK_SF(void)
{
    register_D28D29 &= MASK_CLOCK_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Set_CLOCK_SF(void)
{
    register_D28D29 |= CLOCK_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Clear_SYN_SF(void)
{
    register_D28D29 &= MASK_SYN_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Set_SYN_SF(void)
{
    register_D28D29 |= SYN_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Set_DATA_SF(void)
{
    register_D28D29 |= DATA_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Clear_DATA_SF(void)
{
    register_D28D29 &= MASK_DATA_SF;
	  Write_QBUS(D28D29,register_D28D29);
}
//--------------------------------------------------------------------
void Set_DPRS(void)
{
    register_D18D19 &= MASK_DPRS;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
void Set_BPRS(void)
{
    register_D18D19 |= DPRS;
	  Write_QBUS(D18D19,register_D18D19);
}
//--------------------------------------------------------------------
