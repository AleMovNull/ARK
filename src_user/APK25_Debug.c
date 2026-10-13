#include "Init_APK25.h"
#include "APK25_Debug.h"
#include "APK25_QBUS.h"
#include "APK25_Chassis.h"
#include "APK25_Angle.h"
#include "MDR32F9Qx_uart.h"
#include "APK25_SF.h"

//--------------------------------------------------------------------

extern uint16_t errors;
//extern AM angle_mass;
extern uint16_t errors_1;

extern uint16_t val_angle_gr;
extern _REGISTR R0;
extern _REGISTR R1;
extern uint32_t code_frequency;
extern uint8_t  range_frequency;
extern uint16_t frequency;


extern DATA switch_sin_0;
extern DATA switch_sin_1;

uint8_t ii = 0;
uint8_t debug_size_packet = 0;
uint16_t word_r = 0;
uint16_t word_a = 0;

uint32_t word_rr = 0;
uint8_t  en_trans = 0;



_DEBUGGER_DATA packet;
//--------------------------------------------------------------------
void Check_Leds(void);

//--------------------------------------------------------------------
void Receive_Debug_Packet(uint8_t byte)
{
    if(debug_size_packet==0)
	    packet.len = 2; 
	packet._byte[debug_size_packet] = byte;
	debug_size_packet++;
}
//--------------------------------------------------------------------
void Work_Debugger(void)
{
      
    if(debug_size_packet < packet.len) return;


	  switch(packet.command)
	  {
	      case CHECK_LEDS:
		        Check_Leds();
			      break;
        case GET_SIN_0:
					  //UART_SendDataArray(MDR_UART1, switch_sin_0._byte,SIZE_BUFFER_BYTE);
				    debug_size_packet = 0;
            break;					
        case GET_SIN_1:
					  //UART_SendDataArray(MDR_UART1, switch_sin_1._byte,SIZE_BUFFER_BYTE);
				    debug_size_packet = 0;
            break; 
				case GET_MASS_ANGLE:
					  //UART_SendDataArray(MDR_UART1, angle_mass._byte,sizeof(angle_mass._byte));
				    debug_size_packet = 0;
            break; 
					
				


				
			 case CHECK_AD:
				    word_a = packet.addr;
             
			      //*
			        
			         word_r = PORT_ReadInputData(MDR_PORTB); //word_a;
			 
			      //word_r = PORT_ReadInputData(MDR_PORTB);
			        //word_r = PORT_ReadInputDataBit(MDR_PORTA, PORT_Pin_8);
              UART_SendWord(MDR_UART1, word_r);
              	 
			      //*/
			 
			 /*
		       if((enable_sin_0)&&(enable_sin_1))
					    {       
					      packet.command = 7; // Command control "data"
								packet.len = 6;
								word_r = packet.len;
								word_r = (word_r << 8);
								word_r |= packet.command;								
								UART_SendWord(MDR_UART1, word_r);
								
								word_r = switch_sin_0._word[word_a];
								UART_SendWord(MDR_UART1, word_r);
								
								word_r = switch_sin_1._word[word_a];
								UART_SendWord(MDR_UART1, word_r);
                //enable_sin_0 = 0;	enable_sin_1 = 0;							
						  }
					 else
					    {
							  packet.command = 8; // Command control "no data"
								packet.len = 2;
								word_r = packet.len;
								word_r = (word_r << 8);
								word_r |= packet.command;								
								UART_SendWord(MDR_UART1, word_r);
						  }
							*/
			     debug_size_packet = 0;
			 
			 break;


		 case CHECK_UART:
		      word_rr = packet.addr;
			    word_rr <<= 16;
			    word_rr |= packet.data;
			  
			    errors_1 = 0;
			    Set_Mode_Compass(); // Режим "Компас"
			    Set_Fr_SF(word_rr); // Запись кода частоты в СЧ, выбор диапазона			  			  			  
			    Set_Check_Receiver(); // Проверка приемника	
          Pulse_Fr();		 
				  Delay_SD();                    
          Check_SD();				
		      code_frequency = errors_1; 
				 	UART_SendWWord(MDR_UART1, code_frequency);
			    debug_size_packet = 0;			 				     
			 	 
		 break;
		 case CHECK_RC:
			    errors_1 = 0;
		      Set_Mode_Compass();
		      Set_Fr_SF(0x5A0000);  // f= 168 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();		      
      		if(Check_SF()) errors_1 |= ERROR_HOG_SF_1;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_1);		 
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC1;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC1);
		      Clear_Check_RA();
		 		 
		       
		      Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
          if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN1;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN1);
		      Clear_Check_RA();
		     
		 
		      Set_Fr_SF(0xC00000);  // f= 300 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SF()) errors_1 |= ERROR_HOG_SF_2;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_2);
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC2;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC2);
		      Clear_Check_RA();
			
					Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN2;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN2);
		      Clear_Check_RA();				
      				
					Set_Fr_SF(0x1158000);  // f= 456 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SF()) errors_1 |= ERROR_HOG_SF_3;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_3);
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC3;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC3);
		      Clear_Check_RA();
				
					Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN3;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN3);
		      Clear_Check_RA();				
   				
					Set_Fr_SF(0x1C20000);  // f= 708 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SF()) errors_1 |= ERROR_HOG_SF_4;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_4);
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC4;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC4);
		      Clear_Check_RA();
			
					Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN4;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN4);
		      Clear_Check_RA();
				
				
					Set_Fr_SF(0x4200000);  // f= 1080 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SF()) errors_1 |= ERROR_HOG_SF_5;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_5);
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC5;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC5);
		      Clear_Check_RA();
			
					
					Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN5;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN5);
		      Clear_Check_RA();
				
      				
					Set_Fr_SF(0x54D8000);  // f= 1536 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SF()) errors_1 |= ERROR_HOG_SF_6;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_6);
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC6;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC6);
		      Clear_Check_RA();
			
					
					Set_Check_Antenna();
		      Pulse_Fr();
		      Delay_SD();
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_AN6;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_AN6);
		      Clear_Check_RA();
       
					code_frequency = errors_1; 
				 	UART_SendWWord(MDR_UART1, code_frequency);
			    debug_size_packet = 0;		 
		   break;
			 
			 case CHECK_ANGLE:
				   if(en_trans == 0x00) 
			         en_trans = 0x01;
					 else
						   en_trans = 0x00; 
			 
			 
			 
			 
				   //en_trans = 0x01;
				 /*
				  errors_1 = 0; 
				  word_rr = packet.addr;
			    word_rr <<= 16;
			    word_rr |= packet.data;
          Set_Mode_Ant();			    // Mode Ant
				  //Set_Mode_Compass();   // Mode Compass
			    Set_Fr_SF(word_rr);     // Set f in SF
			    
			    Pulse_Fr();             // Clk f in SF
		      Delay_SD();
			    if(Check_SF()) errors_1 |= 1;
			    if(Check_SD()) errors_1 |= 2;
			    if(errors_1 != 0)		 
					   { 
							 UART_SendWWord(MDR_UART1, errors_1);
			         debug_size_packet = 0;
							 return;
						 }
			    else
					  {
							errors_1 = 16;
							UART_SendWWord(MDR_UART1, errors_1);
						}
			 */
			    /*
		      Set_Fr_SF(0x5A0000);  // f= 168 kHz 
		      Set_Check_Receiver();
		      Pulse_Fr();
		      Delay_SD();		      
      		if(Check_SF()) errors_1 |= ERROR_HOG_SF_1;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SF_1);		 
		      if(Check_SD()) errors_1 |= ERROR_HOG_SD_RC1;
		      else errors_1 &= (0xFFFFFFFF - ERROR_HOG_SD_RC1);
		      Clear_Check_RA();
			    */
			 
				 
			 break;
		 /*
		 case WRITE_REGISTER:
			  Write_QBUS(packet.addr,packet.data);     
		      debug_size_packet = 0;
		 	 break;
		 case READ_MPP:
		     word_r = Read_QBUS(packet.addr);
			 debug_size_packet = 0;
			 UART_SendWord(MDR_UART1, word_r);
			 break;
	     case SET_ANGLE:
		     debug_size_packet = 0;
			 //val_angle_gr = packet.addr;
             R0._uword = packet.addr; 
			 R1._uword = packet.data;
			 // Получние кода управления стрелочным прибором из угла курса в градусах
			 //Get_Code_Arrow_Device_Gr();
			 // Упаковка кода управления
			 Packing_Code_Arrow_Device();
			 // Ввод кода управления
			 Set_DAC_Angle();
		     break;
			 */		  	 		
	}
	
}
//--------------------------------------------------------------------
void Check_Leds(void)
{
    /*
	PORT_SetBits(MDR_PORTD, PORT_Pin_8);
	Delay(5000000);
	PORT_ResetBits(MDR_PORTD, PORT_Pin_8);
	Delay(5000000);

	PORT_SetBits(MDR_PORTD, PORT_Pin_10);
	Delay(5000000);
	PORT_ResetBits(MDR_PORTD, PORT_Pin_10);
	debug_size_packet = 0;
	*/
}
//--------------------------------------------------------------------