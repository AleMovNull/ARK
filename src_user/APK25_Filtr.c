#include "Init_APK25.h"
#include "APK25_Filtr.h"
#include "APK25_Angle.h"
#include "APK25_FC.h"
#include "APK25_SF.h"
#include "math.h"


const int16_t kLP[65] =
  { 0,   0,   0,   1,   2,   3,   5,   6,    9,  12,  // 0...9		
   15,  19,  24,  29,  35,  42,  49,  57,   65,  74,  // 10...19		
   83,  92, 101, 110, 118, 126, 134, 140,  146, 150,  // 20...29		
  153, 155, 156, 155, 153, 150, 146, 140,  134, 126,  // 30...39		
  118, 110, 101,  92,  83,  74,  65,  57,   49,  42,  // 40...49		
   35,  29,  24,  19,  15,  12,   9,   6,    5,   3,  // 50...59		
    2,   1,   0,   0,   0  // 60...64
	};

uint8_t load = 1;                         // Загрузка фильтра
extern uint32_t flags;                    // Регистр состояний
extern uint8_t  state_working;            // Режим работы АРК-25
extern uint16_t ram_022064;               // Содержит код угла курса	

//extern int delta_ch_0;
//extern int delta_ch_180;	
extern uint8_t flags_angle;	
	
	
	
	
//--------------------------------------------------------------------
//                Фильтрация сигналов АЦП 
//--------------------------------------------------------------------

uint16_t load_f = 0;        // Загрузка ФНЧ оценок угла
uint16_t load_fv = 0;       // Загрузка ФНЧ оценок угла	(вспомогательный ФНЧ)

int16_t angle_n[65];
int16_t angle_nv[65];
int16_t code_angle_180;     // Код угла с поворотом на 180 градусов 
int16_t code_angle_0;       // Код угла с поворотом на 0   градусов

int16_t mass512[512];
int16_t mass256[256];
int16_t loaded_Old = 0;


//-------------------------------------------------------------------------------
// ФНЧ Чебышева для оценок угла, порядок фильтра 64; 
// частота дискретизации сигнала fd=20Гц;  
// Fstop1=0.1Гц; 
// Множитель коэффиициентов фильтра 4096 (>>12)
//--------------------------------------------------------------------------------
void L_PASS_Cheb_64_N(int i)
{
    int64_t	angle_f = 0;
	  uint8_t j = 0;  
	  //int16_t delta = 0;
		
    if(i == 0)
	      code_angle_0 = ram_022064;
		
  
	  if(load_f>=64)
	  {
		    angle_n[64] = code_angle_0;
			  
        //Фильтрация
        for(j=3; j<62; j++)			     
		    angle_f += kLP[j]*angle_n[64-j];
			  angle_f >>= 12;
			  code_angle_0 = angle_f;	 							 
			
			  // Cдвиг тела фильтра	
        for(j=0; j<64; j++)
	      angle_n[j] = angle_n[j+1];
						  
			  /*
			  // Определение средней разницы 
			  delta_ch_0 = 0;
			  for(j=0; j<32; j++)
			  {
					  delta = angle_n[j]-angle_n[64 - j];
					  if(delta < 0)
							  delta *= -1;
					  delta_ch_0 +=  delta;
				}
				flags_angle |= DISPERS;
				*/				
	  }
	  else
	  {
	      // Первоначальное заполнение массива фильтра
		    if(state_working == COMPASS)
		    { 
            angle_n[load_f] = code_angle_0;
					  load_f++;
		    }
		    else
		    {
		        load_f = 64;
		        for(j=0; j<load_f; j++)
		        angle_n[j] = code_angle_0; //CODE_135_DEGREES;		        
		    }
	  }
		
}
//-------------------------------------------------------------------------------
//  Вспомогательный фильтр, устраняющий переколебания на угле 180 град.
//-------------------------------------------------------------------------------
void L_PASS_Cheb_64_NV(int i)
{
    int64_t	angle_fv = 0;
	  uint8_t j = 0;
	  //int16_t delta = 0;
	  
	  if(i == 0)
	      code_angle_180 = ram_022064;
	  // Поворот кода угла на 180 градусов
	  if(code_angle_180 == 0)
		{
		    code_angle_180 = 0xF000;	  
    }
    else
    {			
	      if(code_angle_180 > 0)
		        code_angle_180 -= 4096;
	 	    else
		        code_angle_180 += 4096;
		    code_angle_180 *= -1;
		} 				
		
	  
		if(load_fv>=64) // Первоначальная загрузка фильтра
	  {
		    angle_nv[64] = code_angle_180;		
			  
			  //Фильтрация
        for(j=3; j<62; j++)			     
		    angle_fv += kLP[j]*angle_nv[64-j];
			  angle_fv >>= 12;
			  code_angle_180 = angle_fv;	 
						
        // сдвиг тела фильтра	 
        for(j=0; j<64; j++)
	      angle_nv[j] = angle_nv[j+1];	  			
			
			  /*  
			  // Определение средней разницы 
			  delta_ch_180 = 0;
			  for(j=0; j<32; j++)
			  {
					  delta = angle_nv[j]-angle_nv[64 - j];
					  if(delta < 0)
							  delta *= -1;
					  delta_ch_180 +=  delta;
				}	
        */
        		
	  }
	  else
	  {
	      // Первоначальное заполнение массива фильтра
		    angle_nv[load_fv] = code_angle_180;
	    	load_fv++;
		}	
		
		// Второй поворот кода угла на 180 градусов
		if(code_angle_180 == 0)
		{
		    code_angle_180 = 0xF000;	  
    }
    else
    {			
	      if(code_angle_180 > 0)
		        code_angle_180 -= 4096;
	 	    else
		        code_angle_180 += 4096;
		    code_angle_180 *= -1;
		} 
    
		// Компенсация преобразования
		//code_angle_180 += 29;
		
}

//--------------------------------------------------------------------------------	
int16_t Bisector_angle1(int16_t a0,int16_t a1)
{
	uint16_t a00, a10; // f0._uword
	int16_t a11;

	a00 = a0;
	a10 = a1;
  
	a00 ^= a10;
	a11 = a00;   // f2._word

	if (a11 >= 0)
	  {
		  a0 += a1;
		  a0 /= 2;
		  return a0;
	  }
	
	a11 = a1;	

	if (a11 < 0) a11 *= -1;
	
	if (a0 < 0)  a11 -= a0;
	else  a11 += a0;
	
	if (a11 < 4096)
	  {
		  a0 += a1;
		  a0 /= 2;
		  return a0;
	  }
	
	if (a0 >= 0)
	  {
		 if (a1 < 0)
			 a1 += 0x2000;
	  }
	 else
		a0 += 0x2000;
	 a1 += a0;
	 a1 /= 2;
	
	if (a1 >= 4096)
		a1 -= 0x2000;
	else
	  {
		 if(a1 < (-4096))
			 a1 += 0x2000;
	  }
  a0 = a1;
  return a0;	
}
//--------------------------------------------------------------------------------
void Old_PASS_512(void)
{
	  
    uint16_t i;
	 
	
	
    if(loaded_Old == 0)
		{
		    
			  switch(state_working)
        {
	          case FUNC_CONTROL:
							  if(flags&SET_CHECK_ANTENNA)
								    for(i=0; i <512; i++)
								    mass512[i] = CODE_135_DEGREES;	
								else
                    for(i=0; i <512; i++)
								    mass512[i] = CODE_225_DEGREES;                        											
								break;
						case CONTROL:
							  for(i=0; i <512; i++)
						    mass512[i] = CODE_135_DEGREES;									  
								break;
		        case COMPASS:
			          for(i=0; i <512; i++)
		            mass512[i] = ram_022064;
		            break;
				}	
				loaded_Old = 0xFF;
		}
    else
		{		  
		    for(i=0; i <511; i++)
		    mass512[i] = mass512[i+1];
		    mass512[511] = ram_022064;
			
			  for(i=0; i<256; i++)
		    mass256[i] = Bisector_angle1(mass512[i],mass512[i+256]);
		   
			  for(i=0; i<128; i++)
		    mass256[i] = Bisector_angle1(mass256[i],mass256[i+128]);
		   
        for(i=0; i<64; i++)
		    mass256[i] = Bisector_angle1(mass256[i],mass256[i+64]);
		   

		    for(i=0; i<32; i++)
			  mass256[i] = Bisector_angle1(mass256[i],mass256[i+32]);
		   
        for(i=0; i<16; i++)
			  mass256[i] = Bisector_angle1(mass256[i],mass256[i+16]);
			
		    for(i=0; i<8; i++)
			  mass256[i] = Bisector_angle1(mass256[i],mass256[i+8]);
			
        for(i=0; i<4; i++)
			  mass256[i] = Bisector_angle1(mass256[i],mass256[i+4]);
			
		    for(i=0; i<2; i++)
			  mass256[i] = Bisector_angle1(mass256[i],mass256[i+2]);
			
        ram_022064 = Bisector_angle1(mass256[0],mass256[1]);
				
		}		 
		 
}
//--------------------------------------------------------------------------------
