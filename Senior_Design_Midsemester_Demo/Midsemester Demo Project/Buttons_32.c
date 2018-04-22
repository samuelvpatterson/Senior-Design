//Button Logic 
//ec29992

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdio.h>
#include "Buttons_32.h"

#define BLUE      0x04
#define LEDS      (*((volatile uint32_t *)0x40025038))

double error;
double target_error;
int bit_number;
int trim_status;
int trim_algo; //0 for brute force, 1 from trim analysis


/*void Button_Init(void){
	SYSCTL_RCGCGPIO_R |=0x02; //Port B is being used
	while((SYSCTL_RCGCGPIO_R&0x02)==0){
	}; 
  GPIO_PORTB_PCTL_R &= ~0x000F0F00; //Normal GPIO
  GPIO_PORTB_AMSEL_R &= ~0x0F;   //Flips AMSEL disabling analog function
  GPIO_PORTB_DIR_R &= ~0x00;      //Dont have to set pullup since we are using external switches
  GPIO_PORTB_AFSEL_R &= ~0xFF;      //takes off unique port functions
  GPIO_PORTB_DEN_R |= 0x0F;        //Turn on DEN
	GPIO_PORTB_IS_R &= ~0xFF;     //all of them are edge sensitive
	GPIO_PORTB_IBE_R &= ~0xFF;   //not both edges
	GPIO_PORTB_IEV_R &= ~0xFF; //falling edge
	GPIO_PORTB_ICR_R = 0xFF; //clear flags0-7
	GPIO_PORTB_IM_R |= ~0xFF; //arm interrrupts
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFF0FFF) | 0x0000A000; //priority 2
	NVIC_EN0_R = 0x00000002;      // (h) enable interrupt 1 in NVIC
	//make sure to enable interrupts in main
	
}*/

void EdgeCounter_Init(void){                          
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F

  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC

}

double error = 0;
double target_error = 0;
int32_t bit_number = 14;
int32_t trim_status=0;
int32_t trim_algo = 0;
int button_debouncer[8]={0,0,0,0,0,0,0,0};

int error_return(void)
{
	return error;
}

int Bit_numbers_active(void)
{
	return bit_number;
}
	
int Trim_Algo(void)
{
	return trim_algo; //0 if brute force and 1 if trim algo
}

int Trim_algo_status(void)
{
	return trim_status; //0 if off and 1 if on
}



#define CYCLE 5
void Read_Input(void) 
{
	int32_t input = GPIO_PORTF_DATA_R;
	if((input&0x01)!=0)
	{
		button_debouncer[0]=(input&0x01)*CYCLE;
	}
	else if((input&0x02)!=0)
	{
		button_debouncer[1]=((input&0x02)>>1)*CYCLE;
	}
	else if((input&0x04)!=0)
	{
		button_debouncer[2]=((input&0x04)>>2)*CYCLE;
	}
	else if((input&0x08)!=0)
	{
		button_debouncer[3]=((input&0x08)>>3)*CYCLE;
	}
	else if((input&0x010)!=0)
	{
		button_debouncer[4]=((input&0x10)>>4)*CYCLE;
	}
	else if((input&0x20)!=0)
	{
		button_debouncer[5]=((input&0x20)>>5)*CYCLE;
	}
	else if((input&0x40)!=0)
	{
		button_debouncer[6]=((input&0x40)>>6)*CYCLE;
	}
	else if((input&0x80)!=0)
	{
		button_debouncer[7]=((input&0x80)>>7)*CYCLE;
	}
}

void Button_input(int button_in)
{
	if(button_in ==6) //up error if below 0.5
	{
		if((error != 0.5) && (error < 0.5))
		{
			error+=0.1;
		}
		
	}
	else if(button_in == 7) //down error if above 0
	{
		if((error != 0.1) && (error > 0.1))
		{
			error-=0.1;
		}

	}
	else if(button_in==2) //up target range
	{
		if((error != 1.0) && (error < 1.0))
		{
			error+=0.1;
		}
	}
	else if(button_in==3) //down target range
	{
		if((error != 0.0) && (error > 0.0))
		{
			error-=0.1;
		}
	}
	else if(button_in == 0) // increase bits with a max at 14
	{
		if((bit_number == 14) || (bit_number > 14))
		{
			//do nothing
		}
		else{
			bit_number++;
		}
	}
	else if(button_in == 5) // lower bits with a min at 6
	{
		if((bit_number == 6) || (bit_number < 6))
		{
			//do nothing
		}
		else{
			bit_number--;
		}
	}
	else if(button_in == 4) //begin trim 
	{
		if(trim_status == 0)
		{
			trim_status = 1; //turns on trimming process
											//Is a global variable as it will be set back to 0 whenever the process is over
		}
	}
	else if(button_in ==1) //turns on our trim analysis
	{
		if(trim_algo == 0)
		{
			trim_algo = 1; //turn on our trim analysis
		}
		else if(trim_algo == 1)
		{
			trim_algo = 0; //turn onto brute force
		}

	}
		
}

int32_t counter;
void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
	trim_status = 1;
  counter = counter + 1;
	//Read_Input();
	uint32_t i;
	LEDS = BLUE;
	/*for(i=0;i<8;i+=1)
	{
		if(button_debouncer[i] > 0)
		{
			button_debouncer[i]-=1;
			if(button_debouncer[i] == 0) //should always get here
			{
				Button_input(button_debouncer[i]);
			}
		}
	}*/
}

/*void GPIOPORTB_Handler(void)
{
	counter = counter + 1;
	GPIO_PORTB_ICR_R=0x10;///ask 
	Read_Input();
	uint32_t i;
	LEDS = BLUE;
	for(i=0;i<8;i+=1)
	{
		if(button_debouncer[i] > 0)
		{
			button_debouncer[i]-=1;
			if(button_debouncer[i] == 0) //should always get here
			{
				Button_input(button_debouncer[i]);
			}
		}
	}
}*/

