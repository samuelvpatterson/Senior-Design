// SwitchInput.c
// Mohit Joshi and Robert Bolt
// February 7, 2018

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"


#define PF2   (*((volatile uint32_t *)0x40025010))
#define PF4   (*((volatile uint32_t *)0x40025040))
volatile int port_data;
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value

int heartbeat = 0;

void PortB_Init(void)
{
	SYSCTL_RCGCGPIO_R |= 0x08;        // 1) activate clock for Port F
	while((SYSCTL_PRGPIO_R&0x08)==0){}; // allow time for clock to start
	GPIO_PORTD_AMSEL_R &= ~0xFF;      // 3) disable analog on PA5
	GPIO_PORTD_PCTL_R &= ~0xFFFFFFFF; // 4) PCTL GPIO on PA5
	GPIO_PORTD_DIR_R &= ~0xFF;        // 5) direction PA5 input
	GPIO_PORTD_AFSEL_R &= ~0xFF;      // 6) PA5 regular port function
	GPIO_PORTD_DEN_R |= 0xFF;         // 7) enable PA5 digital port

}

void SysTick_Init(uint32_t period){long sr;
  sr = StartCritical();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EndCritical(sr);
}

void SysTick_Handler(void)
{
	if((GPIO_PORTD_DATA_R&0xFF)!=0)
	{
		port_data = (GPIO_PORTD_DATA_R&0xFF);
	}
	else
	{
		port_data = 0;
	}
	heartbeat++;
	if(heartbeat==150)
	{
		PF2 ^= 0x04;
		heartbeat=0;
	}
}

void SwitchInit()
{
	PortB_Init();
	SysTick_Init(80000);
	port_data = 0;
}



