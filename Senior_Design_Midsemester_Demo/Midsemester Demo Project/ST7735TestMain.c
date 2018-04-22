// ST7735TestMain.c
// Runs on LM4F120/TM4C123
// Test the functions in ST7735.c by printing basic
// patterns to the LCD.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano
// March 30, 2015

/* This example accompanies the book
  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
  ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
   You may use, edit, run or distribute this file
   as long as the above copyright notice remains
THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
For more information about my classes, my research, and my books, see
http://users.ece.utexas.edu/~valvano/
*/

// hardware connections
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// **********wide.hk ST7735R with ADXL345 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// SDA  – (NC) I2C data for ADXL345 accelerometer
// SCL  – (NC) I2C clock for ADXL345 accelerometer
// SDO  – (NC) I2C alternate address for ADXL345 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

// **********wide.hk ST7735R with ADXL335 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// X– (NC) analog input X-axis from ADXL335 accelerometer
// Y– (NC) analog input Y-axis from ADXL335 accelerometer
// Z– (NC) analog input Z-axis from ADXL335 accelerometer
// Backlight + - Light, backlight connected to +3.3 V
// ST7735TestMain.c
// Runs on LM4F120/TM4C123
// Test the functions in ST7735.c by printing basic
// patterns to the LCD.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano
// March 30, 2015

/* This example accompanies the book
  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
  ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014

Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
   You may use, edit, run or distribute this file
   as long as the above copyright notice remains
THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
For more information about my classes, my research, and my books, see
http://users.ece.utexas.edu/~valvano/
*/

// hardware connections
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// **********wide.hk ST7735R with ADXL345 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// SDA  – (NC) I2C data for ADXL345 accelerometer
// SCL  – (NC) I2C clock for ADXL345 accelerometer
// SDO  – (NC) I2C alternate address for ADXL345 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

// **********wide.hk ST7735R with ADXL335 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// X– (NC) analog input X-axis from ADXL335 accelerometer
// Y– (NC) analog input Y-axis from ADXL335 accelerometer
// Z– (NC) analog input Z-axis from ADXL335 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "TrimmingDisplayDriver.h"
#include "linear_case.h"
#include "Buttons_32.h"
#include "ST7735.h"
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "DAC.h"
#include "ADCSWTrigger.h"
#include "SwitchInput.h"
#include "mappings.h"
#include "team_32.h"

#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))
#define error_value 25
#define target_v 512

volatile int start = 0;

uint16_t positivelow[]  = {2 ^ 4, 2 ^ 5, 2 ^ 6, 2 ^ 7, 2 ^ 8, 2 ^ 9, 2 ^ 10, 2 ^ 11};
uint16_t positivemid[]  = {2 ^ 5, 2 ^ 5, 2 ^ 5, 2 ^ 5, 2 ^ 5, 2 ^ 5, 2 ^ 5, 2 ^ 5};
uint16_t positivehigh[]  = {2 ^ 11, 2 ^ 10, 2 ^ 9, 2 ^ 8, 2 ^ 7, 2 ^ 6, 2 ^ 5, 2 ^ 4};

uint16_t linear_neg[2048] =

// Right now I am hardcoding values, ideally they will be read in by buttons
    float dac_max_v, dac_min_v;
uint32_t num_bits;

typedef uint8_t bool;
#define true 1
#define false 0

//float error; //currently unused since only testing linear case
void DisableInterrupts(void);
void EnableInterrupts(void);
bool ResultIsCorrect(uint32_t result);

volatile uint32_t ADCvalue;

void DelayWaitms(uint32_t n)
{
	uint32_t volatile time;
	while (n)
	{
		time = 24950;  // 10msec
		while (time) {
			time--;
		}
		n--;
	}
}
#define SWITCHES                (*((volatile uint32_t *)0x40025044))
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08

uint32_t Board_Input(void) {
	return SWITCHES;
}

#define SW1       0x10                      // on the left side of the Launchpad board
#define SW2       0x01                      // on the right side of the Launchpad board
void Board_Init(void) {
	SYSCTL_RCGCGPIO_R |= 0x20;     // 1) activate Port F
	while ((SYSCTL_PRGPIO_R & 0x20) == 0) {}; // ready?
	// 2a) unlock GPIO Port F Commit Register
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTF_CR_R |= (SW1 | SW2); // 2b) enable commit for PF4 and PF0
	// 3) disable analog functionality on PF4 and PF0
	GPIO_PORTF_AMSEL_R &= ~(SW1 | SW2);
	// 4) configure PF0 and PF4 as GPIO
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R & 0xFFF0FFF0) + 0x00000000;
	GPIO_PORTF_DIR_R &= ~(SW1 | SW2); // 5) make PF0 and PF4 in (built-in buttons)
	// 6) disable alt funct on PF0 and PF4
	GPIO_PORTF_AFSEL_R &= ~(SW1 | SW2);
//  delay = SYSCTL_RCGC2_R;        // put a delay here if you are seeing erroneous NMI
	GPIO_PORTF_PUR_R |= (SW1 | SW2); // enable weak pull-up on PF0 and PF4
	GPIO_PORTF_DEN_R |= (SW1 | SW2); // 7) enable digital I/O on PF0 and PF4
}

void Timer0A_Init100HzInt(void) {
	volatile uint32_t delay;
	DisableInterrupts();
	// **** general initialization ****
	SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
	delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
	TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
	TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
	// **** timer0A initialization ****
	// configure for periodic mode
	TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
	TIMER0_TAILR_R = 799999;         // start value for 100 Hz interrupts
	TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interruptF
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
	TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 32-b, periodic, interrupts
	// **** interrupt initialization ****
	// Timer0A=priority 2
	NVIC_PRI4_R = (NVIC_PRI4_R & 0x00FFFFFF) | 0x40000000; // top 3 bits
	NVIC_EN0_R = 1 << 19;            // enable interrupt 19 in NVIC
}

void Timer1_Init(void) {
	volatile uint32_t delay;
	SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
	TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
	TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
	TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
	TIMER1_TAILR_R = 0xFFFFFFFF;    // 4) reload value
//  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
	TIMER1_TAPR_R = 0;            // 5) bus clock resolution
	TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER1A
}
void Timer0A_Handler(void) {
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
	PF2 ^= 0x04;                   // profile
	PF2 ^= 0x04;                   // profile
	ADCvalue = ADC0_InSeq3();
	PF2 ^= 0x04;                   // profile
}

int bits = 6;
int degree = 1;
double target = 1.0;
double printError = 1.0;


void DrawScreenBase() {
	ST7735_FillScreen(0);
	ST7735_SetCursor(0, 0);
	ST7735_OutString("Number of bits: ");
	ST7735_OutUDec(bits);
	ST7735_SetCursor(0, 2);
	ST7735_OutString("Current Mode: Trim");
	ST7735_SetCursor(0, 4);
	ST7735_OutString("Degree: ");
	ST7735_OutUDec(degree);
	ST7735_SetCursor(0, 6);
	ST7735_OutString("Trim Value: ");
	printf("%.1f" , target);
	ST7735_SetCursor(0, 8);
	ST7735_OutString("Error Range: +/- ");
	printf("%.1f" , printError);
}

enum Cases {
	LINEARNEG,
	LINEARCONST,
	LINEAR;
};

uint32_t BS_Find_Dac_Out(uint32_t low, uint32_t high, uint32_t target)
{
	uint32_t mid, voltage;

	if (low >= high)
	{
		mid = (high - low) / 2;

		DAC_Out(mid);
		voltage = ADC0_InSeq3();
		voltage = voltage > 2047 ? 2047 : voltage;

		if (abs(target - voltage) <= error)
			return mid;

		if (target > mid)
			return BS_Find_Dac_Out(mid, high, target);
		else
			return BS_Find_Dac_Out(low, mid, target);
	}

	return -1; //not found
}

uint16_t getADCOutput(uint16_t dacIn) {
	DAC_Out(dacIn);
	//DelayWaitms(20);
	//double newOut = 0;
	uint16_t outputVal = ADC0_InSeq3();
	ST7735_SetCursor(0, 0);
	outputVal = ADC0_InSeq3();
	double ratio = (double)dacIn / (double)outputVal;
	double newOut = (double)(outputVal) * (ratio);
	outputVal = (uint16_t)newOut;
	ST7735_OutUDec(outputVal);
	return outputVal;
}

int main(void)
{
	DisableInterrupts();
	PLL_Init(Bus20MHz);                  // set system clock to 80 MHz
	ST7735_InitR(INITR_REDTAB); 				 // initialize LCD
	//EdgeCounter_Init();		// initialize buttons
	Board_Init();
	DAC_Init(0); 												 // TODO whoever has DAC code
	ADC0_InitSWTriggerSeq3_Ch9();													 // TODO not sure if necessary
	SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
	while ((SYSCTL_PRGPIO_R & 0x20) == 0) {}; // ready?
	// allow time to finish activating
	Timer0A_Init100HzInt();               // set up Timer0A for 100 Hz interrupts
	Timer1_Init();
	GPIO_PORTF_DIR_R |= 0x06;             // make PF2, PF1 out (built-in LED)
	GPIO_PORTF_AFSEL_R &= ~0x06;          // disable alt funct on PF2, PF1
	GPIO_PORTF_DEN_R |= 0x06;             // enable digital I/O on PF2, PF1
	// configure PF2 as GPIO			  // any other inits I forgot about
	EnableInterrupts();

	SwitchInit();
	int32_t y2;
	int32_t y1;
	fixedpt result;
	int32_t slope;
	int32_t potTarget;
	fixedpt resolution = 0; //change to use floating point library
	double conversion = 0;


	/* UI */
	DrawScreenBase();

	uint8_t testCase = 0;
	uint32_t outputVal = 0;
	uint8_t start = 0;

	while (start == 0)
	{
		uint32_t status;
		status = Board_Input();

		//Read from port F for 'start', will be replaced with on-board buttons
		//Port F switches are negative logic.
		switch (status)
		{
		// SW1 pressed
		case 0x01:
			start = 1;
			break;
		}

		//Read rest of buttons to update UI
		if (port_data != 0 && start == 0)
		{
			switch (port_data)
			{
			case 1:
				start = 1;
				DrawScreenBase();
				break;
			case 2:
				bits = (bits) % 6 + 6;
				DrawScreenBase();
				break;
			case 4:
				break;
				degree = (degree) % 3 + 1;
				DrawScreenBase();
			case 8:
				if (target < 3.3) {
					target += 0.1;
					DrawScreenBase();
				}
				break;
			case 64:
				if (target >= 0.1) {
					target -= 0.1;
					DrawScreenBase();
				}
				break;
			}
			DelayWaitms(10);
		}
	}

	/*Trimming Algorithm */

	//This is an incomplete outline that paints the idea. First, assume linear
	//in which case result calculation is straight-forward. Second, assume quadratic.
	//Finally, assume cubic. Using Julian's mappings, we can directly calculate the the
	//result in each case.

	/* TODO */

	//import Julian's tables/fixed point header
	//convert to fixed point arithmetic
	//modularize
	//debug

	//Initialization
	fixedpt fp_v_high = convert_double_to_fp(2.47);
	fixedpt fp_v_low = 0;
	fixedpt fp_bit_power = convert_double_to_fp((1 << num_bits) - 1);

	fixedpt fp_target = convert_double_to_fp(target);
	fixedpt fp_error = convert_double_to_fp(error);

	//Try linear
	resolution = (fp_v_high - fp_v_low) / fp_bit_power;
	result = (fp_target - fp_v_low) / resolution;

	uint16_t ADC_result = (uint16_t) convert_fp_to_uint64_t_rz(result);

	//test it
	if (ResultIsCorrect(ADC_result, fp_target, fp_error))
	{
		//convert to proper format
		//print result
	}
	else
	{
		//Try quadratic 
		resolution = quartic_11_bits[(1 << num_bits) - 1] - 0 / fp_bit_power; //Indices should by 2^n - 1 and 0
		double temp = (target - v_low) / resolution;
		result = temp * temp;

		if (ResultIsCorrect(result))
		{
			//print result
		}
		else
		{
			//Try cubic
			resolution = CUBIC_ROOTS[v_high] - CUBIC_ROOTS[v_low] / (1 << num_bits);
			temp = (target - v_low) / resolution;
			result = temp * temp * temp;

			if (ResultIsCorrect(result))
			{
				//print result
			}
			else
			{
				ST7735_FillScreen(0);
				ST7735_OutString("No Solution Found");

			}
		}
	}
}

bool
ResultIsCorrect(uint16_t result, fixedpt target, fixedpt error)
{
	fixedpt newresult = div_fp(convert_uint64_t_to_fp((uint64_t) result), resolution);
	return abs(target - newresult) < error;
}

//}
/*	ST7735_SetCursor(0,2);
	double divider = max/((double)outputVal);
	ST7735_OutUDec((int)divider);
	ST7735_SetCursor(0,4);
	double subtract = error/divider;
	ST7735_OutUDec((int)subtract);
	ST7735_SetCursor(0,6);
	outputVal = outputVal - (int32_t)subtract;
	ST7735_OutUDec(outputVal);
	ST7735_SetCursor(0,8);*/

//	while(trim_status == 0){} //wait for begin button once begin is clicked, find linear value
//	uint32_t result = Linear_Find_Dac_Out(dac_max_v, dac_min_v, target_v, 8);


/*	}
}
*/


