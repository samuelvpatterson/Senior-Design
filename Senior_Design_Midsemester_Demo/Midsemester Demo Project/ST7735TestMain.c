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

//uint16_t linear_neg[2048] =

// Right now I am hardcoding values, ideally they will be read in by buttons
float dac_max_v, dac_min_v;
uint32_t num_bits;

typedef uint8_t bool;
#define true 1
#define false 0

// Print binary value
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x400 ? '1' : '0'), \
  (byte & 0x200 ? '1' : '0'), \
  (byte & 0x100 ? '1' : '0'), \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

//float error; //currently unused since only testing linear case
void DisableInterrupts(void);
void EnableInterrupts(void);
bool ResultIsCorrect(fixedpt result,
                     fixedpt result_res,
                     fixedpt target,
                     fixedpt target_res,
                     fixedpt error);

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
double printError = 0.1;


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

void DrawSuccessfulResults(char *degree, fixedpt fp_trim) {
	uint16_t trim = (uint16_t) convert_fp_to_uint64_t_rz(fp_trim);

	ST7735_FillScreen(0);
	ST7735_SetCursor(0, 0);
	printf("Successful Trim\n%s (Binary):\n", degree);
	printf(""BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(trim));
}

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
	SwitchInit();
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

	//SwitchInit();
	int32_t y2;
	int32_t y1;
	fixedpt result;
	int32_t slope;
	int32_t potTarget;
	fixedpt resolution = 0; //change to use floating point library
	double conversion = 0;


	//printf("Hello");
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
				bits = (bits + 1) % 6 + 6;
				DrawScreenBase();
				break;
			case 2:
				degree = (degree) % 3 + 1;
				DrawScreenBase();
				break;
			case 4:
				if (target >= 0.1) {
					target -= 0.1;
					DrawScreenBase();
				}
				break;
			case 8:
				if (target < 2.4) {
					target += 0.1;
					DrawScreenBase();
				}
				break;
			case 64:
				if (printError >= 0.1) {
					printError -= 0.1;
					DrawScreenBase();
				}
				break;
			case -128:
				if (printError < 0.5) {
					printError += 0.1;
					DrawScreenBase();
				}
				DrawScreenBase();
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
	fixedpt fp_bit_power = convert_double_to_fp((1 << bits) - 1);

	fixedpt fp_target = convert_double_to_fp(target);
	fixedpt fp_error = convert_double_to_fp(printError);


	// I don't think any of this part is necessary.

	// Try linear
	/*resolution = (fp_v_high - fp_v_low) / fp_bit_power;*/
	/*result = (fp_target - fp_v_low) / resolution;*/

	// Target result (using the resolution computed by the user's specifications)

	// Depending on the degree and bits specified, compute the appropriate target resolution

	/*
	fixedpt fp_resolution;
	fixedpt fp_user_trim;

	switch (degree)
	{
	    case 1:
	        // Regular Linear
	        fp_resolution = fixedpt_div((fp_v_high - 0), fp_bit_power);
	        fp_user_trim = fixedpt_div((fp_target - fp_v_low), fp_resolution);
	        break;
	    case 2:
	        // Quadratic
	        switch (bits)
	        {
	            case 6:
										fp_resolution = div_fp((sqrt_11_bits[63] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 7:
										fp_resolution = div_fp((sqrt_11_bits[127] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 8:
										fp_resolution = div_fp((sqrt_11_bits[255] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 9:
										fp_resolution = div_fp((sqrt_11_bits[511] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 10:
										fp_resolution = div_fp((sqrt_11_bits[1023] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 11:
										fp_resolution = div_fp((sqrt_11_bits[2047] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	        }

	        break;
	    case 3:
	        // Cubic
	        switch (bits)
	        {
	            case 6:
										fp_resolution = div_fp((cubic_11_bits[63] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 7:
										fp_resolution = div_fp((cubic_11_bits[127] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 8:
										fp_resolution = div_fp((cubic_11_bits[255] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 9:
										fp_resolution = div_fp((cubic_11_bits[511] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 10:
										fp_resolution = div_fp((cubic_11_bits[1023] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	            case 11:
										fp_resolution = div_fp((cubic_11_bits[2047] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
	                fp_user_trim = div_fp((fp_target - fp_v_low), fp_resolution);
	                break;
	        }

	        break;

	    // TODO: Map number to quartic, linear_negative, linear constant, and negative_linear
	}
	*/



	// First, find what the linear value would be.
	fixedpt fp_linear_resolution = div_fp((fp_v_high - 0), fp_bit_power);
	fixedpt fp_linear_trim = div_fp((fp_target - fp_v_low), fp_linear_resolution);

	// Once you have that, you assume it is linear, and use ResultIsCorrect to verify. All result is correct requires
	// is the value we should pass in the event of a linear device.

	//if (ResultIsCorrect(fp_linear_trim))
	//{
	if (degree == 1) //ResultIsCorrect(fp_user_trim, fp_resolution, fp_linear_trim, fp_linear_resolution, fp_error)) ----- OLD
	{
		// print result
		DrawSuccessfulResults("Linear", fp_linear_trim);
	}
	else
	{
		// None of this is really necessary. Assuming the device is quadratic, we simply take the square root of the linear solution.
		// We then pass that value to the new ResultIsCorrect in order to test whether that is the case.

		// First get the linear value into proper integer form
		uint32_t linear_int_equiv = (uint32_t) convert_fp_to_uint64_t_rz(fp_linear_trim);

		// Now use this value as an index into the sqrt array in order to determine the sqrt if that number
		fixedpt fp_quadratic_trim = sqrt_11_bits[linear_int_equiv];

		// Now test this value since we assume quadratic by calling ResultIsCorrect(fp_quadratic_trim)





		// //Try quadratic
		//       fixedpt fp_quadratic_resolution;
		//       fixedpt fp_quadratic_trim;

		//       switch (bits)
		//       {
		//           case 6:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[63] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;
		//           case 7:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[127] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;
		//           case 8:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[255] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;
		//           case 9:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[511] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;
		//           case 10:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[1023] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;
		//           case 11:
		//               fp_quadratic_resolution = div_fp((sqrt_11_bits[2047] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
		//               fp_quadratic_trim = div_fp((fp_target - fp_v_low), fp_quadratic_resolution);
		//               break;

//       }


		//if (ResultIsCorrect(fp_quadratic_trim))
		//{
		if (degree == 2)//ResultIsCorrect(fp_quadratic_trim, fp_quadratic_resolution, fp_user_trim, fp_resolution, fp_error))  -------- OLD
		{
			//print result
			DrawSuccessfulResults("Quadratic", sqrt_11_bits[convert_fp_to_uint64_t_rz(fp_linear_trim)]);
		}
		else
		{

			// Finally, we apply a similar approach for cubic.


			// First get the linear value into proper integer form
			uint32_t linear_int_equiv = (uint32_t) convert_fp_to_uint64_t_rz(fp_linear_trim);

			// Now use this value as an index into the sqrt array in order to determine the sqrt if that number
			fixedpt fp_cubic_trim = cubic_11_bits[linear_int_equiv];

			// Now test this value with ResultIsCorrect

			//   fixedpt fp_cubic_resolution;
			//   fixedpt fp_cubic_trim;

			//   switch (bits)
			//   {
			//       case 6:
			// fp_cubic_resolution = div_fp((cubic_11_bits[63] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			//       case 7:
			// fp_cubic_resolution = div_fp((cubic_11_bits[127] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			//       case 8:
			// fp_cubic_resolution = div_fp((cubic_11_bits[255] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			//       case 9:
			// fp_cubic_resolution = div_fp((cubic_11_bits[511] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			//       case 10:
			// fp_cubic_resolution = div_fp((cubic_11_bits[1023] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			//       case 11:
			// fp_cubic_resolution = div_fp((cubic_11_bits[2047] - 0), fp_bit_power); //Indices should by 2^n - 1 and 0
			//           fp_cubic_trim = div_fp((fp_target - fp_v_low), fp_cubic_resolution);
			//           break;
			// }

			//if (ResultIsCorrect(fp_cubic_trim))
			//{
			// Try cubic
			if (degree == 3)//ResultIsCorrect(fp_cubic_trim, fp_cubic_resolution, fp_user_trim, fp_resolution, fp_error)) -------- OLD
			{
				//print result
				DrawSuccessfulResults("Cubic", cubic_11_bits[convert_fp_to_uint64_t_rz(fp_linear_trim)]);
			}
			else
			{
				ST7735_FillScreen(0);
				ST7735_SetCursor(0, 0);
				ST7735_OutString("No Solution Found!");
			}
		}
	}
}


bool
ResultIsCorrect(fixedpt result,
                fixedpt result_res,
                fixedpt target,
                fixedpt target_res,
                fixedpt error)
{

	fixedpt result_voltage, target_voltage;

	switch (degree)
	{
	case 1:
		result_voltage = fixedpt_mul(result, result_res);
	case 2:
		result_voltage = fixedpt_mul(convert_uint64_t_to_fp((uint64_t) quadratic11[convert_fp_to_uint64_t_rz(result)]), result_res);
	case 3:
		result_voltage = fixedpt_mul(convert_uint64_t_to_fp((uint64_t) cubic11[convert_fp_to_uint64_t_rz(result)]), result_res);
	}

	target_voltage = fixedpt_mul(target, target_res);

	ST7735_FillScreen(0);
	ST7735_SetCursor(0, 0);
	ST7735_OutUDec((uint32_t) result);
	ST7735_SetCursor(0, 2);
	ST7735_OutUDec((uint32_t) result_res);
	ST7735_SetCursor(0, 4);
	ST7735_OutUDec((uint32_t) result_voltage);

	while (1);
	return (abs(result_voltage - target_voltage) <= error);
}


// Modified version of ResultIsCorrect that properly uses Robert's arrays as well as DAC and ADC
bool
ResultIsCorrect(fixedpt test_value)
{
	// In order to simulate different devices, we will use user parameters (since we only have an 11 bit linear DAC)

	// First, get the value into integer form no matter which degree.
	uint16_t integer_form = (uint16_t) convert_fp_to_uint64_t_rz(value);

	switch (degree)
	{
	// If linear, we simply pass the calculated value shifted left 1 - numBits times (this is necessary to convert it to the 11 bit equivalent)
	case 1:
	//Use a modified version (yet to be written) of Robert's ADCIn Function above with the left-shift version of this number.

	/* PSEUDO

	* Using the found linear value, pass it out and see if the input matches what we expect in a linear case. If it doesn't,
	* the input isn't linear.

	uint16_t adc_res = testADCIn(integer_form << (1 - numbits));

	* Shift it back to get it original form
	uint16_t shifted_res = adc_res >> (1 - numbits);

	* Convert to FP and check against target value
	fixedpt fp_shifted_res = convert_uint64_t_to_fp((uint64_t) shifted_res);

	* Do this by calculating what the target value would be in a linear case and testing our result against that.
	* In the case of a linear trim, this will always be correct.
	fixedpt fp_target_voltage = test_value;

	return (abs(fp_sub(fp_shifted_res, fp_linear_trim) <= fp_error));
	*/
	// In quadratic case, we use a similar approach, but determine what the 11 bit linear equivalent will be of our result using Robert's tables
	case 2:
		/* PSEUDO

		// In this case, the input is the sqrt of the what the linear value would be. We will use Robert's table to find its quadratic equivalent.
		uint16_t quad_equiv = quadratic11[integer_form];

		uint16_t adc_res = testADCIn(quad_equiv << (1 - numbits));

		* Shift it back to get it original form
		uint16_t shifted_res = adc_res >> (1 - numbits);
		
		* We have to get it back into its sqrt form, so use Julian's tables
		fixedpt sqrt_conversion = sqrt_11_bits[shifted_res];

		* Convert to FP and check against target value
		fixedpt fp_shifted_res = convert_uint64_t_to_fp((uint64_t) sqrt_conversion);

		* Do this by calculating what the target value would be in a linear case and testing our result against that.
		* In the case of a linear trim, this will always be correct.

		fixedpt fp_target_voltage = test_value;

	
		return (abs(fp_sub(fp_shifted_res, fp_linear_trim)) <= fp_error);
		*/
	// Same as before, but with cubic table
	case 3:
		/* PSEUDO

		// In this case, the input is the sqrt of the what the linear value would be. We will use Robert's table to find its quadratic equivalent.
		uint16_t cubic_equiv = cubic11[integer_form];

		uint16_t adc_res = testADCIn(quad_equiv << (1 - numbits));

		* Shift it back to get it original form
		uint16_t shifted_res = adc_res >> (1 - numbits);
		
		* We have to get it back into its sqrt form, so use Julian's tables
		fixedpt cubic_conversion = cubic_11_bits[shifted_res];

		* Convert to FP and check against target value
		fixedpt fp_shifted_res = convert_uint64_t_to_fp((uint64_t) cubic_conversion);

		* Do this by calculating what the target value would be in a linear case and testing our result against that.
		* In the case of a linear trim, this will always be correct.

		fixedpt fp_target_voltage = test_value;

	
		return (abs(fp_sub(fp_shifted_res, fp_target_voltage)) <= fp_error);
	}
}


/*bool*/
/*ResultIsCorrect(uint16_t result, fixedpt target, fixedpt error)*/
/*{*/
/*fixedpt newresult;	*/

/*switch (degree)*/
/*{*/
/*case 1:*/
/*// Regular Linear*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) result), resolution);*/
/*break;*/
/*case 2:*/
/*// Quadratic*/
/*switch (bits)*/
/*{*/
/*case 6:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic6[result]), resolution);*/
/*break;*/
/*case 7:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic7[result]), resolution);*/
/*break;*/
/*case 8:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic8[result]), resolution);*/
/*break;*/
/*case 9:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic9[result]), resolution);*/
/*break;*/
/*case 10:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic10[result]), resolution);*/
/*break;*/
/*case 11:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) quadratic11[result]), resolution);*/
/*break;*/
/*}*/

/*break;*/
/*case 3:*/
/*// Cubic */
/*switch (bits)*/
/*{*/
/*case 6:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic6[result]), resolution);*/
/*break;*/
/*case 7:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic7[result]), resolution);*/
/*break;*/
/*case 8:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic8[result]), resolution);*/
/*break;*/
/*case 9:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic9[result]), resolution);*/
/*break;*/
/*case 10:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic10[result]), resolution);*/
/*break;*/
/*case 11:*/
/*newresult = div_fp(convert_uint64_t_to_fp((uint64_t) cubic11[result]), resolution);*/
/*break;*/
/*}*/

/*break;*/

/*// TODO: Map number to quartic, linear_negative, linear constant, and negative_linear*/

/*default:*/
/*break;*/
/*}*/

/*return abs(target - newresult) < error;*/
/*}*/

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


