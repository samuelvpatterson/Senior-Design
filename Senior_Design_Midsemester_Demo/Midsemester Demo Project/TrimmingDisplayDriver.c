#include "ST7735.h"

//extern uint32_t VoltageTarget;
//extern uint32_t VoltageActual;
//extern uint32_t ErrorTarget;
//extern uint32_t Actual;

static char *equals = " = ";
static char *new_line = "\r";

void Pause(void); //Waits until button press is read, once it is, we can switch screen/lock value, etc.

//Display Target Value
/*void Display_Target(void)
{
	ST7735_FillScreen(ST7735_BLACK); 
	ST7735_SetCursor(0,0);
	printf("Configure Trim Target\rVoltage\r");
	ST7735_sDecOut2(VoltageTarget);  				 	 //'VoltageTarget' will be some global variable maintained
																						 // Value will change through buttons, once pushed we will switch
	Pause();																	 // To target error
}

//Display Target Error Range
void Display_Error(void)
{
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetCursor(0,0);
	printf("Configure Trim Error\rPercentage\r");
	ST7735_sDecOut2(ErrorTarget);  					 	 //'ErrorTarget' will be some global variable maintained
																						 // Value will change through buttons, once pushed we will switch
	Pause();																	 // To Pin display
}

//Display Individual Pin Values
void Display_Pins(void)
{
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetCursor(0,0);
	printf("Pin Voltage Values\r");
	int i = 0;
	for(i=0; i<6; i++)
	{
		ST7735_OutString(Pins[i].name);         //Assuming we store in a Struct with similar structure
		ST7735_OutString(equals);
		ST7735_sDecOut2(Pins[i].value);  				//Can be modified depending on how we store the data
		ST7735_OutString(new_line);
	}
	
	Pause();
}
*/
void Display_Results(uint32_t VoltageTarget, uint32_t VoltageActual)
{
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetCursor(0,0);
	printf("Desired Value\r");
	ST7735_sDecOut2(VoltageTarget);
	printf("Actual Value\r");
	ST7735_sDecOut2(VoltageActual);
	
	Pause();
}
