#include "ST7735.h"

//extern uint32_t VoltageTarget;
//extern uint32_t VoltageActual;
//extern uint32_t ErrorTarget;
//extern uint32_t Actual;

void Pause(void); //Waits until button press is read, once it is, we can switch screen/lock value, etc.

//Display Target Value
void Display_Target(void);

//Display Target Error Range
void Display_Error(void);

//Display Individual Pin Values
void Display_Pins(void);

void Display_Results(uint32_t VoltageTarget, uint32_t VoltageActual);