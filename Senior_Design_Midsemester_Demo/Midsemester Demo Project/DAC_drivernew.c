#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "debug.h"
#include "interrupt.h"
#include "ssi.h"
#include "../inc/tm4c123gh6pm.h"


void Data_result(uint16_t data){
	SysCtlPeripheralEnable(0xf0001C00); //Enable the SSIO peripheral
	while(!SysCtlPeripheralReady(0xf0001C00)) // Wait for the SSI0 module to be ready.
	{
	}
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), 0x00000000, 0x00000000, 2000000, 8); // Configure the SSI.
	SSIEnable(SSI0_BASE);// Enable the SSI module.
	SSIDataPut(SSI0_BASE, data); // Send some data.
	
}
