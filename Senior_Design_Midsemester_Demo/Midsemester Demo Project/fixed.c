////Samuel Patterson svp395
////Christopher Gang cg37877

//#include "fixed.h"
//#include "ST7735.h"
//#include <stdio.h>
//#include <stdlib.h>


//int32_t MaxX, MinX, MaxY, MinY;

///****************ST7735_sDecOut2***************
// converts fixed point number to LCD
// format signed 32-bit with resolution 0.01
// range -99.99 to +99.99
// Inputs:  signed 32-bit integer part of fixed-point number
// Outputs: none
// send exactly 6 characters to the LCD 
//Parameter LCD display
// 12345    " **.**"
//  2345    " 23.45"  
// -8100    "-81.00"
//  -102    " -1.02" 
//    31    "  0.31" 
//-12345    "-**.**"
// */ 
//void ST7735_sDecOut2(int32_t n)
//{
//	//char buffer[6];
//	
//	if (n >= 10000)
//		//ST7735_OutString(" **.**");
//		printf(" **.**");
//	else if (n <= -10000)
//		//ST7735_OutString("-**.**");
//		printf("-**.**");
//	else
//	{	
//		//itoa(n * 0.01, buffer, 10);	
//		//ST7735_OutString(buffer);
//		if (n >= 0)
//			printf(" %.2f", (float) n * 0.01);
//		else
//			printf("%.2f", (float) n * 0.01);
//	}
//}

///**************ST7735_uBinOut6***************
// unsigned 32-bit binary fixed-point with a resolution of 1/64. 
// The full-scale range is from 0 to 999.99. 
// If the integer part is larger than 63999, it signifies an error. 
// The ST7735_uBinOut6 function takes an unsigned 32-bit integer part 
// of the binary fixed-point number and outputs the fixed-point value on the LCD
// Inputs:  unsigned 32-bit integer part of binary fixed-point number
// Outputs: none
// send exactly 6 characters to the LCD 
//Parameter LCD display
//     0	  "  0.00"
//     1	  "  0.01"
//    16    "  0.25"
//    25	  "  0.39"
//   125	  "  1.95"
//   128	  "  2.00"
//  1250	  " 19.53"
//  7500	  "117.19"
// 63999	  "999.99"
// 64000	  "***.**"
//*/
//void ST7735_uBinOut6(uint32_t n)
//{
//	if (n >= 64000)
//		//ST7735_OutString(" **.**");
//		printf(" ***.**");
//	else
//	{	
//		//itoa(n * 0.01, buffer, 10);	
//		//ST7735_OutString(buffer);
//		printf(" %.2f", (float) n/64);
//	}
//}

///**************ST7735_XYplotInit***************
// Specify the X and Y axes for an x-y scatter plot
// Draw the title and clear the plot area
// Inputs:  title  ASCII string to label the plot, null-termination
//          minX   smallest X data value allowed, resolution= 0.001
//          maxX   largest X data value allowed, resolution= 0.001
//          minY   smallest Y data value allowed, resolution= 0.001
//          maxY   largest Y data value allowed, resolution= 0.001
// Outputs: none
// assumes minX < maxX, and miny < maxY
//*/
//void ST7735_XYplotInit(char *title, int32_t minX, int32_t maxX, int32_t minY, int32_t maxY)
//{
//	ST7735_FillScreen(0x0000);
//	ST7735_SetCursor(0, 0);
//	ST7735_PlotClear(minY, maxY);
//	printf("%s", title);
//	
//	MaxX = maxX;
//	MinX = minX;
//	MaxY = maxY;
//	MinY = minY;
//}

///**************ST7735_XYplot***************
// Plot an array of (x,y) data
// Inputs:  num    number of data points in the two arrays
//          bufX   array of 32-bit fixed-point data, resolution= 0.001
//          bufY   array of 32-bit fixed-point data, resolution= 0.001
// Outputs: none
// assumes ST7735_XYplotInit has been previously called
// neglect any points outside the minX maxY minY maxY bounds
//*/
//void ST7735_XYplot(uint32_t num, int32_t bufX[], int32_t bufY[])
//{
//	int32_t x, y;
//	int32_t xdif, ydif;
//	
//	xdif = MaxX - MinX;
//	ydif = MaxY - MinY;
//	
//	for (int i = 0; i < num; i++)
//	{	
//		if ((bufX[i] <= MaxX && bufX[i] >= MinX) && (bufY[i] <= MaxY && bufY[i] >= MinY))
//		{
//			x = (128 * (bufX[i] - MinX) / xdif);
//			y = (128 * (MaxY - bufY[i]) / ydif) + 32;

//			ST7735_DrawPixel(x, y, ST7735_RED);
//		}
//	}
//}
