//-----------------------------------------------------------------------------
// Part of the code used to configure and call the OLED.C routines
// L. BAGHLI 03/2018
//-----------------------------------------------------------------------------

#include <math.h>


// modules
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/memCopy/src/memCopy.h"
#include "F2806x_Device.h"     // F2806x Headerfile Include File

#include "oled.h"


struct {
	Uint16 Running		:1;
	Uint16 CheckRX		:1;
	Uint16 SendTX		:1;
	Uint16 SendData		:1;
	Uint16 UARTIRef		:1;
	Uint16 RefreshAll	:1;
	Uint16 RefreshOLED	:1;
	Uint16 unused 		:9;
	}	Flags;

unsigned int RefreshOLEDCount;
#define RefreshOLEDMax 5000

// GTI variables
GTI gti;

//-----------------------------------------------------------------------------
//	Initialise variables
//-----------------------------------------------------------------------------
void InitVar()
{
	Flags.Running = 1;			// run flag always 1
	Flags.RefreshOLED = 0;
	RefreshOLEDCount = 0;
	gti.thetas=0;
	gti.Vdcref=33;
}
//-----------------------------------------------------------------------------
void RefreshOLED()
{
  GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_54);
  Flags.RefreshOLED = 0;
  oledGotoYX( 1, 6 );
  oledPutDec(gti.Vref1);
  oledGotoYX( 2, 5 );
  //oledPutHexa(gti.Vdc);
  oledPutDec(gti.Vdc);
  oledGotoYX( 3, 5 );
  oledPutHexa(gti.delta);

  GPIO_setLow(halHandle->gpioHandle,GPIO_Number_54);
}
//-----------------------------------------------------------------------------
void main(void)
{
  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

// this part also init the GPIO, I put the part here :

// I2C used on GPIO32/33   LB 03/2018
  // I2C Data
  GPIO_setMode(halHandle->gpioHandle,GPIO_Number_32,GPIO_32_Mode_SDAA);
  // I2C Clock
  GPIO_setMode(halHandle->gpioHandle,GPIO_Number_33,GPIO_33_Mode_SCLA);
  // config I2C
  GPIO_setPullup(halHandle->gpioHandle,GPIO_Number_32, GPIO_Pullup_Enable);	// Enable pull-up for GPIO32 (SDAA)
  GPIO_setPullup(halHandle->gpioHandle,GPIO_Number_33, GPIO_Pullup_Enable);
  GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_32, GPIO_Qual_ASync);// Asynch input GPIO32 (SDAA)
  GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_33, GPIO_Qual_ASync);

  InitVar();

  // initialize the interrupt vector table
	// config an ISR that will call mainISR
	//....

  InitI2C();		// init I2C OLED config

  while(true)
  {
   if (Flags.RefreshOLED) RefreshOLED();
  }
} // end of main() function
//-----------------------------------------------------------------------------
interrupt void mainISR(void)
{
	GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_23);

// some work then we output data on OLED every 

  if (++RefreshOLEDCount == RefreshOLEDMax) {
    Flags.RefreshOLED = 1;
    RefreshOLEDCount=0;
    }

  GPIO_setLow(halHandle->gpioHandle,GPIO_Number_23);
  return;
} // end of mainISR() function
