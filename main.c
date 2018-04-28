/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_foc/src/proj_lab01.c
//! \brief CPU and Inverter Set-up and introduction to interfacing to the ROM library
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB01 PROJ_LAB01
//@{

//! \defgroup PROJ_LAB01_OVERVIEW Project Overview
//!
//! CPU and Inverter Set-up and introduction to interfacing to the ROM library
//!

//
// **************************************************************************
// the includes


// modules
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2802x_Device.h"     // F2802x Headerfile Include File
#include "f2802x_globalprototypes.h"
#include "IQmathLib.h"
#include "string.h"

#include "oled.h"

// the globals
int Vref1;   

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

__interrupt void cpu_timer0_isr(void);

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
//extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;


#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif

//-----------------------------------------------------------------------------
//	Initialise variables
//-----------------------------------------------------------------------------
void InitVar()
{
	Flags.Running = 1;			// run flag always 1
	Flags.RefreshOLED = 0;
	RefreshOLEDCount = 0;
	Flags.RefreshOLED = 0;
	Vref1= 0;
}
//-----------------------------------------------------------------------------
void RightJustify(char * in, char * out, int l)
{
	int i, li=strlen(in);
	for (i=0; i<l;i++)		out[i]=' ';
	for (i=0; i<li; i++)	out[i+l-li]=in[i];
	out[l]=0;
}
//-----------------------------------------------------------------------------
void RefreshOLED()
{
	char buffer[10], buffer2[10];
	//GPIO_setHigh(halHandle->gpioHandle,GPIO_Number_12);

	oledGotoYX( 1, 3 );
	_IQtoa(buffer, "%6.3f", _IQ(11.24));
	RightJustify(buffer, buffer2, 6);
	oledPrint(buffer2);
	oledGotoYX( 2, 6 );
	oledPutDec(Vref1);
	//_IQtoa(buffer, "%6d", Vref1);
	//RightJustify(buffer, buffer2, 6);
	//oledPrint(buffer2);
  //GPIO_setLow(halHandle->gpioHandle,GPIO_Number_12);
}
//-----------------------------------------------------------------------------
void main(void)
{
  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
//  memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
//  memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

  #ifdef CSM_ENABLE
    //copy .econst to unsecure RAM
    if(*econst_end - *econst_start)
      {
        memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
      }

    //copy .switch ot unsecure RAM
    if(*switch_end - *switch_start)
      {
        memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
      }
  #endif
  #endif


    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the f2802x_SysCtrl.c file.
    InitSysCtrl();
    // Step 2. Initialize GPIO:
    // This example function is found in the f2802x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //InitGpio();  // Skipped for this example
    // Disable CPU interrupts
    DINT;
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the f2802x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in f2802x_DefaultIsr.c.
    // This function is found in f2802x_PieVect.c.
    InitPieVectTable();
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;    // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;      // This is needed to disable write to EALLOW protected registers

    // Step 4. Initialize the Device Peripheral. This function can be
    // found in f2802x_CpuTimers.c
    // For this example, only initialize the Cpu Timers
    InitCpuTimers();
    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 60MHz CPU Freq, 50 us Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 60, 10);
   // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in f2802x_CpuTimers.h), the
    // below settings must also be updated.
    // Use write-only instruction to set TSS bit = 0
    CpuTimer0Regs.TCR.all = 0x4001;

    // Configure GPIO34 as a GPIO output pin
    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
// I2C used on GPIO32/33   LB 04/2018 F28027F
 // il faut souder JP9 et JP11
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up for GPIO32 (SDAA)
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pull-up for GPIO33 (SCLA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;  // Asynch input GPIO32 (SDAA)
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;  // Asynch input GPIO33 (SCLA)
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;   // Configure GPIO32 for SDAA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;   // Configure GPIO33 for SCLA
    EDIS;

    InitVar();
    InitI2C();		// init I2C OLED config before Timer starts

    // Enable CPU INT1 which is connected to CPU-Timer 0
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Enable TINT0 in the PIE: Group 1 interrupt 7
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM


  // For ever loop
  while(true)
  {
   if (Flags.RefreshOLED) RefreshOLED();

  }

} // end of main() function


interrupt void mainISR(void)
{


} // end of mainISR() function

__interrupt void cpu_timer0_isr(void)
{
	CpuTimer0.InterruptCount++;
	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	// acknowledge the Timer interrupt
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

	if (++RefreshOLEDCount == RefreshOLEDMax) {
		Flags.RefreshOLED = 1;
		RefreshOLEDCount=0;
		if (++Vref1 > 5000) Vref1 = 0;
		}
	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 0;
}

//@} //defgroup
// end of file




