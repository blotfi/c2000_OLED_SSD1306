/* 
 File:   oled.h
 L. BAGHLI 03/2018
 Adapted from a dspic code
 */
#include <stdint.h>
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_I2c_defines.h"              // Macros used for I2C examples.

#ifndef OLED_H
#define	OLED_H

#define I2C_SLAVE_ADDR       0x3c	

//-----------------------------------------------------------------------------
// I2C OLED Routines
#define kI2C_100KHZ     (0x00)
#define kI2C_400KHZ     (0x02)

void InitI2C();
interrupt void i2c_int1a_isr(void);

void i2c1Reset();

void oledCommand( Uint8 ch );
void oledDisplayOffset( Uint8 offset );
void oledData( Uint8 data );
void oledGotoYX(unsigned char Row, unsigned char Column);
void oledPutChar( char ch );
void oledPutHexa( unsigned int Var );
void oledPutDec( unsigned int Var );
void oledPrint( char *s );
void oledClear();
void oledInit();
void __delay_ms(unsigned long t);

#endif	/* OLED_H */

