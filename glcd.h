#ifndef GLCD_H
#define	GLCD_H

#include <pic18f4550.h>
#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


/*============================================================================*
 *                                 GLCD Module                                *
 *============================================================================*/

#define 	LCD_WIDTH			128
#define 	LCD_HEIGHT			64

#define 	GLCD_CS1			LATBbits.LATB0
#define 	GLCD_CS2			LATBbits.LATB1
#define 	GLCD_RS				LATBbits.LATB2
#define 	GLCD_RW				LATBbits.LATB3
#define 	GLCD_E				LATBbits.LATB4
#define 	GLCD_RST			LATBbits.LATB5

#define 	GLCD_DATA_TRIS		TRISD
#define 	WR_DATA				LATD
#define 	RD_DATA				PORTD

#define 	GLCD_LEFT   	 	0
#define 	GLCD_RIGHT  	 	1

#define 	GLCD_OFF			0
#define 	GLCD_ON				1

#define		OUT					0
#define		IN					1

#define		f3X6				0
#define		f8X8				1



extern void glcd_Init(unsigned char mode);
extern void glcd_WriteByte(unsigned char side, unsigned char data);
extern unsigned char glcd_ReadByte(unsigned char side);
extern void glcd_PlotPixel(unsigned char x, unsigned char y, unsigned char color);
extern void glcd_SetCursor(unsigned char xpos,unsigned char ypos);
void glcd_Rect(unsigned char xs, unsigned char ys, unsigned char xe ,unsigned char ye,unsigned char color);
extern void glcd_FillScreen(unsigned char color);
extern void glcd_WriteChar8X8( unsigned char ch, unsigned char color);
extern void glcd_WriteChar3x6( unsigned char ch, unsigned char color);
extern void glcd_WriteString(const char str[],unsigned char font,unsigned char color);
extern void glcd_Image(void);
void glcd_text_write(const char str[], unsigned char x, unsigned char y);


#endif	/* GLCD_H */
