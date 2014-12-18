#ifndef __LCD_H
#define __LCD_H

#define RS  LATEbits.LATE0
#define RW  LATEbits.LATE1
#define E   LATEbits.LATE2
#define clear_screen 0x01
#define right_screen 0xC0
#define left_screen 0x80
#define LEFT 1
#define RIGHT 0

void LCDinit(void);
void LCDcommand(unsigned char byte);
void LCDwrite(unsigned char data);
void DISPLAYadVALUE(unsigned int ADCresult, int side);
void LCDmessage(const char *message, int side);
void DISPLAYint(signed int integer, int side);
void DISPLAYfloat(const char *message, double number, unsigned char temp_flag);

#endif