#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <plib/delays.h>
#include <plib/adc.h>
#include <plib/pwm.h>
#include <plib/timers.h>
#include <ctype.h>
#include "LCD.h"

#define SHORT 1  //usually 1
#define LONG 3   //usually 3
/********************************************************************
 * Functions called by the main loop
 *******************************************************************/
void LCDinit(void)
{
    E = 0;
    Delay10KTCYx(10);  //~0.1 second delay - If problems occur, change to 1.0 seconds
    LCDcommand(0x30);   //Note that there are two 0x30 command sent, that's on purpose. Sometimes a third is recommended
    LCDcommand(0x30);
    LCDcommand(0x38);
    LCDcommand(0x10);
    LCDcommand(0x0C);
    LCDcommand(0x01);
    LCDcommand(0x06);
}

void LCDcommand(unsigned char command)
{
    LATD = command;
    RS = 0;
    RW = 0;
    E = 1;
    Delay10TCYx(SHORT);
    E = 0;
    Delay1KTCYx(LONG);
}

void LCDwrite(unsigned char data)
{
    LATD = data;
    RS = 1;
    RW = 0;
    E = 1;
    Delay10TCYx(SHORT);
    E = 0;
    Delay1KTCYx(LONG);
}

void DISPLAYadVALUE(unsigned int ADCresult, int side)
{
        unsigned char buffer[4];
        unsigned char i = 0;

        utoa(buffer, ADCresult, 10); //convert to string and store in buffer

        if (side == 0) {
            LCDcommand(right_screen);
        }
        else {
            LCDcommand(left_screen);
        }

        while(buffer[i] >= 48 && buffer[i] <= 57) //ascii values for 0-9 are 48-57
            {
                LCDwrite(buffer[i]);
                i++;
            }
}

void LCDmessage(const char *message, int side)
{
    unsigned char j=0;    

    if (side == 0) {
        LCDcommand(right_screen);
    } else {
        LCDcommand(left_screen);
    }

    for(j = 0; j < 8;j++){
        if(isalnum(*(message+j))){
            LCDwrite(*(message+j));
        } else {
            LCDwrite(0b10100000);
        }
    }
}

void DISPLAYint(signed int integer, int side)
{
        unsigned char buffer[5];
        unsigned char i = 0;

    //    utoa(buffer, integer, 10); //convert to string and store in buffer

        if (side == 0) {
            LCDcommand(right_screen);
        }
        else {
            LCDcommand(left_screen);
        }

        if (integer < 0) {
            LCDwrite(0b00101101);
            integer = -integer;
        }
        
        utoa(buffer, integer, 10); //convert to string and store in buffer
        

        while(buffer[i] >= 48 && buffer[i] <= 57) //ascii values for 0-9 are 48-57
            {
                LCDwrite(buffer[i]);
                i++;
                if (i == 5) {
                    break;
                }
            }
}

void DISPLAYfloat(const char *message, double number, unsigned char temp_flag){
    signed int times_ten;
    unsigned char buffer[5];
    LCDmessage(message, LEFT);

    LCDcommand(right_screen);

    times_ten = (signed int)(number * 10);
    if (times_ten < 0) {
        LCDwrite(0b00101101);
        times_ten = -times_ten;
    } else {
        LCDwrite(0b10100000);
    }
    itoa(buffer, times_ten, 10);

    if (times_ten < 10000 && times_ten >= 1000){
        LCDwrite(buffer[0]);
        LCDwrite(buffer[1]);
        LCDwrite(buffer[2]);
        LCDwrite(0b00101110);
        LCDwrite(buffer[3]);
        if(temp_flag == 1){
            LCDwrite(0b11011111);
            LCDwrite('C');
        } else{
            LCDwrite(0b10100000);
            LCDwrite(0b10100000);
        }
        LCDwrite(0b10100000);
    }
    if (times_ten < 1000 && times_ten >= 100){
        LCDwrite(buffer[0]);
        LCDwrite(buffer[1]);
        LCDwrite(0b00101110);
        LCDwrite(buffer[2]);
        if(temp_flag == 1){
            LCDwrite(0b11011111);
            LCDwrite('C');
        } else{
            LCDwrite(0b10100000);
            LCDwrite(0b10100000);
        }
        LCDwrite(0b10100000);
    }

    if (times_ten < 100 && times_ten >= 10){
        LCDwrite(buffer[0]);
        LCDwrite(0b00101110);
        LCDwrite(buffer[1]);
        if(temp_flag == 1){
            LCDwrite(0b11011111);
            LCDwrite('C');
        } else{
            LCDwrite(0b10100000);
            LCDwrite(0b10100000);
        }
        LCDwrite(0b10100000);
        LCDwrite(0b10100000);
    }

    if (times_ten < 10){
        LCDwrite('0');
        LCDwrite(0b00101110);
        LCDwrite(buffer[0]);
        if(temp_flag == 1){
            LCDwrite(0b11011111);
            LCDwrite('C');
        } else{
            LCDwrite(0b10100000);
            LCDwrite(0b10100000);
        }
        LCDwrite(0b10100000);
        LCDwrite(0b10100000);
    }
}