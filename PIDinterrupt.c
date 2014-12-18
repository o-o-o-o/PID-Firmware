/*******************************************/
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <plib/delays.h>
#include <plib/adc.h>
#include <plib/pwm.h>
#include <plib/timers.h>
#include <plib/usart.h>
#include <plib/i2c.h>
#include <ctype.h>
#include "LCD.h"
/**********************************************/

/**********************************************/
#define PELTIER LATBbits.LATB3
#define RELAY LATBbits.LATB2
#define BUZZER LATBbits.LATB1
#define RELAY4 LATBbits.LATB0
#define RELAY_ON 1
#define RELAY_OFF 0
#define HEAT 0
#define COOL 1
#define LOUD 1
#define QUIET 0
/**********************************************/

double READtemperature(unsigned char address);

void interrupt high_isr(void)
{
    extern volatile double reading[8];
    extern volatile double *p;
    extern volatile double PV;
    extern double SP;
    extern double Kp;
    extern double Ki;
    extern double Kd;
    extern volatile double PVold;
    extern volatile unsigned char flag;
    extern volatile unsigned char dataread;
    extern volatile unsigned int analogtemp;
    extern unsigned char rx_done;
    static unsigned char i = 0;
    unsigned char j;
    double sum = 0;
    static unsigned char rx_index;
    static unsigned char load;
    unsigned char buffer;
    extern volatile unsigned char Rxdata[8];

    if (PIE1bits.RCIE && PIR1bits.RCIF){
        buffer = RCREG;
        if (islower(buffer)){
            rx_index = 0;
            rx_done = 0;
        }

        if (buffer == 'c'){
            PELTIER = COOL;
            rx_index = 7;
        }

        if (buffer == 'h'){
            PELTIER = HEAT;
            rx_index = 7;
        }

        Rxdata[rx_index++ % 8] = RCREG;

        if(rx_index == 6){
 //           Rxdata[5] = ' ';
            Rxdata[6] = ' ';
            Rxdata[7] = '\0';
            if(Rxdata[0] == 't'){
                Rxdata[0] = ' ';
                SP = (strtod(Rxdata, NULL)/10);
            }
            if(Rxdata[0] == 'p'){
                Rxdata[0] = ' ';
                Kp = (strtod(Rxdata, NULL)/10);
            }
            if(Rxdata[0] == 'i'){
                Rxdata[0] = ' ';
                Ki = (strtod(Rxdata, NULL)/10);
            }
            if(Rxdata[0] == 'd'){
                Rxdata[0] = ' ';
                Kd = (strtod(Rxdata, NULL)/10);
            }

        }

 //       T0CONbits.PSA = 1; // disable prescaler on tmr0

    }


    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF){
        INTCONbits.TMR0IF = 0;  //clear interrupt flag

        reading[((i++) %8)] = READtemperature(0);
        for(p = &reading[0]; p <= &reading[7] ; p++){
            sum += *(p);
        }

        PVold = PV;
        PV = sum/8;

        dataread = 1;
        WriteTimer0(1);
    }

    return;
}

double READtemperature(unsigned char address){
 /********************************************************************************
 * I2C sequence.
 *     -reads a 16 bit register from I2C slave device (an MCP9808 temp sensor)
 *     -Further refinements may be necessary
 **************************************************************************************/
        signed int upperbyte = 128;
        signed int lowerbyte = 64;
        unsigned char i2cadd, i2cjunk;
        signed char i2cstatus;
        double temperature;

        i2cadd = (0b00110000 | address);  //address of MCP9808 temp sensor, lsb is r/w (1/0) from slave

        IdleI2C();
        StartI2C();
        i2cjunk = SSPBUF; //Its a good idea to clear the buffer initially

        do {
            i2cstatus = WriteI2C( i2cadd | 0x00 ); //write the address of slave
            if(i2cstatus == -1) {   //check if bus collision happened
               i2cjunk = SSPBUF; //upon bus collision detection clear the buffer,
               SSPCON1bits.WCOL = 0; // clear the bus collision status bit
               LB0 = 0;  //Turn on LED to mark event of i2cstatus == -1
            }
        } while(i2cstatus != 0); //write until successful communication


        IdleI2C();
        WriteI2C(0b00000101);
        IdleI2C();
        StartI2C();
        WriteI2C( i2cadd | 0x01);
        IdleI2C();
        upperbyte = (ReadI2C() & 0b00011111);
        AckI2C();
        lowerbyte = ReadI2C();
        NotAckI2C();
        StopI2C();

 /**********************************************************
 *End of I2C sequence
 ***********************************************************/
        if ((upperbyte & 0b00010000) == 0b00010000){    //TA < 0°C
            upperbyte = upperbyte & 0x0F;   //Clear SIGN
            temperature = (256 - ((((double)upperbyte)*16) + ((double)lowerbyte)/16));
        } else { //TA > 0°C
            temperature = ((((double)upperbyte)*16) + ((double)lowerbyte)/16);
        }//Temperature = Ambient temperature


    return temperature;
}