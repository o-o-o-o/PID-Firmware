#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <plib/delays.h>
#include <plib/adc.h>
#include <plib/pwm.h>
#include <plib/timers.h>
#include <plib/usart.h>
#include <plib/i2c.h>
#include <plib/EEP.h>
#include "LCD.h"
#include "PIDinitialize.h"

#define DOWN PORTBbits.RB4
#define UP PORTBbits.RB5
#define NEXT PORTAbits.RA3//PORTBbits.RB6
#define SELECT PORTAbits.RA4 //PORTBbits.RB7

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
#define BUTTON_ON 0
#define BUTTON_OFF 1

#define targetADDR 0x0010
#define pADDR 0x0020
#define iADDR 0x0030
#define dADDR 0x0040

void PIDinitialize(void){
    const char *message = "Welcome";
    unsigned char ADCconfig1 = 0x00, ADCconfig2 = 0x00, portconfig = 0x00;

    unsigned char TMR0config1 = 0x00, TMR0config2 = 0x00;
    unsigned int TMR0_value=0x00;

    unsigned char TMR1config1 = 0x00;
    unsigned int TMR1_value = 0x00;

    unsigned char USARTconfig = 0;
    unsigned int spbrg = 0;


 /*********************************
 *I2C Variables
 ********************************/

    unsigned char sync_mode=0x00, slew=0x00, i2cadd, i2cjunk;
    signed char i2cstatus;
    signed int i2c_recv_0 = 128;
    signed int i2c_recv_1 = 64;

/********************************
 *SFRs
 ****************************/
    TRISA = 0b11111111;
    TRISB = 0b11110000;
    TRISC = 0b00000000;
    TRISD = 0b00000000;
    TRISE = 0b00000000;

    LATB = 0b00110000;  //buzzer off, peltier in heating mode, relay off
    LATC = 0b00000000;
    LATD = 0b00000000;
    LATE = 0b00000000;

    INTCON = 0b00000000;
    INTCON2 = 0b00000000;
    INTCON3 = 0b00000000;

    PIR1 = 0b00000000;
    PIR2 = 0b00000000;

    PIE1 = 0b00000000;
    PIE2 = 0b00000000;

//Initialize ADC
    ADCconfig1 = ADC_FOSC_4 & ADC_RIGHT_JUST & ADC_12_TAD;
    ADCconfig2 = ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS;
    portconfig = ADC_3ANA;

    OpenADC(ADCconfig1, ADCconfig2, portconfig);

//Initialize PWM
    OpenPWM1(0x80); //set PWM frequency
    SetDCPWM1(0);

//Initialize LCD
    LCDinit();      //Initialize LCD hardware
    LCDmessage(message, LEFT);


//Initialize Timer0
    TMR0config1 = TIMER_INT_ON & T0_SOURCE_INT & T0_PS_1_32 & T0_16BIT & T0_EDGE_RISE;
    WriteTimer0(TMR0_value);
    OpenTimer0(TMR0config1);

//Initialize Timer1
    TMR1config1 = TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8;
    WriteTimer1(TMR1_value);
    OpenTimer1(TMR1config1);

//Initialize USART
    USARTconfig = USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT
            & USART_BRGH_HIGH & USART_CONT_RX;
    spbrg = 25;
    OpenUSART(USARTconfig, spbrg);
    PIE1bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

 //Initialize I2C
    sync_mode = MASTER;
    slew = SLEW_OFF;

    OpenI2C(sync_mode, slew);
    SSPADD = 0x09;  //Fosc is 4 MHz, I2C clock is 100KHz

    PELTIER = HEAT;
    BUZZER = QUIET;
/*
    if(SELECT == 1){
        Write_b_eep(targetADDR, 250); //Select
        Busy_eep();
        Write_b_eep(pADDR, 10);
        Busy_eep();
        Write_b_eep(iADDR, 0); //Select
        Busy_eep();
        Write_b_eep(dADDR, 10);
        Busy_eep();
    }
 */
}
