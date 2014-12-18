/*********************************************
 * File:   PIDmain.c
 * Author: Dave Perry
 *
 * Created on February 14, 2014, 9:37 PM
 ******************************************/


/*******************************************/
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <ctype.h>
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
#define NEXT PORTAbits.RA3 //PORTBbits.RB6//
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
#define MENU_TIME 16
#define BUTTON_ON 0
#define BUTTON_OFF 1

#define targetADDR 0x0010
#define pADDR 0x0020
#define iADDR 0x0030
#define dADDR 0x0040
#define highADDR 0x0050
#define lowADDR 0x0060



/**********************************************
* Forward Declarations
/**********************************************/
void SENDdouble(double number);
void SENDint(signed int number);
void CHECKusartERRORS(void);
double Perror(double PV, double SP);
double Ierror(double PV, double SP);
double Derror(double PV, double PVold);
void COMPUTEpid(void);
double READstring(char * input);
void SENDdoubleTEST(double number);
void BEEPS(void);
signed int CHECKbuttons(void);
void LOGdata(void);
void MENUzero(void);
void MENUone(void);
void MENUtwo(void);
void MENUthree(void);
void MENUfour(void);
void MENUfive(void);
void MENUsix(void);
void MENUseven(void);
void MENUeight(void);
void MENUtimer(void);
signed int READeeprom(unsigned int address);
void WRITEeeprom(unsigned int address, signed int data);
void CHECKalarms(void);
char READtime(void);

/**********************************************
 *Menu variable
 *********************************************/
    unsigned char menu_index = 0;
    unsigned char button_index = 0;
    const char *Saved = "Saved   ";
    unsigned char menu_timer = 0;

/*****************************************
 *These variables contain ADC results
 ****************************************/
    unsigned int light = 0;
    unsigned int ADCresult0 = 500;
    unsigned int ADCresult1 = 500;
    volatile unsigned int analogtemp = 230;

/*****************************************
 *32 element array for 32-count running average of PV
 ****************************************/
    volatile double reading[8] = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    volatile double *p = &reading[0];
    volatile unsigned char dataread = 0;
    unsigned char i = 0;


/**************************************************
*PID variables
*   -PV is Process Variable (i.e. input measurement, e.g temperature)
*    and in this implementation we do a running average of 32 readings
*   -SP is Set Point, i.e. the target for PV
*   -MV is the Moving Variable (i.e the physical output of the process
*    e.g. PWM duty cycle
***************************************************/
    volatile double PV = 25.0;
    volatile double PVold = 25.0; // second to last PV reading, for d(pv)/dt
    double SP = 30.0;
    double MV = 0;
    double displayed_temp;
    volatile unsigned int prescaler = 4;
    double highalarm;
    double lowalarm;

    unsigned int MVint;
    
    double errorp = 0.0;
    double errori = 0.0;
    double errord = 0.0;

    double Kp = 1;
    double Ki = 0;
    double Kd = 0;

    double sample_freq = 15;
    double sample_period = 0.005;

    volatile unsigned char Txdata[] = "what is it?\n";
    volatile unsigned char Rxdata[8] = "40";
    volatile unsigned char over_run[] = "over run\n";
    volatile unsigned char interrupd[] = "interrupt\n";
    volatile unsigned char frame_error[] = "frame error\n";
    volatile unsigned char flag = 0;
    unsigned char rx_done = 0;

/********************
 *EEPROM
 *******************/
    char ee_buffer[10];    //defined in PIDinit
    signed int testee;

main(void) {
    PIDinitialize();
    BEEPS();
 //   di();

    if(SELECT == BUTTON_ON){
        WRITEeeprom(targetADDR, 150); //Select
        WRITEeeprom(pADDR, 1000);
        WRITEeeprom(iADDR, 10); //Select
        WRITEeeprom(dADDR, 1000);
        WRITEeeprom(lowADDR, 10);
        WRITEeeprom(highADDR, 1010);
    }
    
    SP = ((double)READeeprom(targetADDR))/10;
    Kp = ((double)READeeprom(pADDR))/10;
    Ki = ((double)READeeprom(iADDR))/10;
    Kd = ((double)READeeprom(dADDR))/10;
    lowalarm = ((double)READeeprom(lowADDR))/10;
    highalarm = ((double)READeeprom(highADDR))/10;

    while(1)  //main loop
    {
            if(menu_index == 0){
                MENUzero();  // menu one!!
            } else if (menu_index == 1){
                MENUone();
            } else if (menu_index == 2){
                MENUtwo();
            } else if (menu_index == 3){
                MENUthree();
            } else if (menu_index == 4){
                MENUfour();
            } else if (menu_index == 5){
                MENUfive();
            } else if (menu_index == 6){
                MENUsix();
            } else if (menu_index == 7){
                MENUseven();
            } else if (menu_index == 8){
                MENUeight();
            } else if (menu_index > 8){
                menu_index = 0;         //wrap back to menu zero
            }
    }
    return 0;
}




/********************************************************************
 * Functions called by the main loop
 *******************************************************************/

/********************************
 *Serial communication
 *********************************/
void SENDdouble(double number){
    char *double_buffer;
    int double_status;
    double_buffer = ftoa(number, &double_status);
    while(BusyUSART());
    putsUSART(double_buffer);
    while(BusyUSART());
    WriteUSART('\t');
}

void SENDint(signed int number){
    signed char index = 0;
    char int_buf[6];

    if (number < 0) {
        while(BusyUSART());
        WriteUSART('-');
        number = -number;
    }
    itoa(int_buf, number, 10);
    while(isdigit(int_buf[index])){
        while(BusyUSART());
        WriteUSART(int_buf[index]);
        index += 1;
    }
    while(BusyUSART());
    WriteUSART('\t');
}

void CHECKusartERRORS(void){
    unsigned char trash;

    if (RCSTAbits.OERR){
        while(BusyUSART());
        putsUSART((char *)over_run);
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }

    if (RCSTAbits.FERR){
        while(BusyUSART());
        putsUSART((char *)frame_error);
        trash = RCREG;
    }

    if (flag == 1){
        while(BusyUSART());
        putsUSART((char *)interrupd);
        flag = 0;
    }
}

void SENDdoubleTEST(double number){ //this is same as send double except...
    char *double_buffer;    //...it truncates the decimal to the tenths place
    int double_status;
    double_buffer = ftoa(number, &double_status);
    *(double_buffer+4) = '\0';
    while(BusyUSART());
    putsUSART(double_buffer);
    while(BusyUSART());
    WriteUSART('\t');
}


void LOGdata(void){
        SENDdouble(MV);
        SENDdouble(PV);
        SENDdouble(Perror(PV, SP));
        SENDdouble(Ierror(PV, SP));
        SENDdouble(Derror(PV, PVold));
        SENDint(analogtemp);
        while(BusyUSART());
        WriteUSART('\n');
}

void BEEPS(void){
    static char i;
    for(i = 0; i <10; i++){
           Delay1KTCYx(100);
           BUZZER = LOUD;
           Delay1KTCYx(100);
           BUZZER = QUIET;
    }
}

double READstring(char * input){
    return (strtod(input, NULL));
}

/*********************************************
 *See which buttons are pressed
 * -Pressing more than one button at a time may cause unexpected behavior
 *********************************************/
signed int CHECKbuttons(void){
    if (UP == BUTTON_ON){
        Delay1KTCYx(250); //debounce 250 ms
        return 1;
    } else if (DOWN == BUTTON_ON){
        Delay1KTCYx(250); //debounce 250 ms
        return 2;
    } else if (NEXT == BUTTON_ON){
        Delay1KTCYx(250); //debounce 250 ms
        return 3;
    } else if (SELECT == BUTTON_ON){
        Delay1KTCYx(250); //debounce 250 ms
        return 4;
    } else {
        return 0;
    }

}



/*******************************
 *PID computations
 ******************************/
void COMPUTEpid(void){
        if (PORTBbits.RB3 == HEAT){
            MV = Perror(PV, SP) + Ierror(PV, SP) + Derror(PV, PVold);
        } else {
            MV = -(Perror(PV, SP) + Ierror(PV, SP) + Derror(PV, PVold));
        }
        if (MV < 0) {
            MV = 0;
        }
        if (MV > 1022) {
            MV = 1022;
        }

        MVint = (signed int) MV;    //should be unsigned int?
        SetDCPWM1(MVint);
}

double Perror(double PV, double SP){
    double diff = (SP - PV);
    return (Kp * diff);

}

double Ierror(double PV, double SP){
    static double integral;
    static double mult;
    static double integ;
    mult = (double)prescaler/8;
    integ = ((Ki * (SP - PV)));

        integral += (integ * mult);  //sample_freq * KI should be done only once

        if(integral >= 1022){
            integral = 1022;
        }
        if (integral <= -1022){
            integral = -1022;
        }
        return integral;
}

double Derror(double PV, double PVold){
    double derivative = 0;
    derivative = Kd * (PVold - PV);
    return derivative;
}

/*******************************
 *MENU function
 *********************************/
void MENUzero(void){
    const char *message = "Temp    ";
    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 0){
        if(dataread == 1){
            dataread = 0;
            COMPUTEpid();

            SelChanConvADC(ADC_CH0); //do AD conversion
            while(BusyADC());
            analogtemp = (unsigned int) ReadADC(); //put reading in ADCresult1

            DISPLAYfloat(message, PV, 1);  // display temperature
            LOGdata();
            CHECKalarms();
            MENUtimer();
            
        }

        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    break;  //UP button
            case 2: menu_timer = 0;
                    break;  //Down button
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    break;
            default: break;
        }
    }
 }

void MENUone(void){
    const char *message = "target  ";
    double target;
    target = SP;

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 1){
        DISPLAYfloat(message, target, 0);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    target += 0.1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    target -= 0.1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(targetADDR, (signed int)target*10);
                    SP = target;
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen);
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
 }

void MENUtwo(void){
    const char *message = "Kp coeff";
    double Ptarget;
    Ptarget = Kp;

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 2){
        DISPLAYfloat(message, Ptarget, 0);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    Ptarget += 1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    Ptarget -= 1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(pADDR, (signed int)Ptarget*10);
                    Kp = Ptarget;
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUthree(void){
    const char *message = "Ki coeff";
    double Itarget;
    Itarget = Ki;

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 3){
        DISPLAYfloat(message, Itarget, 0);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    Itarget += 0.1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    Itarget -= 0.1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(iADDR, (signed int)Itarget*10);
                    Ki = Itarget;
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}
void MENUfour(void){
    const char *message = "Kd coeff";
    double Dtarget;
    Dtarget = Kd;

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 4){
        DISPLAYfloat(message, Dtarget, 0);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    Dtarget += 0.1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    Dtarget -= 0.1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(dADDR, (signed int)Dtarget*10);
                    Kd = Dtarget;
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUfive(void){
    const char *message = "S. freq";
    unsigned char tcon_copy;

    prescaler = (0b00000111 & T0CON);

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 5){
        DISPLAYfloat(message, prescaler, 0);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    prescaler = (prescaler + 1) % 8;  //UP button
                    break;
            case 2: menu_timer = 0;
                    prescaler = (prescaler - 1) % 8;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                  //  WRITEeeprom(dADDR, (signed int)Dtarget*10);
                    tcon_copy = (T0CON & 0b11111000);
                    T0CON = (prescaler | tcon_copy);
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUsix(void){
    const char *message = "Low     ";

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 6){
        DISPLAYfloat(message, lowalarm, 1);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    lowalarm += 1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    lowalarm -= 1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(lowADDR, (signed int)lowalarm*10);
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUseven(void){
    const char *message = "High     ";

    menu_timer = 0;
    LCDcommand(clear_screen);

    while(menu_index == 7){
        DISPLAYfloat(message, highalarm, 1);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    highalarm += 1;  //UP button
                    break;
            case 2: menu_timer = 0;
                    highalarm -= 1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index += 1;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(highADDR, (signed int)highalarm*10);
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUeight(void){
    const char *message = "Time";
    float time;

    menu_timer = 0;
    LCDcommand(clear_screen);

    time = (float)READtime();
    DISPLAYfloat(message, time, 1);

    while(menu_index == 8){
        DISPLAYfloat(message, time, 1);
        button_index = CHECKbuttons();
        switch (button_index){
            case 1: menu_timer = 0;
                    time = (float)READtime();
                    break;
            case 2: menu_timer = 0;
                    highalarm -= 1;  //Down button
                    break;
            case 3: menu_timer = 0;
                    menu_index = 0;  //NEXT button pressed
                    break;
            case 4: menu_timer = 0;
                    WRITEeeprom(highADDR, (signed int)highalarm*10);
                    LCDcommand(clear_screen);
                    LCDmessage(Saved, LEFT);
                    Delay10KTCYx(255);
                    LCDcommand(clear_screen); //could be the culprit fot blank screen when changing menus
                    menu_index = 0;
                    break;
            default: break;
        }
        MENUtimer();
    }
}

void MENUtimer(void){
        if(PIR1bits.TMR1IF == 1){
            PIR1bits.TMR1IF = 0;
            menu_timer += 1;
            if(menu_timer == MENU_TIME){
                menu_index = 0;
                menu_timer = 0;
            }
            WriteTimer1(1);
        }

}

/********************************************
 *EEPROM
 *******************************************/
signed int READeeprom(unsigned int address){
    unsigned char i = 0;
    for(i = 0; i<10; i++){
        ee_buffer[i] = Read_b_eep(address+i);
    }
    return strtol(ee_buffer, NULL, 10);

}

void WRITEeeprom(unsigned int address, signed int data){
    unsigned char i = 0;
    itoa(ee_buffer, data, 10);
    for(i = 0; i<10; i++){
        Write_b_eep(address+i, ee_buffer[i]);
        Busy_eep();
    }


}

/******************************************
 *alarm
 **************************************/
void CHECKalarms(void){
    const char *messageL = "Low Temp";
    const char *messageH = "High Tem";
    const char *messageRL = "erature ";
    const char *messageRH = "perature";
    signed int anykey = 0;

    if (PV < lowalarm){
        LCDmessage(messageL, LEFT);
        LCDmessage(messageRL, RIGHT);
        BUZZER = LOUD;
        MV = 0;
        while (anykey == 0){
            anykey = CHECKbuttons();
        }
        BUZZER = QUIET;
    }

    if (PV > highalarm){
        LCDmessage(messageH, LEFT);
        LCDmessage(messageRH, RIGHT);
        BUZZER = LOUD;
        MV = 0;
        while (anykey == 0){
            anykey = CHECKbuttons();
        }
        BUZZER = QUIET;
    }
}

char READtime(void){
 /********************************************************************************
 * I2C sequence.
 *     -reads from MCP7940N Real Time Clock
 *     -Further refinements may be necessary
 **************************************************************************************/
        char seconds;
        unsigned char rtc_addr, rtc_junk;
        signed char rtc_status = 0;
        double time;

        rtc_addr = 0b11011110;  //address of MCP7940N,  lsb is r/w (1/0) from slave

        DISPLAYint(23,1);
  //      IdleI2C();
        StartI2C();
        rtc_junk = SSPBUF; //Its a good idea to clear the buffer initially

        do {
            DISPLAYint(25,1);
            rtc_status = WriteI2C(rtc_addr); //write the address of slave
            if(rtc_status == -1) {   //check if bus collision happened
               DISPLAYint(26,1);
               rtc_junk = SSPBUF; //upon bus collision detection clear the buffer,
               SSPCON1bits.WCOL = 0; // clear the bus collision status bit
               LB0 = 0;  //Turn on LED to mark event of i2cstatus == -1
            }
        } while(rtc_status != 0); //write until successful communication


        IdleI2C();
        WriteI2C(0b00000000);
        IdleI2C();
        WriteI2C(0b10000000);
        IdleI2C();

        DISPLAYint(27,1);

        StartI2C();
        WriteI2C(rtc_addr);
        IdleI2C();
        WriteI2C(0b00000000);
        IdleI2C();

        DISPLAYint(28,1);

        StartI2C();
        WriteI2C(0b11011111);
        IdleI2C();

     //   DISPLAYint(29,1);
        
        seconds = ReadI2C();
        AckI2C();
        seconds = ReadI2C();
        NotAckI2C();
        StopI2C();
        return seconds;

 /**********************************************************
 *End of I2C sequence
 ***********************************************************/
}