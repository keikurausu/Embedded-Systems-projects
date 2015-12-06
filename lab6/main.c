#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <stdio.h>
#include <libpic30.h>
#include <stdlib.h>
#include <time.h>

#include "lcd.h"
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "flextouch.h"


/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);

volatile int flag = 1;
void __attribute__((__interrupt__)) _T2Interrupt(void)
{
    flag = 1;

   //CLEARBIT(T2CONbits.TON); // Disable Timer
    IFS0bits.T2IF = 0; // clear the interrupt flag
}

int main(){
    //Init LCD
    __C30_UART=1;
    lcd_initialize();
    lcd_clear();

    //setup Timer 2
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    SETBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit

    PR2 = 2000; // Set timer period 10ms:
    SETBIT(T2CONbits.TON); // Disable Timer

    //disable ADC
    CLEARBIT(AD1CON1bits.ADON);
    //initialize PIN
    //y-axis
    SETBIT(TRISBbits.TRISB9); //set TRISE RE8 to input
    CLEARBIT(AD1PCFGHbits.PCFG20); //set AD1 AN20 input pin as analog
    //x-axis
    SETBIT(TRISBbits.TRISB15); //set TRISE RE8 to input
    //CLEARBIT(AD1PCFGHbits.PCFG15); //set AD1 AN20 input pin as analog
    //Configure AD1CON1
    SETBIT(AD1CON1bits.AD12B); //set 10b Operation Mode
    AD1CON1bits.FORM = 0; //set integer output
    AD1CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD1CON2
    AD1CON2 = 0; //not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD1CON3bits.ADRC); //internal clock source
    AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD1CON1bits.ADON);


    touch_init();

    uint16_t a= 0, i= 0, j=0;

    lcd_locate(0,0);
    lcd_printf("Ball Y pos = ");
    lcd_locate(0,1);
    lcd_printf("Ball X pos = ");

    uint16_t x_vals[5];
    uint16_t y_vals[5];
    int loopcount = 0;
    
    while(1)
    {

        touch_select_dim(0);
       
        

        __delay_ms(10);
        for(i = 0; i< 5; i++)
        {
             x_vals[i] = 0;
            x_vals[i] =  touch_adc();
        }
		//bubble sort
        for(i = 0; i<5; i++)
        {
            for(j = i+1; j<5; j++)
            {
                if(x_vals[i] > x_vals[j])
                {
                    //swap
                    a = x_vals[i];
                    x_vals[i] = x_vals[j];
                    x_vals[j] = a;
                }
            }
        }

        //do 5 reading for y
        touch_select_dim(1);
       // _Delay_ms(10);
       __delay_ms(10);
       
        for(i = 0; i< 5; i++)
        {
             y_vals[i] = 0;
            y_vals[i] =  touch_adc();
        }
		//bubble sort
        for(i = 0; i<5; i++)
        {
            for(j = i+1; j<5; j++)
            {
                if(y_vals[i] > y_vals[j])
                {
                    //swap
                    a = y_vals[i];
                    y_vals[i] = y_vals[j];
                    y_vals[j] = a;
                }
            }
        }
		//x_vals[2] and y_vals[2] hold median values
		//print results to lcd
		//@@TODO

        if (loopcount == 10)
        {
            loopcount = 0;

            lcd_locate(13,0);
            lcd_printf("    ");

            lcd_locate(13,1);
            lcd_printf("    ");

            lcd_locate(13,0);
            lcd_printf("%d",x_vals[2]);

            lcd_locate(13,1);
            lcd_printf("%d",y_vals[2]);


        }
        loopcount++;
    }

    return 0;
}

