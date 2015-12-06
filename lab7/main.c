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
#include "flexmotor.h"
#include <math.h>

#define XMAX 2900
#define YMAX 2631
#define XMIN 370
#define YMIN 480
#define XMID (XMAX + XMIN) /2
#define YMID (YMAX + YMIN) /2


/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);

volatile int flag = 0;
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    flag = 1;

   
    IFS0bits.T1IF = 0; // clear the interrupt flag
}

double findDutyRange(uint16_t amax, uint16_t amin, uint16_t avalue)
{
    double value = avalue;
    double max = amax;
    double min = amin;
    if(value >= max)
        return 2.1;

    if(value <= min)
        return 0.9;

    return ( (double) ((((value - min)/(max-min))*1.2) + 0.9) );
}




int main(){
    //Init LCD
    __C30_UART=1;
    lcd_initialize();
    lcd_clear();

    //setup Timer 2
    CLEARBIT(T1CONbits.TON); // Disable Timer
    CLEARBIT(T1CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T1CONbits.TGATE); // Disable Gated Timer mode
    TMR1 = 0x00; // Clear timer register
    T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T1IF); // Clear Timer2 interrupt status flag
    SETBIT(IEC0bits.T1IE); // Enable Timer2 interrupt enable control bit

    PR1 = 10000; // Set timer period 10ms:
    SETBIT(T1CONbits.TON); // Enable





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

    motor_init(0);
    motor_init(1);

    touch_init();

    uint16_t a= 0, i= 0, j=0;




    uint16_t x_vals[5];
    uint16_t y_vals[5];
    float const Kp = 0.5;
    float const Kd = -0.1;
    float const Ki = 1;
    uint16_t PconX;
    uint16_t PconY;
    uint16_t DconX;
    uint16_t DconY;
    float IconX = 0;
    uint16_t IconY;
    uint16_t currloc;
    uint16_t prevloc;
    float vel, print;
    uint16_t Fx;

    
    lcd_locate(0,0);
    lcd_printf("Fx = ");
    lcd_locate(0,1);
    lcd_printf("Ix = ");
    lcd_locate(0,2);
    lcd_printf("Dx = ");
    lcd_locate(0,3);
    lcd_printf("Px = ");
    lcd_locate(0,4);
    lcd_printf("set x = %d", XMID);
    lcd_locate(0,5);
    lcd_printf("Ki = %f",Ki);
    lcd_locate(0,6);
    lcd_printf("Kp= %.4f Kd = %.4f ",Kp,Kd);
    
    lcd_locate(0,7);
    lcd_printf("Pos = ");


    int loopcount = 0;
    motor_set_duty( 1, findDutyRange( YMAX,  YMIN,  YMAX));
    while(1)
    {
        while(flag != 1) {} // if 50 ms is not up sit here and wait
        flag = 0;


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
        //set the motor values
//
       //calculate proportional control
       // get the difference and translate values so they are all positive
       PconX =  (uint16_t)((((float)XMID - (float)x_vals[2]) * Kp)- (float)(XMID - XMAX));

       //find the velocity. change in X position over change in time(0.05s)
       vel = Kd*(((float)x_vals[2] - (float)prevloc) / .05);
       DconX = (uint16_t)vel; //The velocity becomes the derivative control

       if(x_vals[2] > XMID)
           IconX -= Ki;

       if(x_vals[2] < XMID)
           IconX += Ki;

       Fx = (uint16_t)((float)PconX + (float)DconX + IconX );

        motor_set_duty( 0, findDutyRange( (uint16_t)(XMAX - XMIN),  (uint16_t)(0),  Fx ));
        //motor_set_duty( 1, findDutyRange( (uint16_t)YMAX,  (uint16_t)YMIN,  y_vals[2]));

        prevloc = x_vals[2];
        {
           
            lcd_locate(5,0);
            lcd_printf("          ");
            lcd_locate(5,0);
            lcd_printf("%d",Fx);

            lcd_locate(5,1);
            lcd_printf("          ");
            lcd_locate(5,1);
            lcd_printf("%d",IconX);

            lcd_locate(5,2);
            lcd_printf("          ");
            lcd_locate(5,2);
            lcd_printf("%d",DconX);

            lcd_locate(5,3);
            lcd_printf("          ");
            lcd_locate(5,3);
            lcd_printf("%d",PconX);


            lcd_locate(5,7);
            lcd_printf("          ");
            lcd_locate(5,7);
            lcd_printf("%d",x_vals[2]);

            



        }


    }

    return 0;
}

