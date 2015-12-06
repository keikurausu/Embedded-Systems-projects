#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>

#include "lcd.h"
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "flexmotor.h"




/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  



uint16_t getXval()
{
    AD2CHS0bits.CH0SA = 0x4; //set ADC to Sample AN20 pin
    SETBIT(AD2CON1bits.SAMP); //start to sample
    while(!AD2CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD2CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return (uint16_t)ADC2BUF0; //return sample
}
uint16_t getYval()
{
    AD2CHS0bits.CH0SA = 0x5; //set ADC to Sample AN20 pin
    SETBIT(AD2CON1bits.SAMP); //start to sample
    while(!AD2CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD2CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return (uint16_t)ADC2BUF0; //return sample
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
	lcd_locate(0,0);
	lcd_printf("Hello World!");
    //disable ADC
    CLEARBIT(AD2CON1bits.ADON);
    //initialize PIN
    SETBIT(TRISBbits.TRISB5); //set TRISE RE8 to input
    CLEARBIT(AD2PCFGLbits.PCFG5); //set AD1 AN20 input pin as analog
	SETBIT(TRISBbits.TRISB4); //set TRISE RE8 to input
	CLEARBIT(AD2PCFGLbits.PCFG4); //set AD1 AN20 input pin as analog
    //Configure AD1CON1
    CLEARBIT(AD2CON1bits.AD12B); //set 10b Operation Mode
    AD2CON1bits.FORM = 0; //set integer output
    AD2CON1bits.SSRC = 0x7; //set automatic conversion
    //Configure AD1CON2
    AD2CON2 = 0; //not using scanning sampling
    //Configure AD1CON3
    CLEARBIT(AD2CON3bits.ADRC); //internal clock source
    AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
    AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
    //Leave AD1CON4 at its default value
    //enable ADC
    SETBIT(AD2CON1bits.ADON);


    SETBIT(TRISEbits.TRISE8);
    SETBIT(TRISDbits.TRISD10);

    motor_init(0);
    motor_init(1);

    uint16_t a= 0, i= 0;
    uint16_t xmax,ymax,xmin,ymin;
    uint16_t xconst = 0;
    uint16_t yconst = 0;
    uint16_t xcur, ycur;
    int pcount = 0;
    double tempx, tempy;
	
	
    lcd_locate(0,0);
    lcd_printf("Joystick X max =");
    while(a < 4){
        if (PORTDbits.RD10 == 0)
        {

            for(i = 0; i < 1000; i++)
            {

            }

            if (PORTDbits.RD10 == 0 ){
                       
                if(a == 0 ){
                    xmax = getXval();
                    lcd_locate(0,1);
                    lcd_printf("Joystick X min =");
                    lcd_locate(18,a);
                    lcd_printf("%d",xmax);
                }


                if(a == 1 ){
                    xmin = getXval();
                    lcd_locate(0,2);
                    lcd_printf("Joystick Y max =");
                    lcd_locate(18,a);
                    lcd_printf("%d",xmin);
                }

                if(a == 2 ){
                    ymax = getYval();
                    lcd_locate(0,3);
                    lcd_printf("Joystick Y min =");
                    lcd_locate(18,a);
                    lcd_printf("%d",ymax);
                }

                if(a == 3 ){
                    ymin = getYval();
                    lcd_locate(18,a);
                    lcd_printf("%d",ymin);
                }

                a++;

                while(PORTDbits.RD10 == 0){}

            }
        }
        if(pcount >= 10000){
            if(a < 2){
                lcd_locate(18,a);
                lcd_printf("%d",getXval());
            }
            else {
                lcd_locate(18,a);
                lcd_printf("%d",getYval());
            }
            pcount = 0;
        }
        pcount++;

    }



    //get teh x constant value

    lcd_locate(0,4);
    lcd_printf("Pulse width x =");

    a =0;
    while(a < 1)
    {
        if (PORTDbits.RD10 == 0)
        {

            for(i = 0; i < 1000; i++)
            {

            }

            if (PORTDbits.RD10 == 0 ){

                if(a == 0 ){
                    xconst = getXval();
                    lcd_locate(14,5);
                lcd_printf("%d us",(int)(xconst * 1000));
                }


                a++;

                while(PORTDbits.RD10 == 0){}

            }


        }

        xcur = getXval();
        ycur = getYval();

        tempx = findDutyRange( xmax,  xmin,  xcur);

        motor_set_duty(0, tempx );
        motor_set_duty( 1, findDutyRange( ymax,  ymin,  ycur));

        if(pcount >= 1000){

                lcd_locate(14,5);
                lcd_printf("%d us",(int)(tempx * 1000));

                pcount = 0;
            }
        pcount++;

    }

    motor_set_duty(0, findDutyRange( xmax,  xmin,  xconst));

    //get the y constant value
    lcd_locate(0,5);
    lcd_printf("Pulse width y =");
    a =0;
    while(a < 1)
    {
        if (PORTDbits.RD10 == 0)
        {

            for(i = 0; i < 1000; i++)
            {

            }

            if (PORTDbits.RD10 == 0 ){

                if(a == 0 ){
                    yconst = getYval();
                    lcd_locate(15,5);
                lcd_printf("%d us",(int)(yconst * 1000));
                }

                a++;

                while(PORTDbits.RD10 == 0){}

            }
        }



        ycur = getYval();
        tempy =  findDutyRange( ymax,  ymin,  ycur);
        motor_set_duty( 1, tempy);

        if(pcount >= 1000){

                lcd_locate(15,5);
                lcd_printf("%d us",(int)(tempy * 1000));

                pcount = 0;
            }
        pcount++;
    }

    motor_set_duty( 1, findDutyRange( ymax,  ymin,  yconst));

    while(1){}


    return 0;
}

