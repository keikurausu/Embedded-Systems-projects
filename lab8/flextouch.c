#include "flextouch.h"

#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "lcd.h"


void touch_init()
{
    CLEARBIT(TRISEbits.TRISE1); //I/O pin set to output
    CLEARBIT(TRISEbits.TRISE2);
    CLEARBIT(TRISEbits.TRISE3);

}

void touch_select_dim(uint8_t dim)
{
    if(dim == 0)
    {
         AD1CHS0bits.CH0SA = 0x0F; //set ADC to Sample AN20 pin

        //CLEARBIT(PORTEbits.RE1);
        //SETBIT(PORTEbits.RE2);
        CLEARBIT(LATEbits.LATE1);
        SETBIT(LATEbits.LATE2);
        SETBIT(LATEbits.LATE3);
       // SETBIT(PORTEbits.RE3);
    }
    else if(dim == 1)
    {
        AD1CHS0bits.CH0SA = 0x09; //set ADC to Sample AN20 pin
        //SETBIT(PORTEbits.RE1);
        SETBIT(LATEbits.LATE1);
        CLEARBIT(LATEbits.LATE2);
        CLEARBIT(LATEbits.LATE3);
        //CLEARBIT(PORTEbits.RE2);
        //CLEARBIT(PORTEbits.RE3);
    }
}

uint16_t touch_adc()
{
    SETBIT(AD1CON1bits.SAMP); //start to sample
    while(!AD1CON1bits.DONE); //wait for conversion to finish
    CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
    return (uint16_t) ADC1BUF0; //return sample
}


