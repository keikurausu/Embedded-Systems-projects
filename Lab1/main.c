#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "lcd.h"
#include "led.h"

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

volatile int count = 0;
void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    if(count < 5){
        TOGGLELED(LED4_PORT);
        count++;
    }
    
    IFS0bits.T1IF = 0; // clear the interrupt flag
}


void main(){
	//Init LCD
	__C30_UART=1;	
	lcd_initialize();
	lcd_clear();

        lcd_locate(0,0);
	lcd_printf("Erik");

        lcd_locate(1,1);
	lcd_printf("Dallas");

        lcd_locate(2,2);
	lcd_printf("Caleb ");
        led_initialize();

        //enable LPOSCEN
        __builtin_write_OSCCONL(OSCCONL | 2);
        T1CONbits.TON = 0; //Disable Timer
        T1CONbits.TCS = 1; //Select external clock
        T1CONbits.TSYNC = 0; //Disable Synchronization
        T1CONbits.TCKPS = 0b00; //Select 1:1 Prescaler
        TMR1 = 0x00; //Clear timer register
        PR1 = 32767; //Load the period value
        IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
        IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
        IEC0bits.T1IE = 1;// Enable Timer1 interrupt
        T1CONbits.TON = 1;// Start Timer

        SETBIT(TRISEbits.TRISE8);
        SETBIT(TRISDbits.TRISD10);
        SETBIT(AD1PCFGHbits.PCFG20);
        CLEARBIT(LED2_PORT);
        int i = 0;
        int triggerCount = 0;
        int trig = 1;
        int current_state = 1;
        lcd_locate(3,3);
	lcd_printf("%d",triggerCount);
        lcd_locate(3,4);
        lcd_printf("%x",triggerCount);
	while(1){
           if (PORTEbits.RE8 == 0)
           {

               for(i = 0; i < 1000; i++)
               {
               
               }
               if (PORTEbits.RE8 == 0)
                   trig = 0;
           }

               if(trig == 0)
               {
                   if(current_state == 1){
                   SETBIT(LED1_PORT);
                   triggerCount++;
                   lcd_locate(3,3);
                   lcd_printf("%d",triggerCount);
                   lcd_locate(3,4);
                   lcd_printf("%x",triggerCount);
                   current_state = 0;
                   }
               }

               else
               {
                   current_state = 1;
                   CLEARLED(LED1_PORT);
               }
       
           
           if(PORTDbits.RD10 == 0)
           {
               SETBIT(LED2_PORT);
           }
           else
           {
               CLEARBIT(LED2_PORT);
           }
           Nop();

           if((PORTDbits.RD10 == 0 && trig == 0) || (PORTDbits.RD10 == 1 && trig == 1))
           {
            CLEARBIT(LED3_PORT);
           }
           else{
               SETBIT(LED3_PORT);
           }

           trig =1;
           
         }
 
}

