#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "lcd.h"
#include "led.h"
int loop_count = 2000;

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT); 

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);  

int count = 0;
int SecondsSinceReset = 0;
int MilliSecondsSinceReset = 0;

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    
       // TOGGLELED(LED2_PORT);
        SecondsSinceReset++;
        IFS0bits.T1IF = 0; // clear the interrupt flag
}


void __attribute__((__interrupt__)) _T2Interrupt(void)
{
  
       
        count++;
        if(count % 2 == 0)
        {
        //    TOGGLELED(LED1_PORT);
        }
        MilliSecondsSinceReset++;

        if(MilliSecondsSinceReset == 1000)
        {
            MilliSecondsSinceReset = 0;
        }

    IFS0bits.T2IF = 0; // clear the interrupt flag
}

void __attribute__((__interrupt__)) _INT1Interrupt(void)
{
            SecondsSinceReset = MilliSecondsSinceReset = 0;



          IFS1bits.INT1IF = 0; // clear the interrupt flag
}


void main(){
        double time=0;
        double iteration_time = 0;
        const double time_const = 12800.0;
        //Init LCD
	__C30_UART=1;	
	lcd_initialize();
	lcd_clear();
        led_initialize();
        
        //init timer 1
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

        // init timer 2
       T2CONbits.TON = 0;
        T2CONbits.TCS = 0;
        T2CONbits.TGATE = 0;
        T2CONbits.TCKPS = 0b11; // modulates the input clock
        TMR2 = 0x00; 
        PR2 = 50; // how high the timer counts

        //IPC0bits.T2IP = 0x01; //Why does this give us an error?
        IFS0bits.T2IF = 0; 
        IEC0bits.T2IE = 1;
        T2CONbits.TON = 1;


        // init timer 3
         T3CONbits.TON = 0;
        T3CONbits.TCS = 0;
        T3CONbits.TGATE = 0;
         T3CONbits.TCKPS = 0b00;
        TMR3 = 0x00;
        PR3 = 32767;
        //IPC0bits.T3IP = 0x02; //Why does this give us an error?
        IFS0bits.T3IF = 0;
      // IEC0bits.T3IE = 1;
        T3CONbits.TON = 1;
        

        //set joystick mapped io
       SETBIT(TRISEbits.TRISE8);
       SETBIT(TRISDbits.TRISD10);
       SETBIT(AD1PCFGHbits.PCFG20);


 

        //set joystick interupt
        
      IEC1bits.INT1IE = 1;
      IPC5bits.INT1IP = 0x03;
      INTCON2bits.INT1EP = 1;
       
       Nop();

       
        while(1){
            //lcd_printf("O.%.4x",TMR3);
           TMR3 = 0x00;
            // blink ever iteration
            SETLED(LED4_PORT);
            Nop();
            if(loop_count > 25000)
            {
              
                lcd_locate(0,0);
                Nop();
                lcd_printf("%.2d:%.2d.%.3d",SecondsSinceReset/60,SecondsSinceReset%60,MilliSecondsSinceReset);
                lcd_locate(0,1);
                lcd_printf("%0d cycles",time);
                Nop();
                Nop();
                lcd_locate(0,2);
                iteration_time = (time / time_const);
                lcd_printf("%f %f ms",iteration_time, time_const);
                Nop();
                loop_count = 0;
                Nop();

                
            }
           loop_count++;
           //lcd_locate(0,1);
           time = TMR3;
           
          
         }
        
}

