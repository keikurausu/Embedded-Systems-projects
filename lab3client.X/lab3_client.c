#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "lcd.h"
#include "led.h"
#include "crc16.h"
#include "fflexserial.h"
#include "lab03.h"


/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);



// Disable Code Protection
_FGS(GCP_OFF);


volatile int timeFlag = 0;
void __attribute__((__interrupt__)) _T1Interrupt(void)
{

       // TOGGLELED(LED2_PORT);
        //
        uart2_putc(0);
        timeFlag = 1;


        T1CONbits.TON = 0;// turn off timer
        TMR1 = 0x00; //Clear timer register
        IFS0bits.T1IF = 0; // clear the interrupt flag

}



int main()
{

    //init timer 1
        //enable LPOSCEN
        
        T1CONbits.TON = 0; //Disable Timer
        T1CONbits.TCS = 1; //Select external clock
        T1CONbits.TSYNC = 0; //Disable Synchronization
        T1CONbits.TCKPS = 0b11; //Select 1:1 Prescaler
        TMR1 = 0x00; //Clear timer register
        PR1 = 32767; //Load the period value
        IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
        IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
        IEC0bits.T1IE = 1;// Enable Timer1 interrupt
        T1CONbits.TON = 0;// Start Timer
        



    int i;
    int attempts = 0;
    lcd_initialize();
    uart2_init(9600); //initialize uart 2 with baud rate 9600
    uint8_t data = 1;   //variable to hold data read
    unsigned char message[MSG_BYTES_MSG];
    uint16_t Tcrc_server;
    unsigned char crc_server[2];
    unsigned char message_length;
    uint16_t current_crc;
    int loopCount = 0;
    lcd_clear();
    lcd_locate(0,0);
    lcd_printf("Recv Fail: ");
    lcd_locate(0,1);
    lcd_printf("CRC: ");
    lcd_locate(0,2);
    lcd_printf("Msg: ");

    while(1)
    {
        while(data != 0 )
        {
             uart2_getc(&data);
        }
        T1CONbits.TON = 1;// Start Timer

        Tcrc_server = 0; //initialize variable


        while(uart2_getc(&data) != 0){
           
        }
       
        crc_server[1] = data;
        //lcd_locate(5,2);
        //lcd_printf("%c",data);

       
        while(uart2_getc(&data)!=0 && (timeFlag == 0))
        {
        }
        crc_server[0] = data;
        //lcd_locate(6,2);
        //lcd_printf("%c",data);
         
        Tcrc_server = crc_server[1];
        Tcrc_server = Tcrc_server << 8;
        Tcrc_server |= crc_server[0];
        

        while(uart2_getc(&data) != 0 && (timeFlag == 0))
        {
        }
        message_length = data;
        //lcd_locate(7,2);
       // lcd_printf("%d",data);

        current_crc = 0;
        for(i = 0; (i < message_length) && (timeFlag == 0); i++)
        {
            while(uart2_getc(&data) != 0)
            {
                
            }
            message[i] = data;
            current_crc = crc_update(current_crc, message[i]);
        }
        message[message_length] = '\0';
        if(current_crc != Tcrc_server)
        {
          uart2_putc(0);
          attempts++;
        }
        else
        {
        
             uart2_putc(1);

            lcd_locate(11,0);
            lcd_printf("%d", attempts);
            lcd_locate(5,1);
            lcd_printf("%x",current_crc);
            lcd_locate(5,2);
            lcd_printf("%s",message);
        }
        T1CONbits.TON = 0;// Stop timer
        TMR1 = 0x00; //Clear timer register
        timeFlag = 0;

       
    }


        return 0;
}

