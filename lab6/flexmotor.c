#include "flexmotor.h"

#include <p33Fxxxx.h>
//do not change the order of the following 3 definitions
#define FCY 12800000UL
#include <stdio.h>
#include <libpic30.h>
#include "types.h"
#include "lcd.h"


void motor_init(uint8_t chan)
{
    //setup Timer 2
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T2CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // Set timer period 20ms:
    // 8000= 40*10^-3 * 12.8*10^6 * 1/64
    //setup OC8 or OC7 depending on channel
    if(chan == 0)
    {
        CLEARBIT(TRISDbits.TRISD7); /* Set OC8 as output */
        OC8R = 4000 - 0.9*200; /* Set the initial duty cycle to 5ms*/
        OC8RS = 4000 - 1.5*200; /* Load OCRS: next pwm duty cycle */
        OC8CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */
    }
    else if(chan == 1)
    {
        CLEARBIT(TRISDbits.TRISD6); /* Set OC7 as output */
        OC7R = 4000 - 0.9*200; /* Set the initial duty cycle to 5ms*/
        OC7RS = 4000 -1.5*200; /* Load OCRS: next pwm duty Ocycle */
        OC7CON = 0x0006; /* Set OC8: PWM, no fault check, Timer2 */
    }
    
    SETBIT(T2CONbits.TON); /* Turn Timer 2 on */

}

void motor_set_duty(uint8_t chan, double duty_us)
{
    double temp;

    if((duty_us > 2.1) || (duty_us < 0.9))
        return;

    if(chan == 0)
    {
        temp = ((1- ((duty_us - 0.9) / 1.2)) *1.2) + 0.9;
        //OC8R = duty_us*200; /* Set the initial duty cycle to 5ms*/
        OC8RS = 4000 - temp*200; /* Load OCRS: next pwm duty cycle */
    }
    else if(chan == 1)
    {
        //OC7R = duty_us*200; /* Set the initial duty cycle to 5ms*/
        OC7RS = 4000 - duty_us*200; /* Load OCRS: next pwm duty Ocycle */
    }
}
