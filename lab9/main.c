#include "lcd.h"
#include "types.h"
#include "led.h"
#include "flexmotor.h"
#include "flextouch.h"
#include "performance.h"
#include "math.h"

#include <p33Fxxxx.h>


#define XMAX 3040
#define YMAX 2650
#define XMIN 270
#define YMIN 450
#define XMID (XMAX + XMIN) /2
#define YMID (YMAX + YMIN) /2

#define JXMAX 1023
#define JYMAX 1023
#define JXMIN 200
#define JYMIN 230


#define FCY 12800000UL

// control task frequency (Hz)
#define RT_FREQ 50

//setpoint parameters
#define SPEED 0.12  // tested up to .12!
#define RADIUS 650
#define CENTER_X (XMAX + XMIN) /2
#define CENTER_Y (YMAX + YMIN) /2

// Servo defines
#define MAX_DUTY_MICROSEC 2100
#define MIN_DUTY_MICROSEC 900
#define SERVO_PWM_PERIOD_MICROSEC 20000
#define INIT_DUTYX_MICROSEC 1410    // leveled plate on X axis: 1410
#define INIT_DUTYY_MICROSEC 1400     // leveled plate on Y axis: 1400
#define MOTOR_X_CHAN 1
#define MOTOR_Y_CHAN 2

// Touch screen definitions
#define X_DIM 0
#define Y_DIM 1

uint16_t TOUCH_MIN_X = 340;
uint16_t TOUCH_MAX_X = 3020;
uint16_t TOUCH_MIN_Y = 454;
uint16_t TOUCH_MAX_Y = 2657;



// do not change position of this include
#include <libpic30.h>

/* Initial configuration by EE */
// Primary (XT, HS, EC) Oscillator with PLL
_FOSCSEL(FNOSC_PRIPLL);

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystal
_FOSC(OSCIOFNC_OFF & POSCMD_XT);

// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);

// Disable Code Protection
_FGS(GCP_OFF);


// control setpoint
double Xpos_set = 1650.0, Ypos_set = 1550.0;

// raw, unfiltered X and Y position of the ball
volatile uint16_t Xpos, Ypos;
volatile uint8_t start = 0;
volatile uint8_t select = X_DIM;
volatile uint8_t deadline_miss = 0;

   
   volatile float const Kp = 1.3;
   volatile float const Kd = -.5;
   volatile float const Ki = .05;
    volatile float const Kpy = 0.15;
   volatile float const Kdy = -.2;
   volatile float const Kiy = .5;
  volatile  uint16_t PconX;
  volatile  uint16_t PconY;
  volatile  uint16_t DconX;
  volatile  uint16_t DconY;

  volatile  uint16_t currlocx,currlocy;
  volatile  uint16_t prevlocx,prevlocy;
  volatile  float velx,vely, print;
  volatile  uint16_t Fx,Fy;
  volatile float IconX = 0;
  volatile float IconY = 0;




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

double pidX_controller(double Xp) {
  double pid;

  lcd_locate(0,2);
  lcd_printf("       ");
 lcd_locate(0,2);
  

uint16_t goal = (uint16_t)Xp;



  // TODO: Implement PID X
  PconX =  (uint16_t)((((float)goal - (float)Xpos) * Kp)- (float)((float)goal - (float)XMAX));
  
       //find the velocity. change in X position over change in time(0.05s)
       velx = Kd*(((float)Xpos - (float)prevlocx) / .05);
       DconX = velx; //The velocity becomes the derivative control
       if(goal != Xpos){

            if(Xpos > goal)
                IconX -= Ki;

            if(Xpos < goal)
                 IconX += Ki;
       }

      //IconX = velx =0;
       if(((float)PconX + velx) >= 0)
            Fx = (uint16_t)((float)PconX + velx + IconX );
       else
            Fx = (uint16_t)((float)PconX + IconX );

       //if(goal != x_vals[2])
         motor_set_duty( 0, findDutyRange( (uint16_t)(XMAX - XMIN),  (uint16_t)(0), Fx));
      
        //motor_set_duty( 1, findDutyRange( (uint16_t)YMAX,  (uint16_t)YMIN,  y_vals[2]));
lcd_printf("%d",(int)Ypos);
        prevlocx = Xpos;
  return pid;
}


double pidY_controller(double Yp) {
    double pid;
    uint16_t goal = (uint16_t)Yp;

  // TODO: Implement PID X
  PconY =  (uint16_t)((((float)goal - (float)Ypos) * Kpy)- (float)((float)goal - (float)YMAX));

       //find the velocity. change in X position over change in time(0.05s)
       vely = Kdy*(((float)Ypos - (float)prevlocy) / .05);
       DconY = vely; //The velocity becomes the derivative control
       if(goal != Ypos){

            if(Ypos > goal)
                IconY -= Kiy;

            if(Ypos < goal)
                 IconY += Kiy;
       }

      //IconX = velx =0;
       if(((float)PconY + vely) >= 0)
            Fy = (uint16_t)((float)PconY + vely + IconY );
       else
            Fy = (uint16_t)((float)PconY + IconY );

       //if(goal != x_vals[2])
        // motor_set_duty( 0, findDutyRange( (uint16_t)(XMAX - XMIN),  (uint16_t)(0), Fy ));

        motor_set_duty( 1, findDutyRange( (uint16_t)YMAX,  (uint16_t)0,  Fy));

        prevlocy = Ypos;
  return pid;
 
}








// Configure the real-time task timer and its interrupt.
void timers_initialize() {

  //Set Timer1 to generate an interrupt every 10ms (100Hz) ==> PR1=500
  CLEARBIT(T1CONbits.TON); //Disable Timer
  CLEARBIT(T1CONbits.TCS); //Select internal instruction cycle clock
  CLEARBIT(T1CONbits.TGATE); //Disable Gated Timer mode
  T1CONbits.TCKPS = 0b11; //Select 1:256 Prescaler
  PR1 = 500; //Load the period value ==> running at 100Hz now!
  TMR1 = 0x00; //Clear timer register
  IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
  CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
  SETBIT(IEC0bits.T1IE); // Enable Timer1 interrupt
  SETBIT(T1CONbits.TON); // Start Timer
}

int main(){
  uint8_t start_r, old_IPL;
  uint8_t hz50_scaler, hz5_scaler, hz1_scaler, sec;
  uint32_t tick = 0;

  hz50_scaler = hz5_scaler = hz1_scaler = sec = 0;

  touch_init();
  touch_select_dim(X_DIM);

  __delay_ms(200);
  lcd_initialize();             // Initialize the LCD 

  motor_init(0);
  motor_init(1);

  lcd_clear();
  lcd_locate(0,0);
  lcd_printf("-- Ball position: --");

  timers_initialize();          // Initialize timers


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

    
   // SETBIT(AD1PCFGHbits.PCFG20);

    touch_init();
    
  while (1) {
    start_r = 0;
    while(!start_r) {      
      // disable all maskable interrupts
      SET_AND_SAVE_CPU_IPL(old_IPL, 7);
      start_r = start;

      // enable all maskable interrupts
      RESTORE_CPU_IPL(old_IPL);
    }

    
    // Periodic real-time task code starts here!!!
    double pidX, pidY;
    uint16_t duty_us_x, duty_us_y;
   // int i,j,a ;

    //touch_select_dim(0);

       

        
        //do 5 reading for y
       


      

    // 50Hz control task
    if(hz50_scaler == 0) {
      calcQEI(Xpos_set, Xpos, Ypos_set, Ypos);

      Xpos_set = CENTER_X + RADIUS * cos(tick * SPEED);
      Ypos_set = CENTER_Y + RADIUS * sin(tick * SPEED);
      tick++;

      pidX = pidX_controller(Xpos_set);
      pidY = pidY_controller(Ypos_set);

      // TODO: Convert PID to motor duty cycle (900-2100 us)

      // setMotorDuty is a wrapper function that calls your motor_set_duty
      // implementation in flexmotor.c. The 2nd parameter expects a value
      // between 900-2100 us
     
      //these are called inside of the pid controller function
      //setMotorDuty(MOTOR_X_CHAN, duty_us_x);
      //setMotorDuty(MOTOR_Y_CHAN, duty_us_y);
    }

    // 5Hz display task
    if(hz5_scaler == 0) {
      lcd_locate(0,1);
      lcd_printf("Xf(t)=%u", Xpos);

      if(deadline_miss >= 1) {
        lcd_locate(0,6);
        lcd_printf("%4d d_misses!!!", deadline_miss);
      }
    }

    // 1Hz seconds display task
    if(hz1_scaler == 0) {
      lcd_locate(0,7);
      lcd_printf("QEI: %5u", getQEI());
      sec++;
    }
    
    hz50_scaler = (hz50_scaler + 1) % 2;
    hz5_scaler = (hz5_scaler + 1) % 20;
    hz1_scaler = (hz1_scaler + 1) % 100;

    start = 0;
  }

  return 0;
}

// This ISR will execute whenever Timer1 has a compare match.
// it kicks off the periodic execution of user code and performs I/O
// Min period: 10msec due to X,Y switch time for touchscreen
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    int i,j,a ;
  uint16_t x_vals[5];
  uint16_t y_vals[5];
  
  IFS0bits.T1IF = 0; // clear interrupt flag

  if(start == 1)
    deadline_miss++;

  if (select == X_DIM) {
    // TODO: read 5 samples from X-dimension and set Xpos as the median
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
        Xpos = x_vals[2];


    touch_select_dim(Y_DIM);
    select = Y_DIM;
  }
  else {
    // TODO: read 5 samples from Y-dimension and set Ypos as the median
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
     Ypos = y_vals[2];
    touch_select_dim(X_DIM);
    select = X_DIM;
  }


  start = 1;
}
