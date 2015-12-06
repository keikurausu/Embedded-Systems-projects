#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include "linuxanalog.h"
#include <inttypes.h>
//#include <librt.h>

volatile float v1;
volatile float v2;
volatile int flip = 0;
// Define a signal handler function. The function name can be anything, but
// it must accept a single integer.
void handler_function(int signum)
{
	if(flip == 0)
	{
		flip = 1;
		//dac((uint16_t)(0));		
		dac( (uint16_t)(((v1+5)/(float)(10)) * (float)(pow(2,12)-1) ) );
	}
	else
	{
		flip = 0;
		//dac((uint16_t)(4095));
		dac( (uint16_t)(((v2+5)/(float)(10)) * (float)(pow(2,12)-1) ) );
	}
		
}

// This function associates the SIGINT signal with the handler function.
void setup_signal_handler()
{	
	struct sigaction action;
	
	// Ensure that the entire structure is zeroed out.
	memset(&action, 0, sizeof(action));
	
	// Set the sa_handler member to point to the desired handler function.
	action.sa_handler = handler_function;
	
	// Call sigaction to change the action taken upon receipt of the SIGINT signal.
	if (sigaction(SIGALRM, &action, NULL) != 0)
	{
		// If there is an error, print out a message and exit.
		perror("sigaction");
		exit(1);
	}
	

}


int main()
{
	//set up neccessary stuff here
	float maxFreq,freq;
	struct timespec clock_resolution;

	printf("Enter voltage #1 (-5 to +5 volts, other to quit): ");
	scanf("%f", &v1);
	
	if((v1 < -5)||(v1 > 5))
		return 0;
 
	printf("Enter voltage #1 (-5 to +5 volts, other to quit): ");
	scanf("%f", &v2);
	
	if((v2 < -5)||(v2 > 5))
		return 0;

	clock_getres(CLOCK_REALTIME, &clock_resolution);
	maxFreq = pow(10,9) * clock_resolution.tv_sec;
	maxFreq += clock_resolution.tv_nsec;

	maxFreq = maxFreq / 1000000000;

	maxFreq = 1 / maxFreq;
	maxFreq = maxFreq / 2;

	
	
	
	printf("Enter frequency (0 to %f Hz, other to quit): ", maxFreq);
	scanf("%f", &freq);
	
	if((freq < 0)||(freq > maxFreq))
		return 0;	


	printf("Values:\nVoltage 1: %f volts\nVoltage 2: %f volts\nFrequency: %f hz\n",v1,v2, freq);
	
	
	das1602_initialize();

	setup_signal_handler();

	timer_t timer1;

	if(timer_create(CLOCK_REALTIME, NULL, &timer1) != 0)
	{
		perror("timer_create");
		exit(1);
	}
	//struct sigevent timer_event;
	struct itimerspec timer1_time;
	timer1_time.it_value.tv_sec = 1;
	timer1_time.it_value.tv_nsec = 0;
	timer1_time.it_interval.tv_sec = (int)(1 / freq) ;
	timer1_time.it_interval.tv_nsec = (int)(((1/ freq) - ((float)(int)(1 / freq))) * 1000000000) ;

	if(timer_settime(timer1, 0, &timer1_time, NULL)  != 0)
	{
		perror("timer_settime");
		exit(1);
	}


	//spin forever
	uint32_t counter = 0;
	while (1)
	{
		//Output the count value after every 100 million loops
		if ((counter % 100000000) == 0)
		{
			printf("Counter: %u\n", counter);

		}
		counter++;
	}
}
