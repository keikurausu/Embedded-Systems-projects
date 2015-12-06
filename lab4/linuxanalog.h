#ifndef LINUXANALOG_H
#define	LINUXANALOG_H


#include <signal.h>
#include <sys/io.h>
#include <inttypes.h>


#define BADR0 0xD000
#define BADR1 0xD0A0
#define BADR2 0xD080
#define BADR3 0xD060
#define BADR4 0xD040
#define DAC1R1 1024
#define DAC1R0 512
#define DAC0R1 256
#define DAC0R0 128
#define HS1 64
#define HS0 32
#define DAPS1 16
#define DAPS0 8
#define START 4
#define DACEN 2
#define LDAEMCL 1

void das1602_initialize();
void dac(uint16_t value);

#endif /*LINUXANALOG_H*/
