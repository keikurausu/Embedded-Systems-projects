/* 
 * File:   fflexserial.h
 * Author: delanos2
 *
 * Created on September 30, 2015, 10:13 PM
 */


#ifndef FLEXSERIAL_H
#define	FLEXSERIAL_H
#define FCY 12800000UL
#include "types.h" //make sure this header file is included in project

void uart2_init(uint16_t baud);
void uart2_putc(uint8_t c);
int8_t uart2_getc();

#endif	/* FLEXSERIAL_H */

