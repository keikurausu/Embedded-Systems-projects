/* 
 * File:   flexmotor.h
 * Author: delanos2
 *
 * Created on October 15, 2015, 7:08 PM
 */

#ifndef FLEXMOTOR_H
#define	FLEXMOTOR_H
#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif



void motor_init(uint8_t chan);

void motor_set_duty(uint8_t chan, double duty_us);


#ifdef	__cplusplus
}
#endif

#endif	/* FLEXMOTOR_H */

