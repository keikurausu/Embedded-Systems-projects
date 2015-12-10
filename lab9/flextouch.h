/* 
 * File:   flextouch.h
 * Author: delanos2
 *
 * Created on October 21, 2015, 2:21 PM
 */

#ifndef FLEXTOUCH_H
#define	FLEXTOUCH_H
#include "types.h"
#ifdef	__cplusplus
extern "C" {
#endif


void touch_init();

void touch_select_dim(uint8_t dim);

uint16_t touch_adc();


#ifdef	__cplusplus
}
#endif

#endif	/* FLEXTOUCH_H */

