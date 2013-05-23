/*
Author: Caio Gustavo Mesquita Ângelo
Rev 0 - 18/11/2012
RLEG project - 2012

library of functions that handle Overo's GPIOs 
*/
#ifndef GPIO_FUNCTIONS_H_INCLUDED
#define GPIO_FUNCTIONS_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#define SUCCESS		1
#define FAILURE		-1

/* READ GPIO VALUE */
int gpio_read(uint8_t gpio);
/* Parameter
gpio:	string number of GPIO (e.g. "186")
returns 0 or 1
*/

/* WRITE GPIO VALUE */
int gpio_write(uint8_t gpio, uint8_t value);
/* Parameters
gpio:	string number of GPIO (e.g. "186")
value:	0 or 1
returns 1 if succeded or <0 if not 
*/

/* (FAST/FORCE) WRITE GPIO VALUE WITHOUT VERIFYING DIRECTION */
int gpio_f_write(uint8_t gpio, uint8_t value);
/* Parameters
gpio:	number of GPIO (e.g. 186)
value:	0 or 1
returns 1 if succeded
*/

#endif