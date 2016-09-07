/*
 * sa45s.c
 *
 *  Created on: Sep 5, 2016
 *      Author: adwait
 */

#include "sa45s.h"

UART_Type *csacUart = NULL;

int csacInit(UART_Type *uart_dev) {
	// TODO: add other checks here
	csacUart = uart_dev;
}

int csacDenit(void) {
	// TODO: add other checks here
	csacUart = NULL;
}

int getCsacState(void) {

}
