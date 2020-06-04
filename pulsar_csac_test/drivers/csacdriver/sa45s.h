/*
 * sa45s.h
 *
 *  Created on: Sep 5, 2016
 *      Author: adwait
 */

#ifndef DRIVERS_CSACDRIVER_SA45S_H_
#define DRIVERS_CSACDRIVER_SA45S_H_

#include "fsl_uart.h"

/*
 * CSAC command definitions and macros
 */

#define csacCmd ( cmd ) "!" cmd "\r\n"

#define csacStateQuery	"^"

/*
 * CSAC states
 * TODO: enumerate all the states
 */

/*
 * CSAC functions
 */

// TODO: change csacInit defintion based on what is required
int csacInit(UART_Type *uart_dev);
int csacDenit(void);
int getCsacState(void);

extern UART_Type *csacUart;

#endif /* DRIVERS_CSACDRIVER_SA45S_H_ */
