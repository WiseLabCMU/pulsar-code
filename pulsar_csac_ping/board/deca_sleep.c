/*
 * deca_sleep.c
 *
 *  Created on: Sep 25, 2016
 *      Author: adwait
 */

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

void deca_sleep(unsigned int time_ms) {
	vTaskDelay(time_ms * portTICK_PERIOD_MS);
}
