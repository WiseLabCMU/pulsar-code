/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Simple test application that implements a heartbeat
 **/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pulsar_board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*
 * Useful constants
 */
#define beatFrequency 1000				// beat frequency in OS ticks

/*
 * Useful commands for CSAC UART communication
 */

enum csacCmd {
	cmdNONE 		= 0,
	cmdGET_STATE 	= 1,
	cmdN,
};

typedef enum csacCmd csacCmd_t;

/*
 * CSAC worker thread communication struct
 */

struct csacWorkerStruct {
	csacCmd_t 	cmd;		// command for worker thread
	void 		*data;		// data pointer to pass
};

/*
 *  Task priorities.
 */

#define heartbeat_PRIORITY 				tskIDLE_PRIORITY	// lowest priority possible


/*!
 * @brief Task function headers
 */

static void heartbeat_task(void *pvParameters);
static void csac_worker_task(void *pvParameters);

/*
 * GLOBAL VARIABLES
 */

TaskHandle_t heartbeatHandle = NULL;
uint32_t beatCount = 0;
TickType_t xHeartbeat_period = beatFrequency / portTICK_PERIOD_MS;	// default value: beat once every 1/beatFrequency sec

/*
 * CONFIGURATION SECTION
 */



/*!
 * @brief Application entry point.
 */
int main(void) {
	/* Init board hardware. */
	BOARD_ConfigPinmux();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	/* Add your code here */

	/* Create RTOS heartbeat task */
	xTaskCreate(heartbeat_task, "Heartbeat", configMINIMAL_STACK_SIZE, (void *) &beatCount, heartbeat_PRIORITY, heartbeatHandle);

	vTaskStartScheduler();

	for(;;) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}
}

static void heartbeat_task(void *pvParameters) {

	uint32_t *count = (uint32_t *) pvParameters;
	*count = 0;
	TickType_t t;

	while (true) {		// infinite loop
		t = xTaskGetTickCount();
		(*count)++;
		PRINTF("[%u] Beat #%u\r\n", t, *count);
		LED_RED_ON();
		vTaskDelay((xHeartbeat_period * 10) / 100);
		LED_RED_OFF();
		vTaskDelay((xHeartbeat_period * 13) / 100);
		LED_RED_ON();
		vTaskDelay((xHeartbeat_period * 12) / 100);
		LED_RED_OFF();
		vTaskDelayUntil(&t, xHeartbeat_period);	// wait for next cycle
	}
}

/*!
 * @brief	This task is responsible for UART based interactions with the CSAC
 */

static void csac_worker_task(void *pvParameters) {



	// acquire mutex
}
