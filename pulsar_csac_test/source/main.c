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
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
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


/* Task priorities. */
#define heartbeat_PRIORITY 				tskIDLE_PRIORITY
#define csac_watchdog_PRIORITY 			4
#define csac_query_PRIORITY 			max(csac_watchdog_PRIORITY - 1, 0)

#define CSAC_UART_TIMEOUT 				20		// UART query timeout in OS ticks

/*!
 * @brief Task function headers
 */

static void heartbeat_task(void *pvParameters);
static void csac_watcher(void *pvParameters);
static void csac_state_query(void *pvParameters);

static int getCsacState(void);

/*
 * GLOBAL VARIABLES
 */

// CSAC lock state is presented as a semaphore
SemaphoreHandle_t CSAC_lock_semaphore;


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
	xTaskCreate(heartbeat_task, "Heartbeat", configMINIMAL_STACK_SIZE, NULL, heartbeat_PRIORITY, NULL);
	xTaskCreate(csac_watcher, "CsacWatch", configMINIMAL_STACK_SIZE, NULL, csac_watchdog_PRIORITY, NULL);
	vTaskStartScheduler();

	for(;;) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}
}

static void heartbeat_task(void *pvParameters) {
	static const TickType_t xHeartbeat_period = 1000 / portTICK_PERIOD_MS;	// beat once every 1000 msec
	uint32_t count = 0;

	while (true) {		// infinite loop
		count++;
		PRINTF("Beat #%u\r\n", count);
		LED_RED_ON();
		vTaskDelay((xHeartbeat_period * 10) / 100);
		LED_RED_OFF();
		vTaskDelay((xHeartbeat_period * 13) / 100);
		LED_RED_ON();
		vTaskDelay((xHeartbeat_period * 12) / 100);
		LED_RED_OFF();
		vTaskDelay((xHeartbeat_period * (100 - 10 - 13 -12))/100);	// wait for next cycle
	}
}

static void csac_watcher(void *pvParameters) {

	// useful variables
	int csac_state = -kStatus_InvalidArgument;
	TaskHandle_t *csac_query_handle = NULL;

	// create CSAC semaphore
	CSAC_lock_semaphore = xSemaphoreCreateBinary();	// create "empty" binary semaphore
	if(CSAC_lock_semaphore == NULL) {
		// heap memory not sufficient
		// handle error
		DbgConsole_Printf("[%u] insufficient memory for semaphore. suspend csac_watcher\r\n", xTaskGetTickCount());
		vTaskDelete(NULL);
	} else {

		// check CSAC state in infinite loop
		while(true) {

			// get LOCK pin state
			xSemaphoreTake(CSAC_lock_semaphore, 0);
			lock_pin_state = GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN);

			if(lock_pin_state == 0) {
				// LOCK pin was low (for whatever reason)
				// Something is not right. Wait and check again
				vTaskDelay(100);

			} else {

				/*
				 * CSAC LOCK pin was high.
				 * This could be because:
				 * 	1. CSAC is just starting up
				 * 	2. CSAC is locked
				 * confirm through a UART query
				 */

				if(xTaskCreate(csac_state_query, "CsacQuery", configMINIMAL_STACK_SIZE, (void *) &csac_state, csac_query_PRIORITY, csac_query_handle) = pdTRUE) {
					vTaskDelay(CSAC_UART_TIMEOUT);
					vTaskDelete(csac_query_handle);
					if(csac_state == 0) {
						// CSAC is confirmed to be locked, give CSAC_lock_semaphore
						xSemaphoreGive(CSAC_lock_semaphore);
						DbgConsole_Printf("[%u] CSAC lock detected\r\n", xTaskGetTickCount(), csac_state);
					}

				} else {
					// could not create query task
					vTaskDelete(NULL);
				}
			}

			// sleep for 1 sec then check again
			vTaskDelay(1000);
		}
	}
}

/*
 * This thread is responsible for getting the CSAC state over UART
 * It assumes the CSAC lock pin is already high
 *
 * This task is currently implemented as one-shot.
 * It is the responsibility of the parent task to delete it after completion
 */

static void csac_state_query (void *pvParameters) {
	int *ret_val = (int *) pvParameters;

	// query CSAC over UART
	// TODO: acquire CSAC UART lock
	*ret_val = getCsacState();
	// TODO: release CSAC UART lock
	vTaskSuspend(NULL);
}

/*
 * get and parse state of csac
 */
// TODO: create serial driver for all CSAC operations
// TODO: move this function to generic CSAC library
static int getCsacState(void) {

	// variables
	int state = -kStatus_InvalidArgument;
	char ch;
	const char stateQuery[5] = "!^\r\n";

	// send UART query
	UART_WriteBlocking(BOARD_CSAC_UART, (uint8_t *) stateQuery, strlen(stateQuery));
	// record response (ie the first character)
	// TODO: if the CSAC is not started, ReadBlocking will never return
	UART_ReadBlocking(BOARD_CSAC_UART, (uint8_t *) &ch, 1);

	// CSAC state is represented by the first character in the response.
	// States may be '0' to '7'
	state = atoi(&ch);	// convert char to integer

	// flush the rest of the UART buffer
	while(1) {
		UART_ReadBlocking(BOARD_CSAC_UART, (uint8_t *) &ch, 1);
		if(ch == '\n') {	// \n from CSAC indicates end of current UART transaction
			break;
		}
	}

	// only return valid states, otherwise return error
	// TODO: create #defines with all known CSAC states
	if(state >= 0 && state <= 8) {
		return state;
	} else {
		return -kStatus_OutOfRange;
	}

}
