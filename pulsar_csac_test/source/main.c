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
#define heartbeat_PRIORITY 0
#define csac_watchdog_PRIORITY 4

/*!
 * @brief Task function headers
 */

static void heartbeat_task(void *pvParameters);
static void csac_watcher(void *pvParameters);
//static void csac_uart_query(void *pvParameters);

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

	while (1) {
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
	uint32_t lock_pin_state = 0;
	int csac_state = 0;

	// create CSAC semaphore
	CSAC_lock_semaphore = xSemaphoreCreateBinary();	// create "empty" binary semaphore
	if(CSAC_lock_semaphore == NULL) {

		// heap memory not sufficient
		// handle error
		DbgConsole_Printf("[%u] insufficient memory for semaphore. suspend csac_watcher\r\n", xTaskGetTickCount());
		vTaskSuspend(NULL);

	} else {

		// check CSAC state in infinite loop
		while(1) {

			// loop-wait if lock not acquired

			// check LOCK pin state and wait if it is not high
			while(1) {
				lock_pin_state = GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN);
				if(lock_pin_state == 0){

					// note: instead of loop, an interrupt can be programmed
					// work on this later

					// no lock. wait and loop indefinitely till lock is detected.
					vTaskDelay(100);

				} else {
					// lock pin is high. this can occur due to 2 reasons on the CSAC:
					// 1. power on indication at startup
					// 2. CSAC is actually locked

					// check actual state through UART

					csac_state = getCsacState();
					break;
				}
			}

			// if not locked, wait and loop
			// if locked, query over Serial
			// if not locked, wait and loop
			// if locked, wait and re-check after timeout period (once per sec?)

			// CSAC is confirmed to be locked, give CSAC_lock_semaphore
			xSemaphoreGive(CSAC_lock_semaphore);

			DbgConsole_Printf("[%u] CSAC lock detected\r\n", xTaskGetTickCount(), csac_state);

			// sleep for 1 sec then check again
			vTaskDelay(1000);
		}
	}
}

//static void csac_uart_query(void *pvParameters) {
//	// send query symbols
//	// wait for response but also implement timeout
//	// return
//}

/*
 * get and parse state of csac
 */
// TODO: create serial driver for all CSAC operations
static int getCsacState(void) {

	// variables
	int state = -1;
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