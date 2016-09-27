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

#define DEBUG_MODE 1

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pulsar_board.h"
#include "pin_mux.h"
#include "clock_config.h"

// TODO: get rid of temporary includes
#include "pll_driver.h"

/**
 * Contains application thread functions
 * Threads are implemented in different files to ensure independent operation
 * Any information exchange must thus be intentionally setup
 */
#include "application_threads.h"

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


/**
 * Utility includes
 */
#include "fsl_debug_console.h"



static void commander_thread(void * pvParameters);


/*
 * GLOBAL VARIABLES
 */

TaskHandle_t heartbeatHandle 			= NULL;
/**
 * TODO: needs update
 * Set of shared queues for various tasks
 * 0: heartbeat commands
 * 1: csac-PLL synchronization
 * 2: PLL-DW1000 synchronization
 * 3: DW1000-App synchronization
 */
volatile SemaphoreHandle_t sem[SEM_N];	// general semaphores for synchronization
volatile QueueHandle_t HeartbeatQ;		// allocate space for general purpose queues
/*
 * CONFIGURATION SECTION
 */

// Empty for now

/*!
 * @brief Application entry point.
 */
int main(void) {
	/* Init board hardware. */
	BOARD_ConfigPinmux();


	BOARD_InitPins();
	BOARD_BootClockRUN();

	BOARD_InitDebugConsole();

	vSemaphoreCreateBinary(sem[CSAC_PLL_SEM]);
	vSemaphoreCreateBinary(sem[PLL_DW_SEM]);
	vSemaphoreCreateBinary(sem[DW_APP_SEM]);

	// task for debug and control
	// there can only be one of these
	xTaskCreate(commander_thread, "Command", configMINIMAL_STACK_SIZE, NULL, commander_PRIORITY, NULL);

	// low priority task for heartbeat LED control
	// there can only be one of these
	xTaskCreate(heartbeat_thread, "Heartbeat", configMINIMAL_STACK_SIZE, (void *) &HeartbeatQ, heartbeat_PRIORITY, heartbeatHandle);

	// thread watches over and configures CSAC for proper operation
	// there can only be one of these
	xTaskCreate(csac_watcher_thread, "CSACwatch", configMINIMAL_STACK_SIZE, (void *) sem, watcher_PRIORITY, NULL);

	// thread watches over and configures PLL for proper operation
	// there can only be one of these
	xTaskCreate(pll_watcher_thread, "PLLwatch", configMINIMAL_STACK_SIZE, (void *) sem, watcher_PRIORITY, NULL);

	// thread watches over and configures Decawave radio for proper operation
	// there can only be one of these
	xTaskCreate(dw_watcher_thread, "DWwatch", configMINIMAL_STACK_SIZE, (void *) sem, watcher_PRIORITY, NULL);

	// Application thread. All the magic happens here (hopefully) :-)
	xTaskCreate(application_thread, "App", configMINIMAL_STACK_SIZE, (void *) sem, application_PRIORITY, NULL);

	vTaskStartScheduler();

	while(true) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}
}

/**
 * Commander thread for command and control over the system
 * @param pvParameters
 */
static void commander_thread(void * pvParameters) {
	// do nothing for now
	vTaskSuspend(NULL);
}
