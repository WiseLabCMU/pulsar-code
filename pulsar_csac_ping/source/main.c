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
 * Useful constants
 */

#define heartbeat_PRIORITY 				tskIDLE_PRIORITY		// lowest priority possible
#define commander_PRIORITY				(configMAX_PRIORITIES -1)	// highest priority possible

// TODO: define proper priorities here
#define watcher_PRIORITY
#define worker_PRIORITY

// TODO: get rid of temporary globals


/*
 * GLOBAL VARIABLES
 */

TaskHandle_t heartbeatHandle = NULL;
volatile QueueHandle_t heartbeatCmdQ = NULL;
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

	/* Create RTOS heartbeat task */
	xTaskCreate(heartbeat_thread, "Heartbeat", configMINIMAL_STACK_SIZE, (void *) &heartbeatCmdQ, heartbeat_PRIORITY, heartbeatHandle);
	xTaskCreate(commander_thread, "Commander", configMINIMAL_STACK_SIZE, (void *) &heartbeatCmdQ, commander_PRIORITY, NULL);

	vTaskStartScheduler();

	while(true) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}
}

static void commander_thread(void * pvParameters) {
//	volatile int val = 0;
	dspi_rtos_handle_t spiRtosHandle;
	pll_communication_init(&spiRtosHandle, CLOCK_GetFreq(BOARD_PLL_SPI_MASTER_CLK_SRC));
	while(true) {
		pll_register_write(&spiRtosHandle, lmx2571_reg_val[0].datamap.ADDRESS, lmx2571_reg_val[0].datamap.data);
		vTaskDelay(900);
	}

}
