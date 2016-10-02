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
 * This application will implement a 3 message exchange
 *
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

/**
 * FreeRTOS includes
 */

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

/**
 * Set of shared synchronization semaphores for various tasks

 */
volatile SemaphoreHandle_t sem[SEM_N];	// general semaphores for synchronization

/**
 * Set of shared queues between tasks
 */
volatile QueueHandle_t task_queues[Q_N];		// allocate space for general purpose queues

/**
 * Set of shared task handles between all tasks
 */
volatile TaskHandle_t task_handle[TASK_N];

/**
 * Set of peripheral access mutexes
 */
volatile SemaphoreHandle_t mutex[MUTEX_N];

struct TaskSharedInfo taskInfo = {
		.semaphores = sem,
		.mutex		= mutex,
		.queues 	= task_queues,
		.tasks 		= task_handle,
};

/*
 * CONFIGURATION SECTION
 */

// Empty for now


/*
 * Interrupt handlers go here
 */

static volatile uint32_t irq_reg;

void PORTC_IRQHandler(void) {
	irq_reg = PORT_GetPinsInterruptFlags(PORTC);

	/*
	 * Handle Decawave IRQ from here (by "giving" the appropriate semaphore)
	 */
	if(irq_reg & (1U<<BOARD_DW1000_GPIO_IRQ_PIN)) {
		xSemaphoreGiveFromISR(sem[DW_IRQ_SEM], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOC, (1U<<BOARD_DW1000_GPIO_IRQ_PIN));
	}

}

/*!
 * @brief Application entry point.
 */
int main(void) {
	/* Init board hardware. */
	BOARD_ConfigPinmux();


	BOARD_InitPins();
	BOARD_BootClockHSRUN();

	BOARD_InitDebugConsole();

	// create necessary semaphores
	sem[CSAC_PLL_SEM] = xSemaphoreCreateBinary();
	sem[CSAC_WORK_P_SEM] = xSemaphoreCreateBinary();
	sem[CSAC_WORK_C_SEM] = xSemaphoreCreateBinary();
	sem[PLL_DW_SEM] = xSemaphoreCreateBinary();
	sem[DW_APP_SEM] = xSemaphoreCreateBinary();
	sem[DW_IRQ_SEM] = xSemaphoreCreateBinary();

	// create necessary queues
	task_queues[HEARTBEAT_Q] = xQueueCreate(1, sizeof(int));	// queue for heartbeat task
	task_queues[CSAC_WORKER_RESP_Q] = xQueueCreate(1, sizeof(int));	// queue for CSAC worker responses

	// create necessary mutexes
	mutex[CSAC_UART_MUTEX] = xSemaphoreCreateMutex();
	mutex[PLL_SPI_MUTEX] = xSemaphoreCreateMutex();
	mutex[DW_SPI_MUTEX] = xSemaphoreCreateMutex();

	// task for debug and control
	// there can only be one of these
	xTaskCreate(commander_thread, "Command", configMINIMAL_STACK_SIZE, (void *) &taskInfo, commander_PRIORITY, task_handle[COMMAND]);

	// low priority task for heartbeat LED control
	// there can only be one of these
	xTaskCreate(heartbeat_thread, "Heartbeat", configMINIMAL_STACK_SIZE, (void *) &taskInfo, heartbeat_PRIORITY, task_handle[HEARTBEAT]);

	// thread watches over and configures CSAC for proper operation
	// there can only be one of these
	xTaskCreate(csac_watcher_thread, "CSACwatch", configMINIMAL_STACK_SIZE, (void *) &taskInfo, watcher_PRIORITY, task_handle[CSAC_WATCHER]);

	// TODO: add description here
	xTaskCreate(csac_worker_thread, "CSACwork", configMINIMAL_STACK_SIZE, (void *) &taskInfo, worker_PRIORITY, task_handle[CSAC_WORKER]);

	// thread watches over and configures PLL for proper operation
	// there can only be one of these
	xTaskCreate(pll_watcher_thread, "PLLwatch", configMINIMAL_STACK_SIZE, (void *) &taskInfo, watcher_PRIORITY, task_handle[PLL_WATCHER]);

	// thread watches over and configures Decawave radio for proper operation
	// there can only be one of these
	xTaskCreate(dw_watcher_thread, "DWwatch", configMINIMAL_STACK_SIZE, (void *) &taskInfo, watcher_PRIORITY, task_handle[DW_WATCHER]);

	// Application thread. All the magic happens here (hopefully) :-)
	xTaskCreate(messaging_thread, "Messenger", configMINIMAL_STACK_SIZE, (void *) &taskInfo, application_PRIORITY, task_handle[APPLICATION]);

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



/*
 * Decawave interrupt handler
 */
