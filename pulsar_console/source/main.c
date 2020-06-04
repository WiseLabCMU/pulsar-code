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
 * This would implement the CSAC console.
 * Re-implement all previous systems using the correct interrupt configurations
 **/

/******************************************************************************
 * INCLUDES section
 *****************************************************************************/

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "pulsar_board.h"
#include "pin_mux.h"
#include "clock_config.h"

/**
 * FreeRTOS includes
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "fsl_debug_console.h"
#include "fsl_uart_freertos.h"


//#include "deca_regs.h"
//
//#include "deca_spi.h"
//#include "deca_device_api.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * end of INCLUDES section
 *****************************************************************************/

/******************************************************************************
 * SHARED OBJECT section
 *****************************************************************************/

volatile SemaphoreHandle_t semaphore[SEM_N];

volatile TaskHandle_t tasks[TASK_N];

volatile QueueHandle_t queues[QUEUE_N];

// The CSAC UART is currently shared, but should not be
uart_rtos_handle_t csac_uart_handle;	// CSAC UART

/******************************************************************************
 * end of SHARED OBJECT section
 *****************************************************************************/

/******************************************************************************
 * INTERRUPTS section
 *****************************************************************************/

static volatile uint32_t portA_irq_reg;

void PORTA_IRQHandler(void) {
	portA_irq_reg = PORT_GetPinsInterruptFlags(PORTA);

	if(portA_irq_reg & (1U<<BOARD_CSAC_PPSOUT_PIN)) {
		xSemaphoreGiveFromISR(semaphore[DW_PPS_SEM], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOA, (1U<<BOARD_CSAC_PPSOUT_PIN));
	}

}

static volatile uint32_t portC_irq_reg;

void PORTC_IRQHandler(void) {
	portC_irq_reg = PORT_GetPinsInterruptFlags(PORTC);

	if(portC_irq_reg & (1U<<BOARD_DW1000_GPIO_IRQ_PIN)) {
		xSemaphoreGiveFromISR(semaphore[DW_IRQ_SEM], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOC, (1U<<BOARD_DW1000_GPIO_IRQ_PIN));
	}

}

/******************************************************************************
 * end of INTERRUPTS section
 *****************************************************************************/

/******************************************************************************
 * MAIN FUNCTION section
 *****************************************************************************/




/*
 * All the device data structures
 * TODO: these may be better if not static
 */




/**
 * main function
 * Application entry point
 * @return 0 to comply with GCC requirements
 */
int main(void) {
	/* Init board hardware. */
	BOARD_ConfigPinmux();		// Setup pin modes
	BOARD_InitPins();			// Initialize LEDs
	BOARD_BootClockHSRUN();		// Run processor at high-speed
	BOARD_InitDebugConsole();	// use debug console

	// create common semaphores required for task synchronization
	for(int i=0; i<SEM_N;i++) {
		semaphore[i] = xSemaphoreCreateBinary();
	}

	NVIC_SetPriority(PORTA_IRQn, 5); // GPIO interrupt. 5 is the max allowed interrupt priority with RTOS features
	NVIC_SetPriority(PORTC_IRQn, 5); // GPIO interrupt. 5 is the max allowed interrupt priority with RTOS features
	NVIC_SetPriority(UART0_RX_TX_IRQn, 5);	// Console UART interrupt
	NVIC_SetPriority(UART1_RX_TX_IRQn, 5);	// CSAC UART interrupt
	NVIC_SetPriority(SPI1_IRQn, 5);	// PLL SPI interrupt
	NVIC_SetPriority(SPI0_IRQn, 5);	// DW SPI interrupt

	/* Create RTOS tasks */
	xTaskCreate(command_task, "command", configMINIMAL_STACK_SIZE, NULL, commander_PRIORITY, tasks[COMMAND_TASK]);
	xTaskCreate(heartbeat_task, "heartbeat", configMINIMAL_STACK_SIZE, NULL, heartbeat_PRIORITY, tasks[HEARTBEAT_TASK]);
//	xTaskCreate(csac_watcher_task, "CSAC", configMINIMAL_STACK_SIZE, NULL + 64, watcher_PRIORITY, tasks[CSAC_WATCHER_TASK]);
//	xTaskCreate(pll_watcher_task, "PLL", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[PLL_WATCHER_TASK]);
//	xTaskCreate(dw_watcher_task, "DW", configMINIMAL_STACK_SIZE, NULL, watcher_PRIORITY, tasks[DW_WATCHER_TASK]);
//	xTaskCreate(messenger_task, "messenger", configMINIMAL_STACK_SIZE + 64, NULL, application_PRIORITY, tasks[MESSENGER_TASK]);
//	xTaskCreate(discipline_task, "disc", configMINIMAL_STACK_SIZE + 200, NULL, application_PRIORITY, tasks[DISCIPLINE_TASK]);

	vTaskStartScheduler();

	for(;;) { /* Infinite loop to avoid leaving the main function */
		__asm("NOP"); /* something to use as a breakpoint stop while looping */
	}

	return 0;
}

/******************************************************************************
 * end of MAIN FUNCTION section
 *****************************************************************************/
