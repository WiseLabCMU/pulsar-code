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

#include "pulsar_board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/**
 * FreeRTOS includes
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

struct shared_objs {
	volatile SemaphoreHandle_t *semaphores;
};

/* Task priorities. */
#define counter_task_PRIORITY (configMAX_PRIORITIES - 1)

static void counter_task(void *pvParameters);

volatile SemaphoreHandle_t sem[1];

struct shared_objs objs = {
		.semaphores = sem,
};

/*
 * Interrupt functions
 */
void PORTA_IRQHandler(void) {

	if(PORT_GetPinsInterruptFlags(PORTA) & (1U<<BOARD_CSAC_PPSOUT_PIN)) {
		xSemaphoreGiveFromISR(sem[0], NULL);
		GPIO_ClearPinsInterruptFlags(GPIOA, (1U<<BOARD_CSAC_PPSOUT_PIN));
	}

}


/*!
 * @brief Application entry point.
 */
int main(void) {
  /* Init board hardware. */
	BOARD_ConfigPinmux();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	sem[0] = xSemaphoreCreateBinary();

	NVIC_SetPriority(PORTA_IRQn, 5);
	PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptRisingEdge);
	EnableIRQ(PORTA_IRQn);		// enable IRQ
	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});



  /* Add your code here */

  /* Create RTOS task */
  xTaskCreate(counter_task, "Count_task", configMINIMAL_STACK_SIZE, (void *) &objs, counter_task_PRIORITY, NULL);
  vTaskStartScheduler();

  for(;;) { /* Infinite loop to avoid leaving the main function */
    __asm("NOP"); /* something to use as a breakpoint stop while looping */
  }
}

/*!
 * @brief Task responsible for counting number of PPS pulses
 */
static void counter_task(void *pvParameters) {
	uint32_t count = 0;
	volatile SemaphoreHandle_t pps_sem = ((struct shared_objs *) pvParameters)->semaphores[0];

	// enable interrupts and pin configurations


	while(true) {
		if(xSemaphoreTake(pps_sem, 2000) == pdTRUE) {
			count++;
			PRINTF("count = %u\r\n", count);
		} else {
			PRINTF("err\r\n");
		}
	}
}



