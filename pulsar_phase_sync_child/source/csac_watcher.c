/*
 * csac_watcher.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"

#include "csac_driver.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * CSAC WATCHER section
 *****************************************************************************/
/*
 * NOTE: currently the watcher task is directly checking the csac state.
 * This function may later want to be moved to a worker task
 */


volatile TickType_t timeout = 20;
size_t nRecv = 0;
uint8_t csacRecvBuffer[1];
volatile size_t totalBytes = 0;



void csac_watcher_task(void *pvParameters) {
	uint8_t buf[8];
//	TickType_t timeout = 20;
	size_t nRecv = 0;
	uint8_t csacRecvBuffer[1];

	// initialize lock pin for later use
//	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

	// initialize UART device for communications
	csac_communication_init(&csac_uart_handle, BOARD_CSAC_UART_CLK_FREQ, buf, sizeof(buf));

	while(true) {

		if(csac_get_state(&csac_uart_handle) == CSAC_LOCKED) {

			// disable disciplining
			csac_disable_pps_discipline(&csac_uart_handle);


			xSemaphoreGive(semaphore[CSAC_PLL_OK_SEM]);
			PRINTF("[%u] CSAC lock detected\r\n", xTaskGetTickCount());
			vTaskSuspend(NULL);		// replace this with something nicer once failure handling is dealt with
		}

	}

	vTaskSuspend(NULL);
}

/******************************************************************************
 * end of CSAC WATCHER section
 *****************************************************************************/

