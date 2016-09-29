/*
 * dw_watcher.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"
#include "deca_device_api.h"
#include "deca_spi.h"

dspi_rtos_handle_t dw_rtos_handle;

void dw_watcher_thread(void *pvParameters) {

	uint32_t devid = 0;

	volatile SemaphoreHandle_t pll_dw_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[PLL_DW_SEM];
	volatile SemaphoreHandle_t dw_app_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_APP_SEM];
	volatile SemaphoreHandle_t dw_spi_mutex = ((struct TaskSharedInfo *) pvParameters)->mutex[DW_SPI_MUTEX];

	// initialize SPI device
	dw_communication_slow_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

	while(true) {

		/*
		 * Reset DW chip
		 */
		DW1000_RST_INIT_OUT(LOGIC_DW1000_RST_ASSERT);

		/*
		 * Initialize SPI device
		 */

		/*
		 * Wait for PLL lock
		 */
		xSemaphoreTake(pll_dw_sem, portMAX_DELAY);	// wait for PLL lock
//		PRINTF("got pll_locked message\r\n");

		/*
		 * Start the decawave chip
		 */
		DW1000_RST_INIT_IN();	// Change the DW reset pin to input mode

		/*
		 * Wait for stabilization
		 */
		// TODO: replace with interrupt stabilization later
		while(!GPIO_ReadPinInput(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_RST_PIN));

		devid = dwt_readdevid();
		PRINTF("DW ID: %08x\r\n", devid);


		/*
		 * Reset it to sync with PPS line
		 */

		/*
		 * Wait for stabilization
		 */

		/*
		 * Perform periodic checks on state
		 * If the radio has failed, restart the process and inform all dependent tasks
		 */

//		vTaskDelay(500);

//		dw_state = 1;
//		xQueueSendToBack(*dw_app_queue, &dw_state, 0);
		xSemaphoreGive(dw_app_sem);

		vTaskSuspend(NULL);
	}

}
