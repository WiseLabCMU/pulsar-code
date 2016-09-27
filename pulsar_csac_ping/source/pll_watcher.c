/*
 * pll_watcher.c
 *
 *  Created on: Sep 17, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"
#include "application_threads.h"
#include "pll_driver.h"
#include "fsl_debug_console.h"

void pll_watcher_thread(void *pvParameters) {

	/*
	 * Local data
	 */

//	int csac_state = -1;
//	int pll_state = -1;
//	SemaphoreHandle_t 	pllMutex = NULL;	// mutex to access PLL
	dspi_rtos_handle_t 	pllCommHandle;		// SPI handle to peripheral
	volatile SemaphoreHandle_t *csac_pll_sem = &((QueueHandle_t *)pvParameters)[CSAC_PLL_SEM];
	volatile SemaphoreHandle_t *pll_dw_sem = &((QueueHandle_t *)pvParameters)[PLL_DW_SEM];

	/*
	 * COMM INIT: initialize communication
	 */

	pll_communication_init(&pllCommHandle, CLOCK_GetFreq(BOARD_PLL_SPI_MASTER_CLK_SRC));

	// TODO: initialize CE line as output
	// TODO: initialize mutex if not already initialized
//	*pll_dw_queue = xQueueCreate(1, sizeof(int));
//	if(*pll_dw_queue == NULL) {
//		vTaskSuspend(NULL);
//	}

//	PRINTF("pll_watcher_thread started\r\n");

	while(true) {


		/*
		 * spawn worker thread
		 */
//		xTaskCreate(pll_worker_thread, "PLLworker", configMINIMAL_STACK_SIZE, NULL, worker_PRIORITY, NULL);

		// wait for csac_watcher to respond with locked state
//		while(*csac_pll_queue == NULL) {
//			vTaskDelay(100);
//		}
//		while(true) {
//			while(xQueueReceive(*csac_pll_queue, &csac_state, 100) != pdTRUE);
//			if(csac_state == 1) break;
//		}

		xSemaphoreTake(*csac_pll_sem, portMAX_DELAY);


		PRINTF("got csac_locked message\r\n");

		/*
		 * DISABLE PHASE: Hold PLL in reset. This is done by pulling the CE line to ground
		 */


		/*
		 * WAIT PHASE: Wait for CSAC lock (notification from CSAC thread)
		 */

		/*
		 * PROGRAM PHASE: Program the PLL with default values
		 */

		/*
		 * OPERATIONAL PHASE: PLL should now be operational
		 */

		// inform decawave watcher immediately of lock
//		pll_state = 1;
//		xQueueSendToBack(*pll_dw_queue, &pll_state, 0);

		xSemaphoreGive(*pll_dw_sem);
		PRINTF("sent pll_locked message\r\n");

		/*
		 * FAIL DETECT: Check lock state. If lock failed, restart from reset phase
		 */
		vTaskDelay(100);
	}
	pll_communication_deinit(&pllCommHandle);
	vTaskSuspend(NULL);
}
