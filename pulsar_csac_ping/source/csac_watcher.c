/*
 * csac_watcher.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"

void csac_watcher_thread(void *pvParameters) {

//	int state = -1;
	struct csacWorkerCmdStruct workerCmdStruct;
	volatile SemaphoreHandle_t *csac_pll_sem = &((SemaphoreHandle_t *)pvParameters)[CSAC_PLL_SEM];
//	PRINTF("csac_watcher_thread started\r\n");
	/*
	 * initialize peripherals
	 */
	// TODO: initialize LOCK pin as GPIO input
	// TODO: initialize UART device

	/*
	 * initialize mutex and queues
	 */
	// TODO: create mutex for worker locks
//	*csac_pll_queue = xQueueCreate(1, sizeof(int));
//	if(*csac_pll_queue == NULL) {
//		vTaskSuspend(NULL);
//	}

	while(true) {
		/*
		 * wait for lock pin to turn high
		 */

		/*
		 * check CSAC state over UART
		 */
		// create and assign worker thread

		// wait for worker response (through timeout?)



//		vTaskDelay(1000);	// temporary command

		// get rid of worker

		// inform pll_watcher immediately that CSAC has locked through queue
//		state = 1;
//		xQueueSendToBack(*csac_pll_queue, &state, 0);
		xSemaphoreGive(*csac_pll_sem);
		PRINTF("sent csac_locked message\r\n");

		/*
		 * Wait and repeat
		 */
		vTaskDelay(1000);	// temporary command
	}
	// TODO: de-initialize peripherals
	vTaskSuspend(NULL);
}
