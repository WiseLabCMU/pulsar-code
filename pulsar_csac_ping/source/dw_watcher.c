/*
 * dw_watcher.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"

void dw_watcher_thread(void *pvParameters) {

//	int pll_state = -1;
//	int dw_state = -1;
	volatile SemaphoreHandle_t *pll_dw_sem = &((QueueHandle_t *)pvParameters)[PLL_DW_SEM];
	volatile SemaphoreHandle_t *dw_app_sem = &((QueueHandle_t *)pvParameters)[DW_APP_SEM];

//	*dw_app_queue = xQueueCreate(1, sizeof(int));
//		if(*dw_app_queue == NULL) {
//			vTaskSuspend(NULL);
//		}

//	PRINTF("dw_watcher_thread started\r\n");

//	while(true) {
//		while(*pll_dw_queue == NULL) {
//			vTaskDelay(100);
//		}
//		while(true) {
//			while(xQueueReceive(*pll_dw_queue, &pll_state, 100) != pdTRUE);
//			if(pll_state == 1) break;
//		}

		xSemaphoreTake(*pll_dw_sem, portMAX_DELAY);
		PRINTF("got pll_locked message\r\n");

//		dw_state = 1;
//		xQueueSendToBack(*dw_app_queue, &dw_state, 0);
		xSemaphoreGive(*dw_app_sem);
		PRINTF("sent dw_locked message\r\n");

		vTaskDelay(100);
//	}

	vTaskSuspend(NULL);
}
