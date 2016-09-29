/*
 * application_thread.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"
#include "deca_device_api.h"

// Decawave radio configuration
static const dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/*
 * The application thread needs access to the CSAC and DW worker threads for interaction
 */
void application_thread(void *pvParameters) {

	volatile SemaphoreHandle_t dw_app_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_APP_SEM];
	/*
	 * Wait for lock release from DW watcher
	 */
	xSemaphoreTake(dw_app_sem, portMAX_DELAY);

	/*
	 * All preliminary systems are now locked and ready, prepare for main operation
	 */

	vTaskSuspend(NULL);
}
