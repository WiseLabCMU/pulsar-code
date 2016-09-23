/*
 * pll_watcher.c
 *
 *  Created on: Sep 17, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"
#include "application_threads.h"
#include "pll_driver.h"

void pll_watcher_thread(void *pvParameters) {

	/*
	 * Local data
	 */
	dspi_rtos_handle_t pllCommHandle;

	/*
	 * initialize communication
	 */

	pll_communication_init(&pllCommHandle, CLOCK_GetFreq(BOARD_PLL_SPI_MASTER_CLK_SRC));
}
