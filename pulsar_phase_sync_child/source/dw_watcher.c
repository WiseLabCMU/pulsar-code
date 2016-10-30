/*
 * dw_watcher.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"

#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_device_api.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * DECAWAVE WATCHER section
 *****************************************************************************/

dspi_rtos_handle_t dw_rtos_handle;	// Deacwave SPI

int resync_dw_to_pps() {
	uint32_t ec_ctrl_val = 0;	// local storage for external control register

	// clear semaphore in case it was already given by a previous instance of the IRQ
	xSemaphoreTake(semaphore[DW_PPS_SEM], 0);

	// enable interrupts on pin
	PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptRisingEdge);

	// wait for PPS edge
	if(xSemaphoreTake(semaphore[DW_PPS_SEM], 1100) != pdTRUE) {
		PRINTF("[%u] err: unable to find 1st PPS\r\n", xTaskGetTickCount());
		return -1;
	}

	vTaskDelay(10);



	// program reset on next PPS edge
	ec_ctrl_val = EC_CTRL_OSTRM | EC_CTRL_PLLLCK | (1U << 3);
	dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);

	// wait for next PPS edge
	if(xSemaphoreTake(semaphore[DW_PPS_SEM], 1100) != pdTRUE) {
		PRINTF("[%u] err: unable to find 2nd PPS\r\n", xTaskGetTickCount());
		return -2;
	}

	vTaskDelay(10);

	ec_ctrl_val = 0;
	dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);

	// clear ext sync bit in DW
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ESYNCR);

	// disable PPS pin interrupt
	PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptOrDMADisabled);

	while(!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_CPLOCK));
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_CPLOCK);	// reset PLL lock bit in decawave

	return 0;
}

void dw_watcher_task(void *pvParameters) {
	int sync_status;

	while(true) {
		/*
		 * Initialize SPI device
		 */
		dw_communication_slow_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		/*
		 * Reset DW chip by asserting the reset pin to ground
		 */
		DW1000_RST_INIT_OUT(LOGIC_DW1000_RST_ASSERT);

		/*
		 * Wait for PLL lock. The PLL watcher thread will give a semaphore once this happens
		 */
		xSemaphoreTake(semaphore[PLL_DW_OK_SEM], portMAX_DELAY);	// wait for PLL lock

		/*
		 * Start the decawave chip by de-asserting the reset line.
		 * It will go high once the device is on
		 */
		DW1000_RST_INIT_IN();	// Change the DW reset pin to input mode

		/*
		 * Wait for stabilization
		 */
		// TODO: replace with interrupt stabilization later
		while(!GPIO_ReadPinInput(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_RST_PIN));

		PRINTF("[%u] begin DW startup\r\n", xTaskGetTickCount());

		// load microcode instruction to DW
		dwt_initialise(DWT_LOADUCODE);

		/*
		 * Initialize SPI device in fast mode since microcode is now in place
		 */
		dw_communication_fast_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		EnableIRQ(PORTA_IRQn);		// enable IRQ on PORTA
		GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

		// TODO: add a loop around this for repeat requests
		while(true) {
			sync_status = resync_dw_to_pps();
			if(sync_status == 0) {
				// all good, pass control to next task
				break;
			} else {
				// we couldn't lock. try locking again
				PRINTF("[%u] unable to lock DW to PPS: %d", xTaskGetTickCount(), sync_status);
			}
		}
		PRINTF("[%u] DW clock locked to local PPS\r\n", xTaskGetTickCount());

		DisableIRQ(PORTA_IRQn);		// disable IRQ on PORTA

		xSemaphoreGive(semaphore[DW_MSG_OK_SEM]);
		vTaskSuspend(NULL);
	}
}

/******************************************************************************
 * end of DECAWAVE WATCHER section
 *****************************************************************************/
