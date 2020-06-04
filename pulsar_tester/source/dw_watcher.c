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
#include "deca_regs.h"

dspi_rtos_handle_t dw_rtos_handle;
volatile uint32_t dw_pps_count;		// PPS cycles since the DW was synchronized to CSAC

static inline void delay(uint32_t microsec);

void dw_watcher_thread(void *pvParameters) {

	// scratch space
	uint32_t ec_ctrl_val = 0;	// local storage for external control register

	volatile SemaphoreHandle_t pll_dw_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[PLL_DW_SEM];
	volatile SemaphoreHandle_t dw_app_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_APP_SEM];
	volatile SemaphoreHandle_t dw_pps_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_PPS_SEM];
//	volatile SemaphoreHandle_t dw_spi_mutex = ((struct TaskSharedInfo *) pvParameters)->mutex[DW_SPI_MUTEX];



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
		xSemaphoreTake(pll_dw_sem, portMAX_DELAY);	// wait for PLL lock

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

		PRINTF("starting DW startup process\r\n");

		// load microcode instruction to DW
		dwt_initialise(DWT_LOADUCODE);

		/*
		 * Initialize SPI device in fast mode since microcode is now in place
		 */
		dw_communication_fast_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		/*
		 * Wait for PPS pin rising edge
		 */
		PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptRisingEdge);
		EnableIRQ(PORTA_IRQn);		// enable IRQ
		GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

		PRINTF("waiting for PPS\r\n");

		if(xSemaphoreTake(dw_pps_sem, 10) != pdTRUE) {
			// handle error
			PRINTF("could not find 1st PPS. restart DW\r\n");
			xSemaphoreGive(pll_dw_sem);	// give back PPS lock semaphore
			continue;
		}

		delay(1000);	// wait for a while

		PRINTF("acquired 1st PPS edge\r\n");

		// program external sync functions

		// This is programmed on the EXT_SYNC register
		// enable one shot reset mode bit
		// enable PLL lock bits
		// enable delay time of 1 count on EXT_CLK like
		ec_ctrl_val = EC_CTRL_OSTRM | EC_CTRL_PLLLCK | (1U << 3);
		dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);
//		dwt_readsystime((uint8 *) &dw_time1);


		// wait for another PPS edge
//		while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN) == 0);

		if(xSemaphoreTake(dw_pps_sem, 0) != pdTRUE) {
			// handle error
			PRINTF("could not find 2nd PPS. restart DW\r\n");
			xSemaphoreGive(pll_dw_sem);	// give back PPS lock semaphore
			continue;
		}

		// wait for 10 millisecs for stabilization to complete
		delay(1000);
		ec_ctrl_val = 0;
		dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);
//		dwt_readsystime((uint8 *) &dw_time2);
		dw_pps_count = 0;

		PRINTF("acquired 2nd PPS edge\r\n");


		/*
		 * Configure and enable GPIO interrupts
		 */
		PORT_SetPinInterruptConfig(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_IRQ_PIN, kPORT_InterruptRisingEdge);
		EnableIRQ(PORTC_IRQn);		// enable IRQ
		GPIO_PinInit(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_IRQ_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

		xSemaphoreGive(dw_app_sem);	// hand over to application thread

		vTaskSuspend(NULL);
	}

}

// TODO: replace with PIT based delay function for everyone
static inline void delay(uint32_t microsec) {
	// the core clock is 80MHz
	volatile uint32_t i = 8U * microsec;
	while (i--) {
	asm("nop");
	}
}
