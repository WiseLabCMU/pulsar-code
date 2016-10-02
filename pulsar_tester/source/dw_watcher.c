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

static inline void delay(uint32_t microsec);

void dw_watcher_thread(void *pvParameters) {

	// scratch space
	uint32_t devid = 0;
	uint64_t dw_time1 = 0;
	uint64_t dw_time2 = 0;
	uint32_t *dw_time132 = (uint32_t *) &dw_time1;
	uint32_t *dw_time232 = (uint32_t *) &dw_time2;
	uint32_t ec_ctrl_val = 0;

	volatile SemaphoreHandle_t pll_dw_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[PLL_DW_SEM];
	volatile SemaphoreHandle_t dw_app_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_APP_SEM];
//	volatile SemaphoreHandle_t dw_spi_mutex = ((struct TaskSharedInfo *) pvParameters)->mutex[DW_SPI_MUTEX];

	// initialize PPSOUT as digital input
	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

	while(true) {

		// initialize SPI device
		dw_communication_slow_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

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

		dwt_initialise(DWT_LOADUCODE);

		dw_communication_fast_init(&dw_rtos_handle, CLOCK_GetFreq(BOARD_DW1000_SPI_CLKSRC));

		// wait for PPS pin to go high: simple read loop

		while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN) == 0);
		while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN) == 1);

		delay(1000);	// wait for a while

		// program external sync functions

		// This is programmed on the EXT_SYNC register
		// enable one shot reset mode bit
		// enable PLL lock bits
		// enable delay time of 1 count on EXT_CLK like
		ec_ctrl_val = EC_CTRL_OSTRM | EC_CTRL_PLLLCK | (1U << 3);
		dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);
		dwt_readsystime((uint8 *) &dw_time1);

		// wait for pin to go high again
		while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN) == 0);

		// wait for 10 millisecs for stabilization to complete
		delay(1000);
		ec_ctrl_val = 0;
		dwt_write32bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, ec_ctrl_val);
		dwt_readsystime((uint8 *) &dw_time2);

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
