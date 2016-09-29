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

/**
 * Useful constants
 */

static const union lmx2571_register lmx2571_reg_val[LMX2571_NREG] = {
	{.datamap = {.data = 0x4000, .ADDRESS = R60_addr, .RW = CMD_WRITE}}, // R60
	{.datamap = {.data = 0x0C00, .ADDRESS = R58_addr, .RW = CMD_WRITE}}, // R58
	{.datamap = {.data = 0x2802, .ADDRESS = R53_addr, .RW = CMD_WRITE}}, // R53
	{.datamap = {.data = 0x0000, .ADDRESS = R47_addr, .RW = CMD_WRITE}}, // R47
	{.datamap = {.data = 0x0210, .ADDRESS = R42_addr, .RW = CMD_WRITE}}, // R42
	{.datamap = {.data = 0x0810, .ADDRESS = R41_addr, .RW = CMD_WRITE}}, // R41
	{.datamap = {.data = 0x101C, .ADDRESS = R40_addr, .RW = CMD_WRITE}}, // R40
	{.datamap = {.data = 0x11F8, .ADDRESS = R39_addr, .RW = CMD_WRITE}}, // R39
	{.datamap = {.data = 0x0647, .ADDRESS = R35_addr, .RW = CMD_WRITE}}, // R35
	{.datamap = {.data = 0x1000, .ADDRESS = R34_addr, .RW = CMD_WRITE}}, // R34
	{.datamap = {.data = 0x0000, .ADDRESS = R33_addr, .RW = CMD_WRITE}}, // R33
	{.datamap = {.data = 0x0000, .ADDRESS = R32_addr, .RW = CMD_WRITE}}, // R32
	{.datamap = {.data = 0x0000, .ADDRESS = R31_addr, .RW = CMD_WRITE}}, // R31
	{.datamap = {.data = 0x0000, .ADDRESS = R30_addr, .RW = CMD_WRITE}}, // R30
	{.datamap = {.data = 0x0000, .ADDRESS = R29_addr, .RW = CMD_WRITE}}, // R29
	{.datamap = {.data = 0x0000, .ADDRESS = R28_addr, .RW = CMD_WRITE}}, // R28
	{.datamap = {.data = 0x0000, .ADDRESS = R27_addr, .RW = CMD_WRITE}}, // R27
	{.datamap = {.data = 0x0000, .ADDRESS = R26_addr, .RW = CMD_WRITE}}, // R26
	{.datamap = {.data = 0x0000, .ADDRESS = R25_addr, .RW = CMD_WRITE}}, // R25
	{.datamap = {.data = 0x0010, .ADDRESS = R24_addr, .RW = CMD_WRITE}}, // R24
	{.datamap = {.data = 0x1024, .ADDRESS = R23_addr, .RW = CMD_WRITE}}, // R23
	{.datamap = {.data = 0x8584, .ADDRESS = R22_addr, .RW = CMD_WRITE}}, // R22
	{.datamap = {.data = 0x0101, .ADDRESS = R21_addr, .RW = CMD_WRITE}}, // R21
	{.datamap = {.data = 0x1028, .ADDRESS = R20_addr, .RW = CMD_WRITE}}, // R20
	{.datamap = {.data = 0x0000, .ADDRESS = R19_addr, .RW = CMD_WRITE}}, // R19
	{.datamap = {.data = 0x0000, .ADDRESS = R18_addr, .RW = CMD_WRITE}}, // R18
	{.datamap = {.data = 0x0000, .ADDRESS = R17_addr, .RW = CMD_WRITE}}, // R17
	{.datamap = {.data = 0x0000, .ADDRESS = R16_addr, .RW = CMD_WRITE}}, // R16
	{.datamap = {.data = 0x0000, .ADDRESS = R15_addr, .RW = CMD_WRITE}}, // R15
	{.datamap = {.data = 0x0000, .ADDRESS = R14_addr, .RW = CMD_WRITE}}, // R14
	{.datamap = {.data = 0x0000, .ADDRESS = R13_addr, .RW = CMD_WRITE}}, // R13
	{.datamap = {.data = 0x0000, .ADDRESS = R12_addr, .RW = CMD_WRITE}}, // R12
	{.datamap = {.data = 0x0000, .ADDRESS = R11_addr, .RW = CMD_WRITE}}, // R11
	{.datamap = {.data = 0x0000, .ADDRESS = R10_addr, .RW = CMD_WRITE}}, // R10
	{.datamap = {.data = 0x0000, .ADDRESS = R9_addr,  .RW = CMD_WRITE}}, // R9
	{.datamap = {.data = 0x0000, .ADDRESS = R8_addr,  .RW = CMD_WRITE}}, // R8
	{.datamap = {.data = 0x0C64, .ADDRESS = R7_addr,  .RW = CMD_WRITE}}, // R7
	{.datamap = {.data = 0x1302, .ADDRESS = R6_addr,  .RW = CMD_WRITE}}, // R6
	{.datamap = {.data = 0x0201, .ADDRESS = R5_addr,  .RW = CMD_WRITE}}, // R5
	{.datamap = {.data = 0x10D7, .ADDRESS = R4_addr,  .RW = CMD_WRITE}}, // R4
	{.datamap = {.data = 0x1200, .ADDRESS = R3_addr,  .RW = CMD_WRITE}}, // R3
	{.datamap = {.data = 0xE200, .ADDRESS = R2_addr,  .RW = CMD_WRITE}}, // R2
	{.datamap = {.data = 0x7A04, .ADDRESS = R1_addr,  .RW = CMD_WRITE}}, // R1
	{.datamap = {.data = 0x0003, .ADDRESS = R0_addr,  .RW = CMD_WRITE}}, // R0
};

// TODO: replace with PIT based delay function for everyone
static inline void delay(uint32_t microsec) {
	// the core clock is 80MHz
	volatile uint32_t i = 8U * microsec;
	while (i--) {
	asm("nop");
	}
}

void pll_watcher_thread(void *pvParameters) {

	/*
	 * Local data
	 */

	union lmx2571_register reg_local;	// some space for local storage

	dspi_rtos_handle_t 	pllCommHandle;		// SPI handle to peripheral
	volatile SemaphoreHandle_t csac_pll_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_PLL_SEM];
	volatile SemaphoreHandle_t pll_dw_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[PLL_DW_SEM];
	volatile SemaphoreHandle_t pll_spi_mutex = ((struct TaskSharedInfo *) pvParameters)->mutex[PLL_SPI_MUTEX];

	/*
	 * COMM INIT: initialize communication
	 */

	while(true) {

		/*
		 * DISABLE PHASE: Hold PLL in reset.
		 * This is done by pulling the CE line to ground
		 */
		PLL_CE_INIT(PLL_CE_DISABLE);
		pll_communication_init(&pllCommHandle, CLOCK_GetFreq(BOARD_PLL_SPI_MASTER_CLK_SRC));

		/*
		 * WAIT PHASE: Wait for CSAC lock (notification from CSAC watcher)
		 */
		xSemaphoreTake(csac_pll_sem, portMAX_DELAY);	// allow the task to block forever

		/*
		 * PROGRAM PHASE: Program the PLL with default values
		 */

		PLL_ENABLE();

		delay(100);

		memset((void*) &reg_local, 0, PLL_REG_BYTESIZE);	// clear local register
		reg_local.R0.RESET = 1;								// set reset bit

		xSemaphoreTake(pll_spi_mutex,5);
		pll_register_write(&pllCommHandle, R0_addr, reg_local.datamap.data);	// reset chip
		xSemaphoreGive(pll_spi_mutex);
		vTaskDelay(1);

		// TODO: send structure over queue
		for(int i = 0; i < LMX2571_NREG; i++) {
			// reuse the status provided by the SPI function
			xSemaphoreTake(pll_spi_mutex,5);
			pll_register_write(&pllCommHandle, lmx2571_reg_val[i].datamap.ADDRESS, lmx2571_reg_val[i].datamap.data);
			xSemaphoreGive(pll_spi_mutex);
			delay(100);
		}

		/*
		 * OPERATIONAL PHASE: PLL should now be operational
		 */

		pll_communication_deinit(&pllCommHandle);
		PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MISO_PIN, kPORT_MuxAsGpio);	// PLL: MISO as GPIO

		// TODO: replace with IRQ based system
		while(!GPIO_ReadPinInput(BOARD_PLL_IO_GPIO, BOARD_PLL_SPI_MISO_PIN)) {
			vTaskDelay(100);
		}

			// PLL locked
			// inform decawave watcher immediately of lock
		xSemaphoreGive(pll_dw_sem);
		PRINTF("PLL lock detected\r\n");

		/*
		 * FAIL DETECT: Check lock state. If lock failed, restart from reset phase
		 * Also inform dependent tasks
		 */

		vTaskSuspend(NULL);	// replace this with semaphore wait


		PORT_SetPinMux(BOARD_PLL_SPI_PORT, BOARD_PLL_SPI_MISO_PIN, BOARD_PLL_SPI_ALT); 	// PLL: SPI1 MISO

	}
}
