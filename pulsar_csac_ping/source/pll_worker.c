/*
 * pll_worker.c
 *
 * The pll_worker_thread is responsible for any communication and interfacing
 * with the lmx2571 PLL on the Pulsar board.
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"
#include "application_threads.h"
#include "pll_driver.h"





/**
 * Approximate delay by spinning
 * @param microsec Microseconds to wait
 */
static inline void delay(uint32_t microsec);

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

void pll_worker_thread(void *pvParameters) {
	union lmx2571_register reg_local;	// some space for local storage
	struct pllWorkerCmdStruct *cmdStruct = (struct pllWorkerCmdStruct *) pvParameters;
	status_t status = kStatus_Timeout;	// default q return from thread
	int i;
	switch(cmdStruct->cmd) {

		// do nothing
		case cmdPLL_NONE:
			status = kStatus_Success;
			break;

		// reset the PLL and wait till it restarts
		case cmdPLL_RESET:
			GPIO_SetPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN);	// turn CE = HIGH
			delay(100);											// wait for LDOs to stabilize
			memset((void*) &reg_local, 0, PLL_REG_BYTESIZE);	// clear TX register
			reg_local.R0.RESET = 1;								// set reset bit
			pll_register_write(cmdStruct->spiHandle, R0_addr, reg_local.datamap.data);	// reset chip
			delay(1000);	// empirical time to wait for lock.
							// TODO: maybe replace with a state check loop later
			status = kStatus_Success;
			break;

		// program the PLL with the default register values for 10MHz -> 38.4 MHz
		case cmdPLL_INIT_DEFAULT:
			for(i = 0; i < LMX2571_NREG; i++) {
				// reuse the status provided by the SPI function
				status = pll_register_write(cmdStruct->spiHandle, lmx2571_reg_val[i].datamap.ADDRESS, lmx2571_reg_val[i].datamap.data);
				if(status != kStatus_Success) break;	// something went wrong
				delay(100);
			}
			break;

		// read the register with provided address and respond in data section
		case cmdPLL_READ_REG:
			cmdStruct->data->datamap.data = pll_register_read(cmdStruct->spiHandle, cmdStruct->data->datamap.ADDRESS);
			status = kStatus_Success;
			break;

		// write the register and value in data
		case cmdPLL_WRITE_REG:
			status = pll_register_write(cmdStruct->spiHandle, cmdStruct->data->datamap.ADDRESS, cmdStruct->data->datamap.data);
			break;

		// disable the PLL by pulling the CE line low
		case cmdPLL_DISABLE:
			GPIO_ClearPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN);
			status = kStatus_Success;
			break;

		// default action for unknown commands
		default:
			status = kStatus_InvalidArgument;
	}

	// task completed
	// inform the parent about status through provided queue
	if(cmdStruct->qHandle != NULL) {
		// parent task is expecting a response in queue
		// we assume the parent can manage the space in it's own queue
		xQueueSendToBack(cmdStruct->qHandle, (void *) &status, 0);
	}

	vTaskSuspend(NULL);	// suspend worker task
}

// TODO: replace with PIT based delay function for everyone
static inline void delay(uint32_t microsec) {
	// the core clock is 80MHz
	volatile uint32_t i = 8U * microsec;
	while (i--) {
	asm("nop");
	}
}
