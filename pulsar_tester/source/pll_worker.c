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
//static inline void delay(uint32_t microsec);





void pll_worker_thread(void *pvParameters) {
//	union lmx2571_register reg_local;	// some space for local storage
//	struct pllWorkerCmdStruct *cmdStruct = (struct pllWorkerCmdStruct *) pvParameters;
//	status_t status = kStatus_Timeout;	// default q return from thread
//	int i;
//	switch(cmdStruct->cmd) {
//
//		// do nothing
//		case cmdPLL_NONE:
//			status = kStatus_Success;
//			break;
//
//		// reset the PLL and wait till it restarts
//		case cmdPLL_RESET:
//			GPIO_SetPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN);	// turn CE = HIGH
//			delay(100);											// wait for LDOs to stabilize
//			memset((void*) &reg_local, 0, PLL_REG_BYTESIZE);	// clear TX register
//			reg_local.R0.RESET = 1;								// set reset bit
//			pll_register_write(cmdStruct->spiHandle, R0_addr, reg_local.datamap.data);	// reset chip
//			delay(1000);	// empirical time to wait for lock.
//							// TODO: maybe replace with a state check loop later
//			status = kStatus_Success;
//			break;
//
//		// program the PLL with the default register values for 10MHz -> 38.4 MHz
//		case cmdPLL_INIT_DEFAULT:
//			for(i = 0; i < LMX2571_NREG; i++) {
//				// reuse the status provided by the SPI function
//				status = pll_register_write(cmdStruct->spiHandle, lmx2571_reg_val[i].datamap.ADDRESS, lmx2571_reg_val[i].datamap.data);
//				if(status != kStatus_Success) break;	// something went wrong
//				delay(100);
//			}
//			break;
//
//		// read the register with provided address and respond in data section
//		case cmdPLL_READ_REG:
//			cmdStruct->data->datamap.data = pll_register_read(cmdStruct->spiHandle, cmdStruct->data->datamap.ADDRESS);
//			status = kStatus_Success;
//			break;
//
//		// write the register and value in data
//		case cmdPLL_WRITE_REG:
//			status = pll_register_write(cmdStruct->spiHandle, cmdStruct->data->datamap.ADDRESS, cmdStruct->data->datamap.data);
//			break;
//
//		// disable the PLL by pulling the CE line low
//		case cmdPLL_DISABLE:
//			GPIO_ClearPinsOutput(BOARD_PLL_IO_GPIO, 1U << BOARD_PLL_IO_CE_PIN);
//			status = kStatus_Success;
//			break;
//
//		// default action for unknown commands
//		default:
//			status = kStatus_InvalidArgument;
//	}
//
//	// task completed
//	// inform the parent about status through provided queue
//	if(cmdStruct->qHandle != NULL) {
//		// parent task is expecting a response in queue
//		// we assume the parent can manage the space in it's own queue
//		xQueueSendToBack(cmdStruct->qHandle, (void *) &status, 0);
//	}

	vTaskSuspend(NULL);	// suspend worker task
}


