/*
 * pll_driver.h
 *
 *  Created on: Sep 10, 2016
 *      Author: adwait
 */

// TODO: rewrite this driver assuming pre-defined HW config and access to board files
#ifndef DRIVERS_PLL_DRIVER_H_
#define DRIVERS_PLL_DRIVER_H_

#include "pulsar_board.h" // board configuration file
#include "lmx2571_registers.h" // definition of all PLL registers
#include "fsl_dspi_freertos.h"
#include <stdint.h>		// for uint16_t, uint32_t
#include <string.h>		// for memset

/*
 * Useful defines
 */

#define PLL_REG_BYTESIZE 3 		// size of LMX2571 registers

/*
 * Enumerations
 */
enum workerCommand {
	cmdNONE 	= 0,	// do nothing
	cmdCHECK 	= 1,	// check if device is locked
	cmdPROGRAM 	= 2,	// program default register values
	cmdREAD 	= 3,	// read specified register's value
	cmdWRITE 	= 4,	// write to specified register
};

/*
 * Useful structures
 */

/*
 * Functions
 */

/**
 * Initializes SPI peripheral to communicate with LMX2571
 * @param pllRtosHandle Pointer to RTOS-based SPI device object that will store the config
 * @param srcClock_Hz 	Frequency of clock (Hz) that powers the SPI peripheral
 * @return 				0 or kStatus_Success on success
 */
status_t pll_communication_init(dspi_rtos_handle_t *pllRtosHandle, uint32_t srcClock_Hz);

/**
 * De-initializes SPI peripheral communicating with LMX2571
 * Use this function for cleanup or on failure
 * @param pllRtosHandle	Pointer to RTOS-SPI device containing the config
 * @return				0 or kStatus_Success on success
 */
status_t pll_communication_deinit(dspi_rtos_handle_t *pllRtosHandle);

/**
 * Write data to a register
 * @param pllRtosHandle	Pointer to RTOS-SPI device containing the config
 * @param addr			Address of the PLL register to write to
 * @param data			16-bits of data to write to PLL register
 * @return				0 or kStatus_Success on success
 */
status_t pll_register_write(dspi_rtos_handle_t *pllRtosHandle, unsigned int addr, uint16_t data);

/**
 * Read data from a register
 * @param pllRtosHandle	Pointer to RTOS-SPI device containing the config
 * @param addr			Address of the PLL register to read from
 * @return				Value of data in register
 */
uint16_t pll_register_read(dspi_rtos_handle_t *pllRtosHandle, unsigned int addr);

#endif /* DRIVERS_PLL_DRIVER_H_ */
