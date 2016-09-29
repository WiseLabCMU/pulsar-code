/*
 * deca_spi.c
 *
 *  Created on: Sep 25, 2016
 *      Author: adwait
 */


/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave. Modified by Adwait Dongare for use with Pulsar.
 *
 */
#include <string.h>

#include "pulsar_board.h"
#include "deca_spi.h"
#include "deca_device_api.h"

//static uint32_t old_dw_spi_irq_prio = 0;

static uint8_t spi_transfer_byte(uint8_t txData, dspi_command_data_config_t *spi_command);

// TODO: convert to use RTOS functions later
status_t dw_communication_slow_init(dspi_rtos_handle_t *dwRtosHandle, uint32_t srcClock_Hz) {

//	status_t status;
	dspi_master_config_t dw_slow_spi_master_config = {
		.whichCtar = kDSPI_Ctar0,
		.ctarConfig.baudRate = BOARD_DW1000_SLOW_BAUDRATE,
		.ctarConfig.bitsPerFrame = 8U,
		.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh,
		.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge,
		.ctarConfig.direction = kDSPI_MsbFirst,
		.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / BOARD_DW1000_SLOW_BAUDRATE,
		.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / BOARD_DW1000_SLOW_BAUDRATE,
		.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / BOARD_DW1000_SLOW_BAUDRATE,
		.whichPcs = BOARD_DW1000_SPI_PCS,
		.pcsActiveHighOrLow = kDSPI_PcsActiveLow,
		.enableContinuousSCK = false,
		.enableRxFifoOverWrite = false,
		.enableModifiedTimingFormat = false,
		.samplePoint = kDSPI_SckToSin0Clock,
	};

//	old_dw_spi_irq_prio = NVIC_GetPriority(BOARD_DW1000_SPI_IRQn);
//	NVIC_SetPriority(BOARD_DW1000_SPI_IRQn, 6);

//	status = DSPI_RTOS_Init(pllRtosHandle, BOARD_DW1000_SPI, &dw_slow_spi_master_config, srcClock_Hz);
//	if(status != kStatus_Success) {
//		dw_communication_deinit(dwRtosHandle);
//	}
//	return status;

	DSPI_MasterInit(BOARD_DW1000_SPI, &dw_slow_spi_master_config, srcClock_Hz);
	return kStatus_Success;
}

status_t dw_communication_fast_init(dspi_rtos_handle_t *dwRtosHandle, uint32_t srcClock_Hz) {

//	status_t status;
	dspi_master_config_t dw_fast_spi_master_config = {
		.whichCtar = kDSPI_Ctar0,
		.ctarConfig.baudRate = BOARD_DW1000_FAST_BAUDRATE,
		.ctarConfig.bitsPerFrame = 8U,
		.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh,
		.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge,
		.ctarConfig.direction = kDSPI_MsbFirst,
		.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / BOARD_DW1000_FAST_BAUDRATE,
		.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / BOARD_DW1000_FAST_BAUDRATE,
		.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / BOARD_DW1000_FAST_BAUDRATE,
		.whichPcs = BOARD_DW1000_SPI_PCS,
		.pcsActiveHighOrLow = kDSPI_PcsActiveLow,
		.enableContinuousSCK = false,
		.enableRxFifoOverWrite = false,
		.enableModifiedTimingFormat = false,
		.samplePoint = kDSPI_SckToSin0Clock,
	};

//	old_dw_spi_irq_prio = NVIC_GetPriority(BOARD_DW_SPI_IRQn);
//	NVIC_SetPriority(BOARD_DW_SPI_IRQn, 6);
//
//	status = DSPI_RTOS_Init(dwRtosHandle, BOARD_DW_SPI, &dw_fast_spi_master_config, srcClock_Hz);
//	if(status != kStatus_Success) {
//		dw_communication_deinit(dwRtosHandle);
//	}
//	return status;

	DSPI_MasterInit(BOARD_DW1000_SPI, &dw_fast_spi_master_config, srcClock_Hz);
	return kStatus_Success;

}

// TODO: adapt for FreeRTOS UART functions later
status_t dw_communication_deinit(dspi_rtos_handle_t *dwRtosHandle) {

//	// TODO: add wait until SPI device has finished transmission
//	NVIC_SetPriority(BOARD_PLL_SPI_IRQn, old_dw_spi_irq_prio);
//	old_dw_spi_irq_prio = 0;
//	return DSPI_RTOS_Deinit(dwRtosHandle);

	DSPI_Deinit(BOARD_DW1000_SPI);
	return kStatus_Success;

}

// TODO: clean this up for FreeRTOS use
// TODO: make sure the function is still thread-safe

/**
 *
 * @param headerLength
 * @param headerBuffer
 * @param bodylength
 * @param bodyBuffer
 * @return
 */
#pragma GCC optimize ("O3")
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {

	uint16_t hCount;
	uint32_t bCount;
	static dspi_command_data_config_t dw_spi_command;

	dw_spi_command.whichPcs = kDSPI_Pcs0;
	dw_spi_command.isEndOfQueue = false;
	dw_spi_command.clearTransferCount = false;
	dw_spi_command.whichCtar = kDSPI_Ctar0;
	dw_spi_command.isPcsContinuous = true;

	DSPI_FlushFifo(BOARD_DW1000_SPI, true, true);
	DSPI_ClearStatusFlags(BOARD_DW1000_SPI, kDSPI_AllStatusFlag);

	DSPI_StartTransfer(BOARD_DW1000_SPI);

	// loop to write header
	for(hCount = 0; hCount < headerLength; hCount++) {
		spi_transfer_byte((uint8_t) headerBuffer[hCount], &dw_spi_command);
	}
	// loop to read data
	for(bCount = 0; bCount < bodylength; bCount++) {
		if(bCount == bodylength -1) {
			dw_spi_command.isEndOfQueue = true;
			dw_spi_command.clearTransferCount = true;
			dw_spi_command.isPcsContinuous = false;
		}
		spi_transfer_byte((uint8_t) bodyBuffer[bCount], &dw_spi_command);
	}

	DSPI_StopTransfer(BOARD_DW1000_SPI);

	return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {

	uint16_t hCount;
		uint32_t bCount;
		static dspi_command_data_config_t dw_spi_command;

		dw_spi_command.whichPcs = kDSPI_Pcs0;
		dw_spi_command.isEndOfQueue = false;
		dw_spi_command.clearTransferCount = false;
		dw_spi_command.whichCtar = kDSPI_Ctar0;
		dw_spi_command.isPcsContinuous = true;

		DSPI_FlushFifo(BOARD_DW1000_SPI, true, true);
		DSPI_ClearStatusFlags(BOARD_DW1000_SPI, kDSPI_AllStatusFlag);

		DSPI_StartTransfer(BOARD_DW1000_SPI);

		// loop to write header
		for(hCount = 0; hCount < headerLength; hCount++) {
			spi_transfer_byte((uint8_t) headerBuffer[hCount], &dw_spi_command);
		}
		// loop to read data
		for(bCount = 0; bCount < readlength; bCount++) {
			if(bCount == readlength -1) {
				dw_spi_command.isEndOfQueue = true;
				dw_spi_command.clearTransferCount = true;
				dw_spi_command.isPcsContinuous = false;
			}
			readBuffer[bCount] = (uint8) spi_transfer_byte(0, &dw_spi_command);
		}

		DSPI_StopTransfer(BOARD_DW1000_SPI);

		return 0;
} // end readfromspi()

// TODO: implement more robust mutex functions using FreeRTOS APIs
decaIrqStatus_t decamutexon (void) {
	PORT_SetPinInterruptConfig(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_IRQ_PIN, kPORT_InterruptOrDMADisabled);
	return PORTC_IRQn;
}

// TODO: implement more robust mutex functions using FreeRTOS APIs
void decamutexoff (decaIrqStatus_t state) {
	PORT_SetPinInterruptConfig(BOARD_DW1000_GPIO_PORT, BOARD_DW1000_GPIO_IRQ_PIN, kPORT_InterruptRisingEdge);
}


/**
 * Local synchronous function to exchange 1 byte over SPI
 *
 * @param txData
 * @param spi_command
 * @return
 */
#pragma GCC optimize ("O3")
static uint8_t spi_transfer_byte(uint8_t txData, dspi_command_data_config_t *spi_command) {
	uint8_t rxData;

	while (!(DSPI_GetStatusFlags(BOARD_DW1000_SPI) & kDSPI_TxFifoFillRequestFlag)) {
		DSPI_ClearStatusFlags(BOARD_DW1000_SPI , kDSPI_TxFifoFillRequestFlag);
	}

	// send TX data
	DSPI_MasterWriteData(BOARD_DW1000_SPI , spi_command, (uint16_t)txData);
	DSPI_ClearStatusFlags(BOARD_DW1000_SPI , kDSPI_TxFifoFillRequestFlag);

	// wait for TX done
	while (!(DSPI_GetStatusFlags(BOARD_DW1000_SPI) & kDSPI_TxCompleteFlag));

	// get data from RX
	while (!(DSPI_GetStatusFlags(BOARD_DW1000_SPI) & kDSPI_RxFifoDrainRequestFlag));
	rxData = DSPI_ReadData(BOARD_DW1000_SPI);
	DSPI_ClearStatusFlags(BOARD_DW1000_SPI, kDSPI_RxFifoDrainRequestFlag);

	return rxData;
}
