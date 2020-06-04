/*
 * pll_driver.c
 *
 *	None of the functions in this thread are thread-safe.
 *	It is the responsibility of the calling thread to protect access
 *
 *  Created on: Sep 10, 2016
 *      Author: Adwait Dongare
 */

#include "pll_driver.h"



/**
 * Variables only necessary for this driver
 */
static uint32_t old_pll_spi_irq_prio = 0;

status_t pll_communication_init(dspi_rtos_handle_t *pllRtosHandle, uint32_t srcClock_Hz) {

	status_t status;
	dspi_master_config_t pll_spi_master_config = {
		.whichCtar = kDSPI_Ctar0,
		.ctarConfig.baudRate = BOARD_PLL_BAUDRATE,
		.ctarConfig.bitsPerFrame = 8U,
		.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh,
		.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge,
		.ctarConfig.direction = kDSPI_MsbFirst,
		.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / BOARD_PLL_BAUDRATE,
		.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / BOARD_PLL_BAUDRATE,
		.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / BOARD_PLL_BAUDRATE,
		.whichPcs = BOARD_PLL_SPI_PCS,
		.pcsActiveHighOrLow = kDSPI_PcsActiveLow,
		.enableContinuousSCK = false,
		.enableRxFifoOverWrite = false,
		.enableModifiedTimingFormat = false,
		.samplePoint = kDSPI_SckToSin0Clock,
	};

	old_pll_spi_irq_prio = NVIC_GetPriority(BOARD_PLL_SPI_IRQn);
	NVIC_SetPriority(BOARD_PLL_SPI_IRQn, 6);

	status = DSPI_RTOS_Init(pllRtosHandle, BOARD_PLL_SPI, &pll_spi_master_config, srcClock_Hz);
	if(status != kStatus_Success) {
		pll_communication_deinit(pllRtosHandle);
	}
	return status;
}

// TODO: deinit must clean a PLL protector mutex later
status_t pll_communication_deinit(dspi_rtos_handle_t *pllRtosHandle) {

	NVIC_SetPriority(BOARD_PLL_SPI_IRQn, old_pll_spi_irq_prio);
	old_pll_spi_irq_prio = 0;
	return DSPI_RTOS_Deinit(pllRtosHandle);
}

status_t pll_register_write(dspi_rtos_handle_t *pllRtosHandle, unsigned int addr, uint16_t data) {
	union lmx2571_register reg;
	dspi_transfer_t spiData;
	status_t status;
    // buffers are volatile as some other thread/function may modify them
    uint8_t txBuf[PLL_REG_BYTESIZE];
    uint8_t rxBuf[PLL_REG_BYTESIZE];

    // use the union structure to parse register values
    reg.datamap.data = data;
	reg.datamap.ADDRESS = addr;
	reg.datamap.RW = CMD_WRITE;

	// arrange bytes in the correct order
	// this happens since ARM is a little endian system and inverts the actual
	// byte structure in memory
	// NOTE: if there is a send last byte first protocol, we can explore it
	txBuf[0] = reg.bytes[2];
	txBuf[1] = reg.bytes[1];
	txBuf[2] = reg.bytes[0];

	memset((void *) rxBuf, 0, PLL_REG_BYTESIZE);

    // Note: Though this looks like a waste of space, it is irrelevant for
    // small 3 byte transfers involving PLL registers
    spiData.txData = txBuf;
    spiData.rxData = rxBuf;
    spiData.dataSize = PLL_REG_BYTESIZE;
    spiData.configFlags = BOARD_PLL_SPI_MASTER_CTAR | BOARD_PLL_SPI_MASTER_PCS | BOARD_PLL_SPI_MASTER_PCS_MODE;

    // TODO: acquire lock here
    status = DSPI_RTOS_Transfer(pllRtosHandle, &spiData);
    // TODO: release lock here

    return status;
}

uint16_t pll_register_read(dspi_rtos_handle_t *pllRtosHandle, unsigned int addr) {
	union lmx2571_register reg;
	status_t status;
	dspi_transfer_t spiData;
	// buffers are volatile as some other thread/function may modify them
	uint8_t txBuf[PLL_REG_BYTESIZE];
	uint8_t rxBuf[PLL_REG_BYTESIZE];

    // use the union structure to parse register values
	reg.datamap.data = 0U;
	reg.datamap.ADDRESS = addr;
	reg.datamap.RW = CMD_READ;
	txBuf[0] = reg.bytes[2];

	// set all remaining bits in buffer to 0
	memset((void *) &txBuf[1], 0, PLL_REG_BYTESIZE-1);
	// clean the receive buffer
	memset((void *) rxBuf, 0, PLL_REG_BYTESIZE);

	spiData.txData = txBuf;
	spiData.rxData = rxBuf;
	spiData.dataSize = PLL_REG_BYTESIZE;
	spiData.configFlags = BOARD_PLL_SPI_MASTER_CTAR | BOARD_PLL_SPI_MASTER_PCS | BOARD_PLL_SPI_MASTER_PCS_MODE;

	// TODO: acquire lock
	status = DSPI_RTOS_Transfer(pllRtosHandle, &spiData);
	// TODO: release lock
	if(status == kStatus_Success) {
		reg.bytes[1] = rxBuf[1];
		reg.bytes[2] = rxBuf[0];
	}

	return reg.datamap.data;	// temporary return value. replace this with actual code
}

