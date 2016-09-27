/*
 * deca_spi.h
 *
 *  Created on: Sep 25, 2016
 *      Author: adwait
 */

#ifndef BOARD_DECA_SPI_H_
#define BOARD_DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "pulsar_board.h"
#include "deca_device_api.h"
#include "fsl_dspi_freertos.h"

#define DECA_MAX_SPI_HEADER_LENGTH      (3)                     // max number of bytes in header (for formating & sizing)

/**
 *
 * @param dwRtosHandle
 * @param srcClock_Hz
 * @return
 */
status_t dw_communication_slow_init(dspi_rtos_handle_t *dwRtosHandle, uint32_t srcClock_Hz);

/**
 *
 * @param dwRtosHandle
 * @param srcClock_Hz
 * @return
 */
status_t dw_communication_fast_init(dspi_rtos_handle_t *dwRtosHandle, uint32_t srcClock_Hz);

/**
 *
 * @param dwRtosHandle
 * @return
 */
status_t dw_communication_deinit(dspi_rtos_handle_t *dwRtosHandle);

/**
 *
 * @param headerLength
 * @param headerBuffer
 * @param bodylength
 * @param bodyBuffer
 * @return
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);

/**
 *
 * @param headerLength
 * @param headerBuffer
 * @param readlength
 * @param readBuffer
 * @return
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);

/**
 *
 * @return
 */
decaIrqStatus_t decamutexon (void);

/**
 *
 * @param state
 */
void decamutexoff (decaIrqStatus_t state);

#ifdef __cplusplus
}
#endif



#endif /* BOARD_DECA_SPI_H_ */
