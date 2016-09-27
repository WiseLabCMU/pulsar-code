/*
 * csac_driver.h
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

// assuming access to corresponding board.h file

#ifndef BOARD_CSAC_DRIVER_H_
#define BOARD_CSAC_DRIVER_H_

#include "pulsar_board.h"
#include "fsl_uart_freertos.h"
#include <stdint.h>

/**
 * @brief Initializes the CSAC UART interface
 *
 * @param csacRtosHandle The RTOS UART handle, the pointer to allocated space for RTOS context.
 * @param srcClock_Hz
 * @param buffer
 * @param buffer_size
 * @return
 */
status_t csac_communication_init(uart_rtos_handle_t *csacRtosHandle, uint32_t srcClock_Hz, uint8_t *buffer, uint32_t buffer_size);

/**
 * @brief Deinitializes the CSAC UART interface
 *
 * This function deinitializes the UART module, sets all register values to reset value,
 * and releases the resources.
 *
 * @param handle The RTOS UART handle.
 * @return
 */
status_t csac_communication_deinit(uart_rtos_handle_t *csacRtosHandle);


/**
 * @brief Sends data to the CSAC in the background.
 *
 * This function sends data. It is a synchronous API.
 * If the hardware buffer is full, the task is in the blocked state.
 *
 * @param handle The RTOS UART handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 * @return
 */
status_t csac_send(uart_rtos_handle_t *csacRtosHandle, const uint8_t *buffer, uint32_t length);

/**
 * @brief Receives data from CSAC
 *
 * This function receives data from UART. It is a synchronous API. If data is immediately available,
 * it is returned immediately and the number of bytes received.
 *
 * @param handle The CSAC RTOS UART handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to a variable of size_t where the number of received data is filled.
 * @return
 */
status_t csac_receive(uart_rtos_handle_t *csacRtosHandle, uint8_t *buffer, uint32_t length, size_t *received);

#endif /* BOARD_CSAC_DRIVER_H_ */
