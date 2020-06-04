/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __FSL_LPUART_RTOS_H__
#define __FSL_LPUART_RTOS_H__

#include "fsl_lpuart.h"
#include <os.h>

/*!
 * @addtogroup lpuart_ucosiii_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief LPUART µCOS-III driver version 2.0.0. */
#define FSL_LPUART_UCOSIII_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

struct rtos_lpuart_config
{
    LPUART_Type *base;                /*!< UART base address */
    uint32_t srcclk;                  /*!< UART source clock in Hz*/
    uint32_t baudrate;                /*!< Desired communication speed */
    lpuart_parity_mode_t parity;      /*!< Parity setting */
    lpuart_stop_bit_count_t stopbits; /*!< Number of stop bits to use */
    uint8_t *buffer;                  /*!< Buffer for background reception */
    uint32_t buffer_size;             /*!< Size of buffer for background reception */
};

typedef struct _lpuart_rtos_handle
{
    LPUART_Type *base;               /*!< UART base address */
    struct _lpuart_transfer tx_xfer; /*!< TX transfer structure */
    struct _lpuart_transfer rx_xfer; /*!< RX transfer structure */
    OS_SEM rx_sem;                   /*!< RX semaphore for resource sharing */
    OS_SEM tx_sem;                   /*!< TX semaphore for resource sharing */
#define RTOS_LPUART_COMPLETE 0x1
    OS_FLAG_GRP rx_event; /*!< RX completion event */
    OS_FLAG_GRP tx_event; /*!< TX completion event */
    void *t_state;        /*!< Transactional state of the underlying driver */
} lpuart_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPUART RTOS Operation
 * @{
 */

/*!
 * @brief Initializes an LPUART instance for operation in RTOS.
 *
 * @param handle The RTOS LPUART handle, the pointer to allocated space for RTOS context.
 * @param lpuart_t_handle The pointer to allocated space where to store transactional layer internal state.
 * @param cfg The pointer to the parameters required to configure the LPUART after initialization.
 * @return 0 succeed, others failed
 */
int LPUART_RTOS_Init(lpuart_rtos_handle_t *handle, lpuart_handle_t *t_handle, const struct rtos_lpuart_config *cfg);

/*!
 * @brief Deinitializes an LPUART instance for operation.
 *
 * This function deinitializes the LPUART modulem, set all register value to reset value
 * and releases the resources.
 *
 * @param handle The RTOS LPUART handle.
 */
int LPUART_RTOS_Deinit(lpuart_rtos_handle_t *handle);

/*!
 * @name LPUART transactional Operation
 * @{
 */

/*!
 * @brief Send data in background.
 *
 * This function sends data. It is synchronous API.
 * If the HW buffer is full, the task is in the blocked state.
 *
 * @param handle The RTOS LPUART handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 */
int LPUART_RTOS_Send(lpuart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data. It is synchronous API.
 *
 * This function receives data from LPUART. If any data is immediately available
 * it will be returned imidiately and the number of bytes received.
 *
 * @param handle The RTOS LPUART handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to variable of size_t where the number of received data will be filled.
 */
int LPUART_RTOS_Receive(lpuart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPUART_RTOS_H__ */
