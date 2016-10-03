/*
 * csac_driver.c
 *
 * A simple driver for interfacing with the CSAC
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

// TODO: add more complex function here later

#include "csac_driver.h"

static uint32_t old_uart_irq_prio = 0;
static uart_handle_t csacUartHandle;

static int UART_RTOS_Receive_wTimeout(uart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received, TickType_t timeout);

status_t csac_communication_init(uart_rtos_handle_t *csacRtosHandle, uint32_t srcClock_Hz, uint8_t *buffer, uint32_t buffer_size) {
	status_t status;
	struct rtos_uart_config uart_config = {
			.base = BOARD_CSAC_UART,
			.srcclk = srcClock_Hz,
			.baudrate = 57600,
			.parity = kUART_ParityDisabled,
			.stopbits = kUART_OneStopBit,
			.buffer = buffer,
			.buffer_size = buffer_size,
	};

	old_uart_irq_prio = NVIC_GetPriority(BOARD_UART1_IRQn);
	NVIC_SetPriority(BOARD_UART1_IRQn, 5);

	status = UART_RTOS_Init(csacRtosHandle, &csacUartHandle, &uart_config);
	if(status != kStatus_Success) {
		csac_communication_deinit(csacRtosHandle);
	}
	return status;
}


status_t csac_communication_deinit(uart_rtos_handle_t *csacRtosHandle) {
	// restore old NVIC priority
	NVIC_SetPriority(BOARD_UART1_IRQn, old_uart_irq_prio);
	old_uart_irq_prio = 0;
	UART_RTOS_Deinit(csacRtosHandle);
	return kStatus_Success;
}

// This is just a simple redefinition of the existing RTOS function
status_t csac_send(uart_rtos_handle_t *csacRtosHandle, const uint8_t *buffer, uint32_t length) {
	return UART_RTOS_Send(csacRtosHandle, buffer, length);
}

// This is just a simple redefinition of the existing RTOS function
status_t csac_receive(uart_rtos_handle_t *csacRtosHandle, uint8_t *buffer, uint32_t length, size_t *received) {
	return UART_RTOS_Receive(csacRtosHandle, buffer, length, received);
}

// UART RTOS Receive modified for timeout implementation
status_t csac_receive_timeout(uart_rtos_handle_t *csacRtosHandle, uint8_t *buffer, uint32_t length, size_t *received, TickType_t timeout) {
	return UART_RTOS_Receive_wTimeout(csacRtosHandle, buffer, length, received, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : UART_RTOS_Recv_wTimeout
 * Description   : Receives chars for the application. Terminates when timeout period is reached
 *
 *END**************************************************************************/
static int UART_RTOS_Receive_wTimeout(uart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received, TickType_t timeout)
{
    EventBits_t ev;
    size_t n = 0;
    int retval = kStatus_Success;
    TickType_t timestart, timesem;

    timestart = xTaskGetTickCount();	// function start time
    if (NULL == handle->base)
    {
        /* Invalid handle. */
        return kStatus_Fail;
    }
    if (0 == length)
    {
        if (received != NULL)
        {
            *received = n;
        }
        return 0;
    }
    if (NULL == buffer)
    {
        return kStatus_InvalidArgument;
    }

    if (pdFALSE == xSemaphoreTake(handle->rx_sem, timeout))
    {
        /* We could not take the semaphore, exit with 0 data received */
        return kStatus_Fail;
    }

    timesem = xTaskGetTickCount();	// time we acquired the semaphore
    if(timeout < (timesem - timestart)) {
    	return kStatus_Fail;
    }

    handle->rx_xfer.data = buffer;
    handle->rx_xfer.dataSize = (uint32_t)length;

    xEventGroupClearBits(handle->rx_event, RTOS_UART_COMPLETE);

    /* Non-blocking call */
    UART_TransferReceiveNonBlocking(handle->base, handle->t_state, &handle->rx_xfer, &n);

    if (n < length)
    {
        ev = xEventGroupWaitBits(handle->rx_event, RTOS_UART_COMPLETE, pdTRUE, pdFALSE, timeout - (timesem-timestart));
        if (ev & RTOS_UART_COMPLETE)
        {
            n = length;
        }
        else
        {
            retval = kStatus_Fail;
        }
    }

    if (pdFALSE == xSemaphoreGive(handle->rx_sem))
    {
        /* We could not post the semaphore, exit with error */
        retval = kStatus_Fail;
    }

    if (received != NULL)
    {
        *received = n;
    }

    return retval;
}
