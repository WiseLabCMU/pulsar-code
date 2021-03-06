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

/******************************************************************************
 * UART SETTINGS
 *****************************************************************************/

#define CSAC_LOCKED 0


#define CSAC_TX_BUFFER_SIZE 16	//
#define CSAC_RX_BUFFER_SIZE 64	// size of UART receive ring buffer.
								// note that we only get N-1 chars to work with

// buffers for CSAC communications
static uint8_t csacTxBuffer[CSAC_TX_BUFFER_SIZE];	// buffer for constructing transmit messages
static uint8_t csacRxBuffer[CSAC_RX_BUFFER_SIZE];	// buffer for receive messages


/*
 * Set of CSAC commands
 * TODO: Q. should this be a constant or be reconstructed
 */

static const uint8_t csacStatusRequest[1] = "^";
static const uint8_t csacDisableDiscipline[2] = "Md";
static const uint8_t csacEnableDiscipline[2] = "MD";

// TODO: add function descriptions here
static int UART_RTOS_Receive_wTimeout(uart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received, TickType_t timeout);
static int UART_RTOS_Clear_Pending(uart_rtos_handle_t *handle);

/******************************************************************************
 * BASIC CSAC COMMUNICATION OPERATIONS
 *****************************************************************************/

status_t csac_communication_init(uart_rtos_handle_t *csacRtosHandle, uint32_t srcClock_Hz, uint8_t *buffer, uint32_t buffer_size) {
	status_t status;
	struct rtos_uart_config uart_config = {
			.base = BOARD_CSAC_UART,
			.srcclk = srcClock_Hz,
			.baudrate = 57600,
			.parity = kUART_ParityDisabled,
			.stopbits = kUART_OneStopBit,
			.buffer = buffer,				// ring buffer
			.buffer_size = buffer_size,		// ring buffer size
	};

	// save old IRQ priority so we can restore it later
	old_uart_irq_prio = NVIC_GetPriority(BOARD_UART1_IRQn);
	NVIC_SetPriority(BOARD_UART1_IRQn, 5);

	status = UART_RTOS_Init(csacRtosHandle, &csacUartHandle, &uart_config);
	if(status != kStatus_Success) {
		// restore state back to wat it was before we attempted initialization
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

void csac_flush_rx_buffer(uart_rtos_handle_t *csacRtosHandle) {
	uint8_t tmpBuf;
	int nReceived = 0;
	// flush UART buffers
	// TODO: there should be a better way of handling this
	while(true) {
		nReceived = 0;
		if(csac_receive_timeout(csacRtosHandle, &tmpBuf, 1, &nReceived, 0) != kStatus_Success) {
			break;
		}
	}
}

/******************************************************************************
 * end of BASIC CSAC COMMUNICATION OPERATIONS
 *****************************************************************************/

/******************************************************************************
 * ADDITIONAL RTOS FUNCTIONS
 *****************************************************************************/

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

/**
 * Clears all remaining contents from
 * @param handle
 * @return
 */
static int UART_RTOS_Clear_Pending(uart_rtos_handle_t *handle) {
	if (NULL == handle->base) {
		/* Invalid handle. */
		return kStatus_Fail;
	}

	if (pdFALSE == xSemaphoreTake(handle->rx_sem, 0)) {
		/* We could not take the semaphore, exit with 0 data received */
		return kStatus_Fail;
	}
	// we have the UART semaphore
	// TODO: abort any active transfers
	UART_TransferAbortReceive(handle->base, &csacUartHandle);

	xEventGroupClearBits(handle->rx_event, RTOS_UART_COMPLETE);
	memset(handle->rx_xfer.data, 0, handle->rx_xfer.dataSize);

	// give back UART semaphore
	if (pdFALSE == xSemaphoreGive(handle->rx_sem)) {
		/* We could not post the semaphore, exit with error */
		retval = kStatus_Fail;
	}
	return kStatus_Success;
}

/******************************************************************************
 * end of ADDITIONAL RTOS FUNCTIONS
 *****************************************************************************/

/******************************************************************************
 * CSAC OPERATIONS section
 *****************************************************************************/



/**
 * Unified function for all CSAC UART commands
 * @param rtos_uart_handle
 * @param cmd
 * @return
 */
int csac_command(uart_rtos_handle_t *rtos_uart_handle, enum csac_command cmd, uint8_t* data) {
	int ret_val;
	size_t nRecv;

	// TODO: flush RX buffer and it's contents here
	csac_flush_rx_buffer();

	switch(cmd) {
		case cmd_getStatus:
			ret_val = -1;	// set default return value to 0
			nRecv = 0;		// set number of characters received to 0
			// send status command
			csac_send(rtos_uart_handle, "!" csacStatusRequest "\r\n",4);
			// wait for response
			// only grab the first character in the response. we don't really care about the rest
			if(csac_receive_timeout(rtos_uart_handle, csacRxBuffer, 64, &nRecv, (TickType_t) 50) == kStatus_Success) {
				ret_val = (int)csacRxBuffer[0] - (int)'0';
			} else {
				// didn't receive response
				// handle error here?
			}

			break;
		case cmd_NONE:
		default:
	}
}

int csac_get_state(uart_rtos_handle_t *rtos_uart_handle) {
	int csac_state = -1;
	int nRecv = 0;

	csac_flush_rx_buffer(rtos_uart_handle);

	// send telemetry request
	csac_send(rtos_uart_handle, csacStatusRequest, 4);


	// wait for response
	// only grab the first character in the response. we don't really care about the rest
	if(csac_receive_timeout(rtos_uart_handle, csacRxBuffer, 1, &nRecv, (TickType_t) 20) == kStatus_Success) {
				csac_state = (int)csacRxBuffer[0] - (int)'0';
	} else {
		// didn't receive response
		// handle error here?
	}

// unaltered commented version in case we need to recover
//	// listen to UART and parse response
//	while(totalBytes < 128) {	// the 128 character limit is to make sure string runoffs dont mess up communications
//		nRecv = 0;
//
//		if(csac_receive_timeout(rtos_uart_handle, csacRxBuffer, 1, &nRecv, timeout) == kStatus_Success) {
//			if(first == true) {
//				csac_state = (int)csacRxBuffer[0] - (int)'0';
//				timeout = 2;
//				first = false;
//			}
//
//			if(csacRecvBuffer[0] == (uint8_t) '\n') {
//				break;
//			}
//			totalBytes++;
//		} else {
//			break;
//		}
//	}

	return csac_state;
}

long csac_get_freq_steer(uart_rtos_handle_t *rtos_uart_handle) {
	char buf[18];
	long f = 0;
	int nReceived = 0;
	TickType_t timeout = 20;
	int index = 0;

	totalBytes = 0;
	memset((void *) csacRxBuffer, 0, 17);	// clear buffer

	csac_flush_rx_buffer(rtos_uart_handle);

	// send steer query
	csac_send(rtos_uart_handle, csacSteerQuery, 5);

	// listen to UART and parse response
	while(totalBytes < 17) {
//		PRINTF("*");
		nReceived = 0;
		if(csac_receive_timeout(rtos_uart_handle, &csacRxBuffer[index], 1, &nReceived, timeout) == kStatus_Success) {
			index++;
			timeout = 2;
			if(csacRxBuffer[index] == (uint8_t) '\n') {
				break;
			}
		} else {
			// UART receive timed out. No response can be expected.
			PRINTF("[%u] timeout\r\n",xTaskGetTickCount());
			break;
		}
	}
	buf[index+1] = '\0';
	sscanf(buf, "Steer = %ld\r\n", &f);
	return f;

}

void csac_apply_absolute_steer(uart_rtos_handle_t *rtos_uart_handle, long steer_val) {
	int query_size = 0;

	query_size = sprintf((void *) csacTxBuffer,"!FA%ld\r\n", (long) steer_val);
	if(query_size > 0 && query_size < 15) {
		csac_send(rtos_uart_handle, (uint8_t *) csacTxBuffer, query_size);
	}
}

void csac_disable_pps_discipline(uart_rtos_handle_t *rtos_uart_handle) {
//	TickType_t timeout = 10;
	// disable disciplining
	csac_send(rtos_uart_handle, csacDisableDiscipline, sizeof(csacDisableDiscipline));		// this command disables disciplining from PPS

	// as long as we flush buffers before every command, we shouldnt have to worry about the contents in there.
//	while(csac_receive_timeout(&csac_uart_handle, csacRecvBuffer, 1, &nRecv, timeout) == kStatus_Success) {
//		nRecv = 0;
//		if(csacRecvBuffer[0] == (uint8_t) '\n') {
//			break;
//		}
//	}
}

void csac_enable_pps_discipline(uart_rtos_handle_t *rtos_uart_handle) {
	// disable disciplining
	csac_send(rtos_uart_handle, csacEnableDiscipline, sizeof(csacEnableDiscipline));		// this command disables disciplining from PPS

}


/******************************************************************************
 * end of CSAC OPERATIONS section
 *****************************************************************************/
