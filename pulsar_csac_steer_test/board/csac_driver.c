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
