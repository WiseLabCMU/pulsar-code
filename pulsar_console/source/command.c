/*
 * commander.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include <stdio.h>

#include "pulsar_board.h"

#include "fsl_debug_console.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * SHARED OBJECTS section
 *****************************************************************************/

uart_rtos_handle_t console_handle;
struct _uart_handle console_t_handle;

/******************************************************************************
 * end of SHARED OBJECTS section
 *****************************************************************************/


/******************************************************************************
 * COMMAND TASK section
 *****************************************************************************/

#define CONSOLE_RING_BUF_SIZE 32		// UART ring buffer of size 32. This is used for the background receive buffer.
#define CONSOLE_RX_BUF_SIZE 32			// RX buffer size of 32
#define CONSOLE_TX_BUF_SIZE 32			// TX buffer size of 32

uint8_t console_ring_buffer[CONSOLE_RING_BUF_SIZE];
uint8_t console_rx_buffer[CONSOLE_RX_BUF_SIZE];
uint8_t console_tx_buffer[CONSOLE_TX_BUF_SIZE];

// Constant strings. These hopefully stay in flash memory.
static const char hello_string[15] = "Hello, world!\r\n";
static const char invalid_cmd[13] = "invalid cmd\r\n";
static const char tof_on[19] = "ToF correction ON\r\n";
static const char tof_off[20] = "ToF correction OFF\r\n";

static struct rtos_uart_config console_uart_config = {
	.base = UART0,
    .baudrate = 115200,
    .parity = kUART_ParityDisabled,
    .stopbits = kUART_OneStopBit,
    .buffer = console_ring_buffer,
    .buffer_size = sizeof(console_ring_buffer),
};

/**
 * This function parses the contents from the required buffer and acts on them.
 * Note that the buffer is shared as a global variable.
 * @param n_bytes	number of bytes in command string
 * @return
 */
static int parse_and_act(size_t n_bytes) {
	int value = 0;
	size_t written_size = 0;
	switch((char) console_rx_buffer[0]) {
		case 'h':
			// trigger single heartbeat
			if(strncmp((char *) &console_rx_buffer[1], "\r\n", 2) == 0) {
				xSemaphoreGive(semaphore[HB_SEM]);
				break;
			} else {
				goto invalid_command;
			}
		case 'o':
			// set PPS offset in nsec
			// NOTE: This correction is relative

			if( sscanf((char *) &console_rx_buffer[1], " %d\r\n", &value) == 1) {
				// TODO: implement exact mechanics for data transfer
				written_size = snprintf((char *) console_tx_buffer, CONSOLE_TX_BUF_SIZE, "off=%d\r\n", value);
				UART_RTOS_Send(&console_handle, (uint8_t *) console_tx_buffer, written_size);
				break;
			} else {
				goto invalid_command;
			}

		case 'T':
			// turn on ToF corrections
			if(strncmp((char *) &console_rx_buffer[1], "\r\n", 2) == 0) {
				// TODO: implement exact mechanics for turning on tof corrections
				UART_RTOS_Send(&console_handle, (uint8_t *) tof_on, sizeof(tof_on));
				break;
			} else {
				goto invalid_command;
			}

		case 't':
			// turn off ToF corrections
			if(strncmp((char *) &console_rx_buffer[1], "\r\n", 2) == 0) {
				// TODO: implement exact mechanics for turning off tof corrections
				UART_RTOS_Send(&console_handle, (uint8_t *) tof_off, sizeof(tof_off));
				break;
			} else {
				goto invalid_command;
			}

		default:

		invalid_command:
			UART_RTOS_Send(&console_handle, (uint8_t *) invalid_cmd, sizeof(invalid_cmd));
		}

	return kStatus_Success;
}

/*!
 * @brief Command task. Handles inputs to/from user over UART console.
 */
void command_task(void *pvParameters) {

	size_t n = 0;
	size_t index = 0;

//	PRINTF("hello from command task\r\n");	// just to show that everything works

	console_uart_config.srcclk = CLOCK_GetFreq(SYS_CLK);
	// initialize UART console
	if(UART_RTOS_Init(&console_handle, &console_t_handle, &console_uart_config) < 0) {
		PRINTF("UART init error\r\n");
		vTaskSuspend(NULL);
	}

	if(UART_RTOS_Send(&console_handle, (uint8_t *) hello_string, 15) < 0) {
		PRINTF("UART send error\r\n");
		vTaskSuspend(NULL);
	}
	while(1) {
		// get commands from console (and echo them)
		n = 0;
		if(UART_RTOS_Receive(&console_handle, &console_rx_buffer[index], 1, &n) == kStatus_Success) {
			if(n > 0) {
				// receive was successful. echo it out

				UART_RTOS_Send(&console_handle, (uint8_t *) &console_rx_buffer[index], 1);
				if(console_rx_buffer[index] == (uint8_t) '\n') {
					// indicates we have a complete command
					// work on string operations here
					parse_and_act(index);

					// reset index to 0
					index = 0;
				} else {
					index = (index + 1) % CONSOLE_RX_BUF_SIZE;
				}
			}
		}
//		PRINTF("-\r\n");
//		vTaskDelay(10);
	}
	UART_RTOS_Deinit(&console_handle);
	vTaskSuspend(NULL);
}

/******************************************************************************
 * end of COMMAND TASK section
 *****************************************************************************/
