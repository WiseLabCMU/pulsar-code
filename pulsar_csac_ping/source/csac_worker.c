/*
 * csac_worker.c
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"
#include "application_threads.h"
#include "fsl_debug_console.h"

/**
 * Drivers
 */
#include "fsl_uart_freertos.h"

enum CsacCommands {
	cmdNONE = 0,
	cmdGET_STATE = 1,
};

static const char csacStatusRequest[4] = "!^\r\n";

// TODO: figure out how FreeRTOS queues work
/**
 * CSAC UART worker thread is responsible for all communication with the CSAC
 * There should only be 1 instance of this thread
 * Requests will be passed to it over a queue
 * @param pvParameters
 */
void csac_worker_thread(void *pvParameters) {

	size_t nRecv = 0;
	char first = true;
	int csacState = -1;
	uint32_t runCount = 0;	// debug variable to keep track of worker calls

	// these are temp variables to solve compile errors
	uint8_t csacUartBuffer[2];
	uint8_t csacRecvBuffer[2];
	SemaphoreHandle_t gCsacWorkerMutex = NULL;
	uart_rtos_handle_t csacUartRtosHandle;
	uart_handle_t csacUartHandle;
	enum CsacCommands csacCommand;


	// UART configuration for CSAC communication
	struct rtos_uart_config csacUartConfig = {
		.base = BOARD_CSAC_UART,
		.srcclk = CLOCK_GetFreq(SYS_CLK),
	    .baudrate = 57600,
	    .parity = kUART_ParityDisabled,
	    .stopbits = kUART_OneStopBit,
	    .buffer = csacUartBuffer,
	    .buffer_size = sizeof(csacUartBuffer),
	};



	/*
	 * If the CSAC UART worker mutex is empty, the device has not been setup
	 * by any other thread. This thread is then responsible for initializing
	 * the peripheral
	 */

	if(gCsacWorkerMutex == NULL) {
		// No mutex detected

		// Create mutex
		gCsacWorkerMutex = xSemaphoreCreateMutex();
		if(gCsacWorkerMutex == NULL) {
			// Mutex creation failed. Warn and stop execution
			PRINTF("[%08u] mutex create fail. suspending csacWorker\r\n", xTaskGetTickCount());
			goto fail;
		} else {
			// Mutex creation was successful
			// Try to initialize UART device
			if(UART_RTOS_Init(&csacUartRtosHandle, &csacUartHandle, &csacUartConfig) != 0) {
				// UART initialization failed. Warn and suspend task
				PRINTF("[%08u] unable to init UART. suspending csacWorker\r\n", xTaskGetTickCount());
				goto fail;
			} else {	// UART successfully initialized
				xSemaphoreGive(gCsacWorkerMutex);
				PRINTF("[%08u] UARt initialized\r\n", xTaskGetTickCount());
			}
		}
	}

	// make sure commands are only accepted after setup is complete
	// maybe get rid of this later
	csacCommand = cmdNONE;

	// infinite service loop for thread
	while(true) {

		runCount++;
		switch(csacCommand) {

			// get state of CSAC
			case cmdGET_STATE:
				csacState = -1;		// reset status variable

				// lock CSAC device access for UART operations
				// TODO: UART timeout period is currently randomly set to 50 ticks
				if(xSemaphoreTake(gCsacWorkerMutex, 50) == pdTRUE) {

					// send status request
					UART_RTOS_Send(&csacUartRtosHandle, (uint8_t *)csacStatusRequest, sizeof(csacStatusRequest) -1);

					// wait for response
					while(true) {
						nRecv = 0;
						UART_RTOS_Receive(&csacUartRtosHandle, csacRecvBuffer, 1, &nRecv);

						// get state from the first character of response
						if(first == true) {
							csacState = (int)csacRecvBuffer[0] - (int)'0';
							first = false;
						}

						// flush buffer
						// end of transmission is indicated by \r\n sent by the CSAC
						if(csacRecvBuffer[0] == (uint8_t) '\n') {
							break;
						}

						// the detected state was invalid
						// return error code for invalid output
						// TODO: use proper k return error code
						if(csacState < 0 || csacState > 8) {
							csacState = -1;
						}
					}
					// UART ops complete. unlock CSAC device access over UART
					xSemaphoreGive(gCsacWorkerMutex);
				}
				break;
			case cmdNONE:
			default:
				break;
		}

		// Worker task will suspend itself by default at the end of operation
		// Task will delay itself instead during the testing phase.
		// Get rid of this when developing actual worker thread
//		vTaskDelay(2000);
		PRINTF("[%08u] csac_worker_thread run count: %u\r\n", xTaskGetTickCount(), runCount);
		vTaskSuspend(NULL);

	}
	fail:
		vSemaphoreDelete(gCsacWorkerMutex);
		vTaskSuspend(NULL);
}

