/*
 * csac_worker.c
 *
 * The CSAC worker thread is responsible for all communication with the CSAC
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"
#include "application_threads.h"
#include "fsl_debug_console.h"
#include "csac_driver.h"

static const char csacStatusRequest[4] = "!^\r\n";

/**
 * The worker manages all communication with the CSAC
 * An application will spawn a new worker when it needs to communicate
 * with the CSAC
 *
 * @param pvParameters
 */

// TODO: setup worker for single command
void csac_worker_thread(void *pvParameters) {

	int csacState;
	size_t nRecv = 0;
	volatile struct csacWorkerCmdStruct *cmdStruct = (struct csacWorkerCmdStruct *) pvParameters;
	uint8_t csacRecvBuffer[2];
	bool first = true;	// flag to indicate if this is first byte in stream

	switch(cmdStruct->cmd) {

		case cmdCSAC_NONE:
			break;
		// get state of CSAC
		case cmdCSAC_GET_STATE:
			csacState = -1;		// reset status variable

			// lock CSAC device access for UART operations
			// TODO: UART timeout period is currently randomly set to 50 ticks
			if(xSemaphoreTake(cmdStruct->csacMutex, 50) == pdTRUE) {

				// send status request
				UART_RTOS_Send(cmdStruct->spiHandle, (uint8_t *)csacStatusRequest, sizeof(csacStatusRequest) -1);

				// wait for response
				first = true;
				while(true) {
					nRecv = 0;
					UART_RTOS_Receive(cmdStruct->spiHandle, csacRecvBuffer, 1, &nRecv);

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
				xSemaphoreGive(cmdStruct->csacMutex);
			}
			break;
		default:
			break;
	}

	vTaskSuspend(NULL);

}

