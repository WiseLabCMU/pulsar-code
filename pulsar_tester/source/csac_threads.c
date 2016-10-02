/*
 * csac_threads.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"
#include "csac_driver.h"

static const uint8_t csacStatusRequest[4] = "!^\r\n";
static uart_rtos_handle_t csac_uart_handle;

void csac_watcher_thread(void *pvParameters) {

	volatile SemaphoreHandle_t csac_pll_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_PLL_SEM];
	volatile SemaphoreHandle_t csac_worker_prod_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_P_SEM];
	volatile SemaphoreHandle_t csac_worker_con_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_C_SEM];
//	volatile SemaphoreHandle_t csac_uart_mutex = ((struct TaskSharedInfo *) pvParameters)->mutex[CSAC_UART_MUTEX];
	volatile QueueHandle_t csac_worker_response = ((struct TaskSharedInfo *) pvParameters)->queues[CSAC_WORKER_RESP_Q];
	uint8_t buf[8];	// 8 character buffer
	int csac_state = -1;


//	PRINTF("csac_watcher_thread started\r\n");
	/*
	 * initialize peripherals
	 */

	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

	csac_communication_init(&csac_uart_handle, BOARD_CSAC_UART_CLK_FREQ, buf, sizeof(buf));

	// infinite watcher loop start
	while(true) {
		/*
		 * wait for lock pin to turn high
		 * uses simple polling for now
		 */
		// TODO: use interrupts for this later
		while(!GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN)) {
			vTaskDelay(50);
		}

//		PRINTF("CSAC lock pin was on\r\n");

		/*
		 * check CSAC state over UART
		 */
		while(true) {
//			if(xSemaphoreTake(csac_uart_mutex, 0) == pdTRUE) {	// lock access to UART
				csac_send(&csac_uart_handle, csacStatusRequest, 4); // send status query request
//				xSemaphoreGive(csac_uart_mutex);
//			}
				xSemaphoreGive(csac_worker_prod_sem);

			// wait for response from worker
			if(xSemaphoreTake(csac_worker_con_sem, 200) == pdTRUE) {
				// worker responded. Read CSAC state from queue
				xQueueReceive(csac_worker_response, &csac_state, 0);
//				PRINTF("CSAC: %d\r\n", csac_state);
				// state was correct. break
				if(csac_state == 0) break;
			}

			// if you get here the worker didn't respond in time.
			// re-send query for state

		}

		xSemaphoreGive(csac_pll_sem);
		PRINTF("CSAC lock detected\r\n");

		/*
		 * Wait and repeat
		 */
		vTaskSuspend(NULL);	// temporary command
	}

}

/**
 * The worker manages all communication with the CSAC
 * An application will spawn a new worker when it needs to communicate
 * with the CSAC
 *
 * @param pvParameters
 */

// TODO: setup worker for single command
void csac_worker_thread(void *pvParameters) {

	int csac_state = -1;
	size_t nRecv = 0;
	uint8_t csacRecvBuffer[1];
	bool first = true;	// flag to indicate if this is first byte in stream
	// TODO: pointer to necesssary sem_C
	volatile SemaphoreHandle_t csac_worker_prod_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_P_SEM];
	volatile SemaphoreHandle_t csac_worker_con_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_C_SEM];
	volatile QueueHandle_t worker_response_q = ((struct TaskSharedInfo *) pvParameters)->queues[CSAC_WORKER_RESP_Q];

	while(true) {
		// wait till watcher requests for read service
		xSemaphoreTake(csac_worker_prod_sem, portMAX_DELAY);

		// receive and parse state value
		while(true) {
			nRecv = 0;

			// this is a blocking functions which may never return if the
			// necessary peripherals are bad or off. How do we deal with this
			// case?
			UART_RTOS_Receive(&csac_uart_handle, csacRecvBuffer, 1, &nRecv);

			if(first == true) {
				csac_state = (int)csacRecvBuffer[0] - (int)'0';
				first = false;
			}

			if(csacRecvBuffer[0] == (uint8_t) '\n') {
				break;
			}
		}
		// add state result to queue
		xQueueSendToBack(worker_response_q, &csac_state, 0);

		// release consumer semaphore
		xSemaphoreGive(csac_worker_con_sem);
	}
	vTaskSuspend(NULL);

}
