/*
 * csac_threads.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"
#include "csac_driver.h"

static const uint8_t csacStatusRequest[] = "!^\r\n";
static const uint8_t csacFreqSteerCmd[] = "!FD1000000\r\n";
static uart_rtos_handle_t csac_uart_handle;

void csac_watcher_thread(void *pvParameters) {


	volatile SemaphoreHandle_t csac_pps_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_PPS_SEM];
	volatile char buf[128];			// 128 character buffer
	uint8_t uart_buf[1];	// 1 char UART buffer
//	int csac_state = -1;
	bool first = true;
	bool f_tune = false;
	size_t nRecv = 0;
	int ptr = 0;
	TickType_t t;
//	uint32_t pin_state;
//	uint32_t last_pin_state;

	PRINTF("#csac_watcher_thread started\r\n");
	/*
	 * initialize peripherals
	 * Lock pin is input to uC
	 * PPS Out is input to uC
	 */




	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});
	GPIO_PinInit(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN, &(gpio_pin_config_t){kGPIO_DigitalInput, 0});

	csac_communication_init(&csac_uart_handle, BOARD_CSAC_UART_CLK_FREQ, uart_buf, sizeof(uart_buf));

	// infinite watcher loop start
	while(true) {
		t = xTaskGetTickCount();
		first = true;
		f_tune = false;
		/*
		 * wait for lock pin to turn high
		 * uses simple polling for now
		 */
		// TODO: use interrupts for this later
		while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_LOCK_PIN) == 0) {
			if(first) {
				vTaskDelay(4000);
				first = false;
			}
			else vTaskDelay(50);
		}


		/*
		 * Configure interrupts for PPS pin
		 */
//		PORT_SetPinInterruptConfig(BOARD_CSAC_GPIO_PORT, BOARD_CSAC_PPSOUT_PIN, kPORT_InterruptRisingEdge);
//		EnableIRQ(PORTA_IRQn);		// enable IRQ


		/*
		 * check CSAC state over UART
		 */
		while(true) {

			ptr = 0;

			// wait for PPS line approximately
//			PRINTF("waiting\r\n");
//			xSemaphoreTake(csac_pps_sem, portMAX_DELAY);
//			pin_state = 1;
//			last_pin_state = 1;
//			while(true) {
//				pin_state = GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN);
//				if(pin_state ==  1 && last_pin_state == 0) {
//					break;
//				} else {
//					last_pin_state = pin_state;
//				}
//			}

//			while(GPIO_ReadPinInput(BOARD_CSAC_GPIO, BOARD_CSAC_PPSOUT_PIN) == 0)

			// send UART request
			csac_send(&csac_uart_handle, csacStatusRequest, sizeof(csacStatusRequest)-1);

			// hope that the
			while(ptr < 127) {
				csac_receive(&csac_uart_handle, uart_buf, 1, &nRecv);
				buf[ptr] = (char) uart_buf[0];
				ptr++;
				if(uart_buf[0] == '\n') {
					break;
				}
			}
			buf[ptr] = '\0';

			PRINTF("[%u] %s", xTaskGetTickCount(), buf);

			if(f_tune == false && (t > (1000*60*5))) {
				csac_send(&csac_uart_handle, csacFreqSteerCmd, sizeof(csacFreqSteerCmd) -1 );
				f_tune = true;
				PRINTF("# !FD1000000 correction applied\r\n");
			}

//			vTaskDelay(1000);
			vTaskDelayUntil(&t, 1000);

		}

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

//// TODO: setup worker for single command
//void csac_worker_thread(void *pvParameters) {
//
//	int csac_state = -1;
//	size_t nRecv = 0;
//	uint8_t csacRecvBuffer[1];
//	bool first = true;	// flag to indicate if this is first byte in stream
//	// TODO: pointer to necesssary sem_C
//	volatile SemaphoreHandle_t csac_worker_prod_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_P_SEM];
//	volatile SemaphoreHandle_t csac_worker_con_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[CSAC_WORK_C_SEM];
//	volatile QueueHandle_t worker_response_q = ((struct TaskSharedInfo *) pvParameters)->queues[CSAC_WORKER_RESP_Q];
//
//	while(true) {
//		// wait till watcher requests for read service
//		xSemaphoreTake(csac_worker_prod_sem, portMAX_DELAY);
//
//		// receive and parse state value
//		while(true) {
//			nRecv = 0;
//
//			// this is a blocking functions which may never return if the
//			// necessary peripherals are bad or off. How do we deal with this
//			// case?
//			UART_RTOS_Receive(&csac_uart_handle, csacRecvBuffer, 1, &nRecv);
//
//			if(first == true) {
//				csac_state = (int)csacRecvBuffer[0] - (int)'0';
//				first = false;
//			}
//
//			if(csacRecvBuffer[0] == (uint8_t) '\n') {
//				break;
//			}
//		}
//		// add state result to queue
//		xQueueSendToBack(worker_response_q, &csac_state, 0);
//
//		// release consumer semaphore
//		xSemaphoreGive(csac_worker_con_sem);
//	}
//	vTaskSuspend(NULL);
//
//}
