/*
 * heartbeat.c
 *
 *  Created on: Sep 14, 2016
 *      Author: adwait
 */

#define DEBUG_MODE 1

#include "pulsar_board.h"
#include "application_threads.h"
#include "FreeRTOS.h"				// to have access to portTICK_PERIOD_MS
#include "task.h"
#include "queue.h"

#if DEBUG_MODE == 1
// Assume the debug console has been properly setup by the main thread
#include "fsl_debug_console.h"
#endif // DEBUG_MODE


// LED settings
#ifndef HEARTBEAT_LED_ON
#define HEARTBEAT_LED_ON() 			LED_RED_ON()
#endif	// HEARTBEAT_LED_ON

#ifndef HEARTBEAT_LED_OFF
#define HEARTBEAT_LED_OFF() 		LED_RED_OFF()
#endif // HEARTBEAT_LED_OFF

#ifndef HEARTBEAT_LED_TOGGLE
#define HEARTBEAT_LED_TOGGLE() 		LED_RED_TOGGLE()
#endif //HEARTBEAT_LED_TOGGLE

#ifndef HEARTBEAT_LED_INIT
#define HEARTBEAT_LED_INIT(output) 	LED_RED_INIT(output)
#endif // HEARTBEAT_LED_INIT

#ifndef HEARTBEAT_DEFAULT_RATE_MS
#define HEARTBEAT_DEFAULT_RATE_MS 1000
#endif // HEARTBEAT_DEFAULT_RATE_MS



/**
 * Self contained heartbeat task.
 * The task assumes peripherals are properly setup.
 * It does handle initialization from access to board.h file
 * Other tasks may message the heartbeat to change the beat rate
 * @param pvParameters Command queue handle (QueueHandle_t) for inter-task comm
 */
void heartbeat_thread(void *pvParameters) {
	/*
	 * Discussion: the queue is created and owned by the thread. Thus, the
	 * memory allocation would come from the thread's memory space.
	 * For external threads to access the queue, they can only have access to
	 * the pointer to the locatoion of the queue. Hence QueueHandle_t *q is
	 * passed as a parameter
	 */
	volatile QueueHandle_t *q = ((struct TaskSharedInfo *) pvParameters)->queues[HEARTBEAT_Q];
	volatile int request = 0;
	TickType_t xHeartbeat_period = HEARTBEAT_DEFAULT_RATE_MS/portTICK_PERIOD_MS;
	uint32_t beat_count = 0;

//#if DEBUG_MODE == 1
//	PRINTF("heartbeat: q addr=%p\r\n", q);
//#endif // DEBUG_MODE

//	*q = xQueueCreate(1, sizeof(int));
//	if(*q == NULL) {
//
//#if DEBUG_MODE == 1
//		PRINTF("Q not created. Suspending heartbeat\r\n");
//#endif // DEBUG_MODE
//
//		vTaskSuspend(NULL);
//	}

	// setup heartbeat LED
	HEARTBEAT_LED_INIT(LOGIC_LED_OFF);

	while (true) {		// infinite loop
		beat_count++;		// increments beat counter

		/**
		 * Check queue for new messages
		 */
		if(xQueueReceive(*q, (void *) &request, 0) == pdTRUE) {

			/**
			 * new request received. update heartbeat period
			 */
			if(request > 0) {
				request = 1;
			} else {
				if(request < 0) request = -1;
			}
			xHeartbeat_period = MAX(MIN((xHeartbeat_period + request*100), 1000) ,600);

		}

		// note that all the fractions should total 100
		// 10 + 13 + 12 + 65 = 100
		HEARTBEAT_LED_ON();
		vTaskDelay((xHeartbeat_period * 10) / 100);
		HEARTBEAT_LED_OFF();
		vTaskDelay((xHeartbeat_period * 13) / 100);
		HEARTBEAT_LED_ON();
		vTaskDelay((xHeartbeat_period * 12) / 100);
		HEARTBEAT_LED_OFF();
		vTaskDelay((xHeartbeat_period * 65) / 100);	// wait for next cycle
	}
}
