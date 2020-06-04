/*
 * application_threads.h
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#ifndef SOURCE_APPLICATION_THREADS_H_
#define SOURCE_APPLICATION_THREADS_H_

#include "fsl_uart_freertos.h"
#include "fsl_dspi_freertos.h"
#include "pulsar_board.h"

/*
 * Useful constants
 */

enum SEM_ID {
	CSAC_PPS_SEM = 0,
	SEM_N,			// 1 total semaphores for task synchronization
};

enum TASK_ID {
	COMMAND 		= 0,
	HEARTBEAT 		= 1,
	CSAC_WATCHER 	= 2,
	TASK_N,	// 3 total tasks
};

enum Q_ID {
	HEARTBEAT_Q = 0,
//	CSAC_WORKER_RESP_Q = 1,
	Q_N,
};

enum MUTEX_ID {
	CSAC_UART_MUTEX = 0,
	MUTEX_N,
};

#define heartbeat_PRIORITY 				(tskIDLE_PRIORITY)		// lowest priority possible = 0
#define watcher_PRIORITY				(configMAX_PRIORITIES -2) // 3
#define worker_PRIORITY					(tskIDLE_PRIORITY + 1) 	// 1
#define application_PRIORITY			(tskIDLE_PRIORITY + 2) 	// 2
#define commander_PRIORITY				(configMAX_PRIORITIES -1)	// highest priority possible = 4

struct TaskSharedInfo {
	volatile SemaphoreHandle_t 	*semaphores;
	volatile SemaphoreHandle_t 	*mutex;
	volatile QueueHandle_t 		*queues;
	volatile TaskHandle_t  		*tasks;
};

/**
 * Keeps a check on the state of the CSAC
 * Objective is to detect failures and restart the respective processes
 * @param pvParameters
 */
void csac_watcher_thread(void *pvParameters);


/**
 * Heartbeat thread. Blinks specified LED in heartbeat pattern
 * @param pvParameters Command queue handle (QueueHandle_t) for inter-task comm
 */
void heartbeat_thread(void *pvParameters);

#endif /* SOURCE_APPLICATION_THREADS_H_ */
