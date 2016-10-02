/*
 * application_threads.h
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#ifndef SOURCE_APPLICATION_THREADS_H_
#define SOURCE_APPLICATION_THREADS_H_

#include "lmx2571_registers.h"
#include "fsl_uart_freertos.h"
#include "fsl_dspi_freertos.h"

#include "pulsar_board.h"

/*
 * Useful constants
 */

/**
 *
 */
enum SEM_ID {
	CSAC_PLL_SEM 	= 0,  //!< CSAC_PLL_SEM
	CSAC_WORK_P_SEM = 1,//!< CSAC_WORK_P_SEM
	CSAC_WORK_C_SEM = 2,//!< CSAC_WORK_C_SEM
	PLL_DW_SEM 		= 3,   //!< PLL_DW_SEM
	DW_APP_SEM 		= 4,   //!< DW_APP_SEM
	DW_IRQ_SEM 		= 5,   //!< DW_IRQ_SEM
	DW_PPS_SEM		= 6,   //!< DW_PPS_SEM
	SEM_N,			// 6 total semaphores for task synchronization
};

/**
 *
 */
enum TASK_ID {
	COMMAND 		= 0,    //!< COMMAND
	HEARTBEAT 		= 1,  //!< HEARTBEAT
	CSAC_WATCHER 	= 2,//!< CSAC_WATCHER
	PLL_WATCHER 	= 3, //!< PLL_WATCHER
	DW_WATCHER 		= 4, //!< DW_WATCHER
	CSAC_WORKER 	= 5, //!< CSAC_WORKER
	APPLICATION		= 6, //!< APPLICATION
	TASK_N,           //!< TASK_N
};

/**
 *
 */
enum Q_ID {
	HEARTBEAT_Q = 0,       //!< HEARTBEAT_Q
	CSAC_WORKER_RESP_Q = 1,//!< CSAC_WORKER_RESP_Q
	Q_N,                   //!< Q_N
};

/**
 *
 */
enum MUTEX_ID {
	CSAC_UART_MUTEX = 0,//!< CSAC_UART_MUTEX
	PLL_SPI_MUTEX 	= 1, //!< PLL_SPI_MUTEX
	DW_SPI_MUTEX 	= 2,  //!< DW_SPI_MUTEX
	MUTEX_N,            //!< MUTEX_N
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
 * Keeps a check on the state of the PLL
 * @param pvParameters
 */
void pll_watcher_thread(void *pvParameters);

/**
 * Keeps a check on the state of the CSAC
 * Objective is to detect failures and restart the respective processes
 * @param pvParameters
 */
void csac_watcher_thread(void *pvParameters);

/**
 * Keeps a check on the state of the DW
 * Objective is to detect failures and restart the respective peripheral
 * @param pvParameters
 */
void dw_watcher_thread(void *pvParameters);

/**
 * This thread is responsible for all application messaging on the decawave
 * @param pvParameters
 */
void messaging_thread(void *pvParameters);

/**
 * This thread is responsible for implementing the discipline algorithm
 * @param pvParameters
 */
void clock_discipline_thread(void *pvParameters);

/**
 * This thread is responsible for handling all communications with the CSAC
 * over the UART interface.
 * Other application threads will request services from the worker for CSAC
 * interactions
 * @param pvParameters
 */
void csac_worker_thread(void *pvParameters);

/**
 * This thread is responsible for handling all communications with the RF Synth
 * over the SPI interface. Other application specific threads may request
 * services from the worker
 * @param pvParameters
 */
void pll_worker_thread(void *pvParameters);

/**
 * This thread is responsible for handling all communications with the DW1000
 * over the SPI interface. Other application specific threads may request
 * services from the worker.
 * @param pvParameters
 */
void dw_worker_thread(void *pvParameters);

/**
 * Heartbeat thread. Blinks specified LED in heartbeat pattern
 * @param pvParameters Command queue handle (QueueHandle_t) for inter-task comm
 */
void heartbeat_thread(void *pvParameters);

extern volatile uint32_t dw_pps_count;

#endif /* SOURCE_APPLICATION_THREADS_H_ */
