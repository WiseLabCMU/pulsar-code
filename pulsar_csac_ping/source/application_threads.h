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

/*
 * Useful constants
 */

enum QID {
	CSAC_PLL_SEM 	= 0,
	PLL_DW_SEM 		= 1,
	DW_APP_SEM 		= 2,
	SEM_N
};

#define heartbeat_PRIORITY 				(tskIDLE_PRIORITY)		// lowest priority possible = 0
#define watcher_PRIORITY				(configMAX_PRIORITIES -2) // 3
#define worker_PRIORITY					(tskIDLE_PRIORITY + 1) 	// 1
#define application_PRIORITY			(tskIDLE_PRIORITY + 2) 	// 2
#define commander_PRIORITY				(configMAX_PRIORITIES -1)	// highest priority possible = 4


/**
 * Enums
 */

/**
 * possible commands that have been implemented for PLL worker thread
 */
enum pllWorkerCmd {
	cmdPLL_NONE 		= 0, 	//!< cmdNONE 			Do nothing
	cmdPLL_RESET,     			//!< cmdRESET 			Reset/restart the PLL
	cmdPLL_INIT_DEFAULT,		//!< cmdINIT_DEFAULT 	Initialize with default 10->38.4MHz settings
	cmdPLL_READ_REG,   			//!< cmdREAD_REG 		Read register in param
	cmdPLL_WRITE_REG,  			//!< cmdWRITE_REG 		Write to register
	cmdPLL_DISABLE,   			//!< cmdDISABLE 		Disable the PLL
	cmdPLL_N,				//!< cmdPLL_N 			Number of PLL commands
};
typedef enum pllWorkerCmd pllWorkerCmd_t;

/**
 * possible commands that will be implemented for CSAC worker
 */
enum csacWorkerCmd {
	cmdCSAC_NONE 		= 0, 	//!< cmdNONE 		Do nothing
	cmdCSAC_GET_STATE, 			//!< cmdGET_STATE 	Get current state of CSAC
	cmdCSAC_N, 					//!< cmdCSAC_N 		Number of CSAC commands
};
typedef enum csacWorkerCmd csacWorkerCmd_t;

/**
 * commands that will be implemented for DW worker
 */
enum dwWorkerCmd {
	cmdDW_NONE 	= 0, 	//!< cmdNONE 		Do nothing
	cmdDW_RESET,    		//!< cmdRESET 		Reset the chip
	cmdDW_SEND_MSG,		//!< cmdSEND_MSG 	Send message
	cmdDW_N,     		//!< cmdDW_N 		Number of DW commands
};
typedef enum dwWorkerCmd dwWorkerCmd_t;

/**
 * Structures
 */
struct pllWorkerCmdStruct {
	enum pllWorkerCmd 		cmd; 		// required command structure
	dspi_rtos_handle_t 		*spiHandle; // required handle to spi device
	QueueHandle_t 			qHandle; 	// (optional) queue handle for returns
	SemaphoreHandle_t 		pllMutex;	// (optional) device access mutex
	volatile union lmx2571_register 	*data; 		// (optional) pointer to data
	// TODO: add required mutexes to command struct later
};

struct csacWorkerCmdStruct {
	enum csacWorkerCmd 		cmd; 		// required command structure
	uart_rtos_handle_t 		*spiHandle; // required handle to spi device
	QueueHandle_t 			qHandle; 	// (optional) queue handle for returns
	SemaphoreHandle_t 		csacMutex;	// device access mutex
	volatile void 			*data; 		// (optional) pointer to data
	// TODO: add required mutexes to command struct later
};

struct dwWorkerCmdStruct {
	enum dwWorkerCmd 		cmd; 		// required command structure
	dspi_rtos_handle_t 		*spiHandle; // required handle to spi device
	QueueHandle_t 			qHandle; 	// (optional) queue handle for returns
	SemaphoreHandle_t 		dwMutex;	// device access mutex
	volatile void 			*data; 		// (optional) pointer to data
	// TODO: add required mutexes to command struct later
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
 * This is the main application thread
 * @param pvParameters
 */
void application_thread(void *pvParameters);

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

#endif /* SOURCE_APPLICATION_THREADS_H_ */
