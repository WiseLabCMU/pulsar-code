/*
 * application_threads.h
 *
 *  Created on: Sep 12, 2016
 *      Author: adwait
 */

#ifndef SOURCE_APPLICATION_THREADS_H_
#define SOURCE_APPLICATION_THREADS_H_



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
