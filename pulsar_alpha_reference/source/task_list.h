/*
 * task_list.h
 *
 *	The header contains a list of all the tasks used
 *  Created on: Nov 15, 2016
 *      Author: Adwait Dongare
 */

#ifndef SOURCE_TASK_LIST_H_
#define SOURCE_TASK_LIST_H_

/**
 * Console worker: handles commands from user console
 * @param pvParameters
 */
void console_worker_task(void *pvParameters);

/**
 * CSAC worker: handles all communication with CSAC
 * @param pvParameters
 */
void csac_worker_task(void *pvParameters);

/**
 * LED worker: handles LED operations.
 * This implements functions like heartbeat, on, off etc
 * LED identification is provided as a parameter
 * @param pvParameters
 */
void led_worker_task(void *pvParameters);

/**
 *
 * @param pvParameters
 */
void console_task(void *pvParameters);

#endif /* SOURCE_TASK_LIST_H_ */
