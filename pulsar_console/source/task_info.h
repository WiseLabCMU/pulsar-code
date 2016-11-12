/*
 * task_info.h
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#ifndef SOURCE_TASK_INFO_H_
#define SOURCE_TASK_INFO_H_

#include "FreeRTOS.h"
#include "task.h"

/* Task priorities. */
#define heartbeat_PRIORITY 				(tskIDLE_PRIORITY)		// lowest priority possible = 0
//#define watcher_PRIORITY				(configMAX_PRIORITIES -2) // 3
//#define worker_PRIORITY					(configMAX_PRIORITIES -3) 	// 2
//#define application_PRIORITY			(configMAX_PRIORITIES -2) 	// 3
#define commander_PRIORITY				(configMAX_PRIORITIES -2)	// highest priority possible = 4

/*
 * All the task definitions we have
 */
void command_task(void *pvParameters);
void heartbeat_task(void *pvParameters);
//void csac_watcher_task(void *pvParameters);
//void pll_watcher_task(void * pvParameters);
//void dw_watcher_task(void *pvParameters);
//void messenger_task(void *pvParameters);
//void discipline_task(void *pvParameters);

#endif /* SOURCE_TASK_INFO_H_ */
