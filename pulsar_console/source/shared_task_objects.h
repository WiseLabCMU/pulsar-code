/*
 * shared_task_objects.h
 *
 *  This should contain all objects shared between different tasks
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#ifndef SOURCE_SHARED_TASK_OBJECTS_H_
#define SOURCE_SHARED_TASK_OBJECTS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "fsl_uart_freertos.h"
#include "fsl_dspi_freertos.h"

/******************************************************************************
 * LISTS section
 *****************************************************************************/

/*
 * List of all binary semaphores we expect to have
 */
enum SEM_ID {
	CSAC_PLL_OK_SEM 	= 0,	// indicates CSAC lock to PLL
	PLL_DW_OK_SEM 		= 1,	// indicates PLL lock to DW
	DW_MSG_OK_SEM 		= 2,	// indicates DW init to messenger
	MSG_DISC_OK_SEM 	= 3,	// indicates discipline values available
	DW_IRQ_SEM 			= 4,	// indicates IRQ event
	DW_PPS_SEM 			= 5,	// indicates PPS event
	HB_SEM				= 6,	// indicates trigger to beat now
	SEM_N,
};

/*
 * List of all the tasks we expect to have
 */
enum TASK_ID {
	COMMAND_TASK 		= 0,
	HEARTBEAT_TASK 		= 1,
	CSAC_WATCHER_TASK 	= 2,
	PLL_WATCHER_TASK 	= 3,
	DW_WATCHER_TASK 	= 4,
	MESSENGER_TASK 		= 5,
	DISCIPLINE_TASK 	= 6,
	TASK_N,           //
};

/*
 * List of all the queues we expect to have
 */

enum QUEUE_ID {
	HEARTBEAT_Q			= 0,
	QUEUE_N,
};



/******************************************************************************
 * end of LISTS section
 *****************************************************************************/

extern volatile SemaphoreHandle_t semaphore[SEM_N];

extern volatile TaskHandle_t tasks[TASK_N];

extern volatile QueueHandle_t queues[QUEUE_N];

// The CSAC UART is currently shared, but should not be
extern uart_rtos_handle_t csac_uart_handle;	// CSAC UART
// The DW SPI is currently shared but should not be
extern dspi_rtos_handle_t dw_rtos_handle;	// Deacwave SPI


// Timestamp information passed between message and discipline tasks through global variables.
// Perhaps pass this through a structure later

extern volatile uint32_t rx_msg_no;
extern volatile uint64_t local_rx_timestamp;
extern volatile uint64_t local_tx_timestamp;
extern volatile uint64_t reference_tx_timestamp;
extern volatile uint64_t previous_rx_timestamp;
extern volatile uint64_t global_offset;

// ID number for node
#define NODE_ID 2
#define REFERENCE_ID	1

// Type of node
#define REFERENCE_NODE 	0
#define RELAY_NODE 		0
#define CHILD_NODE 		1

// converts 64 bit timestamp into 2 32 bit timestamps
// this is useful while printing
#define TS64_TO_TS32(ts64,index) ( ((uint32_t *)&ts64)[index] )

// shortcut string modifiers to print 64 bit and 40 bit timestamps easily
#define TS40_XSTR "0x%02X%08X"	// used to print 40 bit timestamps in hex
#define TS64_XSTR "0x%08X%08X" // used to print 64 bit timestamps in hex

#define BEACON_SIZE 30

enum BEACON_MSG_OFFSETS {
	beacon_TYPE = 0,	// type of message
	beacon_ORIGIN = 1,	// ID of origin node
	beacon_TARGET = 2,	// ID of target node
	beacon_NUMBER = 3,	// message number (4 bytes)
	beacon_TX_TS = 7,	// 64 bit beacon timestamp (8 bytes)
	beacon_CHILD_RX_TS = 15,// 40 bit child timestamp from previous exchange (5 bytes)
	beacon_GOFFSET = 20,// offset between local clock and global time (8 bytes)
	beacon_CHECKSUM = 28,	// 2 bytes for the checksum (2 bytes)
	beacon_SIZE = BEACON_SIZE,	// total size of message
};

#define RESPONSE_SIZE 9

enum RESP_MSG_OFFSETS {
	response_TYPE = 0,	// type of message
	response_ORIGIN = 1,	// ID of origin node
	response_TARGET = 2,	// ID of target node
	response_NUMBER = 3,	// message number (4 bytes)
	response_CHECKSUM = 7,	// 2 bytes for checksum (2 bytes)
	response_SIZE = RESPONSE_SIZE,
};

#endif /* SOURCE_SHARED_TASK_OBJECTS_H_ */
