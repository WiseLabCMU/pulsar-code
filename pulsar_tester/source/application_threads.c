/*
 * application_thread.c
 *
 *  Created on: Sep 26, 2016
 *      Author: adwait
 */

#include "application_threads.h"
#include "fsl_debug_console.h"
#include "deca_device_api.h"
#include "deca_regs.h"

/*
 * Useful defines
 */

/*
 * This will help setup the TDMA scheme
 */
// ID number for node
#define NODE_ID 1

#define BEACON_SLOT 	1
#define RESPONSE_SLOT 	0

/*
 * TDMA parameters
 */

#define N_TDMA_SLOTS 100		// 100 slots per second at gap of 5 msec

/*
 * the number of children, siblings and parents determines where the node lies
 * in the tree
 *
 * A reference node will only have non-zero children
 * A relay node will have non-zero children and a parent
 * An end node will have a parent, no children and optionally siblings
 *
 */

#define N_CHILDREN 	1
#define N_SIBLINGS 	0
#define N_PARENT 	0

#define BEACON_MSG 		1
#define RESPONSE_MSG 	2

/*
 * Time conversion sections:
 * 1 UWB us (UUS)	= 1.0256 real microsec
 * 65536 DW TICKS 	= 1 UUS
 * 63897.6 DW TICKS = 1 real microsec
 */

#define UUS2DWT(uus) 	(uint64_t)((uint64_t)uus*65536) 			// UUS to DW ticks
#define SEC2DWT(sec) 	(uint64_t)((uint64_t)sec*63897600000ULL) 	// second to DW ticks
#define USEC2DWT(usec) 	(uint64_t)((uint64_t)usec*638976/10) 	// microsec to DW ticks. Has residual error

#define SEC_2_DWT 		(63897600000ULL)	// conversion factor: sec to DW ticks
#define UUS_TO_DWT_TIME 65536
#define US_TO_DWT_TIME 	63897.6 			// Warning: use this only if using floats

// converts 64 bit timestamp into 2 32 bit timestamps
// this is useful while printing
#define TS64_TO_TS32(ts64,index) ( ((uint32_t *)&ts64)[index] )

// shortcut string modifiers to print 64 bit and 40 bit timestamps easily
#define TX40_XSTR "0x%02X%08X"	// used to print 40 bit timestamps in hex
#define TS64_XSTR "0x%08X%08X" // used to print 64 bit timestamps in hex

// Default antenna delay values for 64 MHz PRF
#define TX_ANT_DLY 16436		// in DW time units
// Default antenna delay values for 64 MHz PRF
#define RX_ANT_DLY 16436		// in DW time units
// RX after TX delay timeout
#define RX_AFTER_TX_DLY 1200 	// in UUS
// TX after RX delay time
// NOTE: glitches for time smaller than 2950 UUS
//#define TX_AFTER_RX_DLY 4000 	// in UUS
// RX state timeout (this is a reasonable period which gets a message through)
#define RX_TIMEOUT	8000 		// in UUS

#define LIGHT_US 299.702547		// meter distance covered by light in 1 real microsec

// 1 UWB microsec = 128 x 512 x DW time


// 1 real microsec = 128 x 499.2 x DW time


#define SEC_TO_DW_TICK 63897600000



// conversion of 1 UWB microsec in DW time units
#define UUS_TO_DWCOUNT(uus) (uint64_t)((uint64_t)uus * 65536)

/*
 * Useful global variables
 */

// Beacons are only sent out if you have child nodes
#define BEACON_SIZE 30
#if (N_CHILDREN > 0)
/*
 * beacon_message structure
 * # preamble section
 * 1: 			message type
 * 2: 			origin node ID
 * 3: 			beacon slot
 * 4,5,6,7: 	message number
 * 8,9,10,11,12,13,14,15: 	TX timestamp (in 64 bit values)
 * 16,17:		Child IDs
 * 18,19: 		Child response slots
 * 20,21,22,23,24: Child 1 RX timestamp
 * 25,26,27.28.29: Child 2 RX timestamp
 * 29,30: 		Checksum
 */
static char beacon_message[BEACON_SIZE];
#endif

// Responses are only sent out if you have parent nodes
#define RESPONSE_SIZE 15
#if (N_PARENT > 0)
/*
 * response_message structure
 * # preamble section
 * 1: 			message type
 * 2: 			origin node ID
 * 3: 			response slot
 * 4:			parent node ID
 * 5,6,7,8: 	message number
 * 9,10,11,12,13: 	TX timestamp
 * 14,15: 		Checksum
 */
static char response_message[RESPONSE_SIZE];
#endif

// Decawave radio configuration
static dwt_config_t channel_config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/*
 * useful functions
 */

/**
 * Find the next acceptable DW time
 * @param last_send_time
 * @param interval_uus
 * @return
 */
static inline uint32_t compute_next_dwtime_uus(uint32_t last_send_time, uint32_t interval_uus) {
	return((last_send_time + interval_uus)&0xFFFFFFFE);
}

/**
 * Find the next send time based on value of last send time and microsecond delay.
 * This function will have a residual value due to integer division
 * @param last_send_time
 * @param usec
 * @return The next send time (in 32-bit). Note that this will have residual errors
 */
static inline uint32_t compute_next_dwtime_usec(uint32_t last_send_time, uint32_t interval_usec) {
	return ((last_send_time + ((interval_usec*4992UL)/5120UL))&0xFFFFFFFE);
}

/**
 * Use this function while embedding timestamps in beacon messages
 * @param send_time
 * @param overflow_correction
 * @return
 */
static inline uint64_t sendtime_to_expected_timestamp(uint32_t send_time, uint64_t overflow_correction) {
	return ((( (uint64_t) (send_time & 0xFFFFFFFE) ) << 8) + TX_ANT_DLY + overflow_correction);
}


/*
 * The application thread needs access to the CSAC and DW worker threads for interaction
 *
 * Note to self: the DW clock is king. Any and every timing must be done
 * with the decawave clocks
 */
void messaging_thread(void *pvParameters) {

	uint32_t msg_no = 0;	// keep a track of the message number

	uint32_t programmed_send_time; 	// used for storing 31 bit DW delayed TX time
//	uint32_t programmed_recieve_time; 	// used for storing 31 bit DW delayed RX time

	uint64_t dw_time = 0;	// store decawave clock time
	uint64_t tx_timestamp = 0;
	uint64_t read_tx_timestamp = 0;	// temporary: actual timestamp seen by chip

	volatile SemaphoreHandle_t dw_app_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_APP_SEM];
	volatile SemaphoreHandle_t dw_irq_sem = ((struct TaskSharedInfo *) pvParameters)->semaphores[DW_IRQ_SEM];
	/*
	 * Wait for lock release from DW watcher
	 * If all the previous systems are properly locked the dw_watcher will
	 * give a semaphore
	 */
	xSemaphoreTake(dw_app_sem, portMAX_DELAY);

	/*
	 * Initialize radio configuration
	 */

	dwt_configure(&channel_config);		// configure decawave channel parameters
	dwt_setrxantennadelay(RX_ANT_DLY);	// antenna delay value for 64 MHz PRF
	dwt_settxantennadelay(TX_ANT_DLY);	// antenna delay value for 64 MHz PRF

	// RX timeout after start
	dwt_setrxtimeout(RX_TIMEOUT);			// in UWB microsec

	/*
	 * beacon_message structure guide
	 * # preamble section
	 * 1: 			message type
	 * 2: 			origin node ID
	 * 3: 			beacon slot
	 * 4,5,6,7: 	message number
	 * 8,9,10,11,12,13,14,15: 	TX timestamp (in 64 bit values)
	 * 16,17:		Child IDs
	 * 18,19: 		Child response slots
	 * 20,21,22,23,24: Child 1 RX timestamp
	 * 25,26,27.28.29: Child 2 RX timestamp
	 * 29,30: 		Checksum
	 */

	// set init values for beacon message
	beacon_message[0] = BEACON_MSG; 	// type of message (unchanged)
	beacon_message[1] = NODE_ID; 		// ID of beacon originator (unchanged)
	beacon_message[2] = BEACON_SLOT; 	// Which slot is this transmitting in (unchanged)
	memset((void *)&beacon_message[3], 0, 4 + 8);	// clear message number and TX timestamp (volatile)
	beacon_message[15] = 2;				// 1st child ID (unchanged)
	beacon_message[16] = 0; 			// 2nd child ID (unchanged)
	beacon_message[17] = 50; 			// 1st child response slot (unchanged)
	beacon_message[18] = 0; 			// 2nd child response slot (unchanged)
	memset((void *)&beacon_message[19], 0, 5 + 5 + 2);	// clear timestamp and checksum bits (volatile)

	/*
	 * All preliminary systems are now locked and ready, prepare for main operation
	 */




	dwt_readsystime((uint8 *) &dw_time);
	programmed_send_time = (uint32_t)(dw_time>>8);	// bootstrap the process with current decawave time

	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

	while(true) {

		// the objective here is to see if we can send 100 messages every second
		// also to estimate how long a message transmission takes

#if N_CHILDREN > 0
		msg_no++;	// increment message number

		/*
		 * beacon message preparations
		 */

		// compute time to send next message
		programmed_send_time = compute_next_dwtime_usec(programmed_send_time, 10000); // schedule next message for 10msec from now
		tx_timestamp = sendtime_to_expected_timestamp(programmed_send_time, 0); // compute expected timestamp

		// construct message
		memcpy((void *) &beacon_message[3], (void *) &msg_no, 4);
		memcpy((void *) &beacon_message[7], (void *) &tx_timestamp, 8);

		// TODO: write parsers for child responses

		// write data to DW1000
		dwt_writetxdata(sizeof(beacon_message), (uint8_t *) beacon_message, 0);
		dwt_writetxfctrl(sizeof(beacon_message), 0, 1);

		// program chip to send message at correct time
		dwt_setdelayedtrxtime(programmed_send_time);
		dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_SFDT | DWT_INT_ARFE, (uint8_t) 1);	// enable only transmit frame sent interrupt & ignore all other events

		// begin and wait for transmission
		dwt_starttx(DWT_START_TX_DELAYED);

		// wait for semaphore to be given from IRQ

		if(xSemaphoreTake(dw_irq_sem, 1000) != pdTRUE) {
			// transmission failed. this needs some damage control

			// renew notion of current time
			dwt_readsystime((uint8 *) &dw_time);
			programmed_send_time = (uint32_t)(dw_time>>8);	// bootstrap the process with current decawave time
			dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);	// disable interrupt
			PRINTF("err\r\n");

			continue;	// start from beginning of the loop and hope all goes well
		}

		dwt_isr();		// run ISR function to complete callbacks etc
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);	// clear transmit status flag

		// read timestamp
		dwt_readtxtimestamp((uint8 *) &read_tx_timestamp);

		read_tx_timestamp -= tx_timestamp;

		PRINTF("%u:" TX40_XSTR "\r\n", msg_no, TS64_TO_TS32(read_tx_timestamp,1), TS64_TO_TS32(read_tx_timestamp,0));

#endif // N_CHILDREN > 0

	}

	// we should hopefully never get here. if we do, suspend the task
	vTaskSuspend(NULL);
}


/**
 * This thread is responsible for hardware disciplining the CSAC.
 *
 * This application currently waits for a little while before sending a
 * single discipline command to the CSAC to look at the frequency change over
 * time
 *
 * @param pvParameters
 */
void clock_discipline_thread(void *pvParameters) {

	// DO NOTHING FOR NOW

	// wait for successful sample from messaging thread

	// compute offset and frequency if possible

	vTaskSuspend(NULL);
}



