/*
 * messager.c
 *
 *  Created on: Oct 29, 2016
 *      Author: adwait
 */

#include "pulsar_board.h"

#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_device_api.h"

#include "fsl_debug_console.h"

#include "shared_task_objects.h"
#include "task_info.h"

/******************************************************************************
 * MESSENGER section
 *****************************************************************************/



/*
 * TDMA parameters
 * Number of slots in a cycle and width of each slot determine the total cycle
 * period. Here we use 1 sec long cycles with 25 slots, each of size 40msec.
 * This may be extended all the way to 5 msec and 200 slots if desired.
 *
 */
#define N_TDMA_SLOTS	25		// 25 slots per second at gap of 40 msec
#define TDMA_WIDTH_UUS	39000	// each TDMA slot is 39000 UUS or 40000 microsec long

enum msg_type {
	BEACON_MSG_ID = 1, RESPONSE_MSG_ID = 2,
};

/*
 * Time conversions:
 * 1 UWB us (UUS)	= 1.0256 real microsec
 * 65536 DW TICKS 	= 1 UUS
 * 63897.6 DW TICKS = 1 real microsec
 */
#define USEC2UUS(usec)	((usec*39)/40)
#define UUS2DWT(uus) 	(uint64_t)((uint64_t)uus*65536) 			// UUS to DW ticks
#define SEC2DWT(sec) 	(uint64_t)((uint64_t)sec*63897600000ULL) 	// second to DW ticks
#define USEC2DWT(usec) 	(uint64_t)((uint64_t)usec*638976/10) 	// microsec to DW ticks. Has residual error
#define SEC_2_DWT 		(63897600000ULL)	// conversion factor: sec to DW ticks
#define UUS_TO_DWT_TIME 65536
#define US_TO_DWT_TIME 	63897.6 			// Warning: use this only if using floats
#define LIGHT_US 299.702547		// meter distance covered by light in 1 real microsec
#define SEC_TO_UUS 		975000

// Default antenna delay values for 64 MHz PRF
#define TX_ANT_DLY	16436		// in DW time units
#define RX_ANT_DLY	16436		// in DW time units
#define RX_TIMEOUT	24375 		// in UUS

#define RX_WINDOW_KLUDGE_UUS 	50	// kludge factor to start recieve window earlier
#define RX_TIMEOUT_KLUDGE	(RX_TIMEOUT + RX_WINDOW_KLUDGE_UUS) // in UUS with added kludge factor

// Decawave radio configuration
static dwt_config_t channel_config = { 2, /* Channel number. */
DWT_PRF_64M, /* Pulse repetition frequency. */
DWT_PLEN_1024, /* Preamble length. */
DWT_PAC32, /* Preamble acquisition chunk size. Used in RX only. */
9, /* TX preamble code. Used in TX only. */
9, /* RX preamble code. Used in RX only. */
1, /* Use non-standard SFD (Boolean) */
DWT_BR_110K, /* Data rate. */
DWT_PHRMODE_STD, /* PHY header mode. */
(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};



// these are to be used as tx and rx buffers
static volatile char beacon_message[BEACON_SIZE];// use as buffer for beacon message
static volatile char response_message[RESPONSE_SIZE];// use to construct local response



/*
 * useful functions
 */

/**
 * Find the next acceptable DW time
 * @param last_send_time
 * @param interval_uus
 * @return
 */
static inline uint32_t compute_next_dwtime_uus(uint32_t last_send_time,
		uint32_t interval_uus) {
	return ((last_send_time + (interval_uus << 8)) & 0xFFFFFFFE);
}

/**
 * Find the next send time based on value of last send time and microsecond delay.
 * This function will have a residual value due to integer division
 * @param last_send_time
 * @param usec
 * @return The next send time (in 32-bit). Note that this will have residual errors
 */
static inline uint32_t compute_next_dwtime_usec(uint32_t last_send_time,
		uint32_t interval_usec) {
	return ((last_send_time + ((interval_usec * 1248UL) / 5UL)) & 0xFFFFFFFE);
}

//static inline uint64_t add_corrrection_factor_ts64(uint64_t ts64, uint64_t correction) {
//	return (ts64 + correction);
//}

/**
 * Use this function while embedding timestamps in beacon messages
 * @param send_time
 * @param overflow_correction
 * @return
 */
static inline uint64_t sendtime_to_expected_timestamp(uint32_t send_time,
		uint64_t overflow_correction) {
	return ((((uint64_t) (send_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY
			+ overflow_correction);
}

/**
 * Gets the next TDMA transmit slot approximately assuming the beacon message was sent at start of slot.
 *
 * @param rx_ts64
 * @param rx_slot		Slot number of last RX timestamp. The function assumes this lies in 1 to N_TDMA_SLOTS
 * @param desired_slot	Desired slot number. The function assumes this lies in 1 to N_TDMA_SLOTS
 * @return
 */
static inline uint32_t get_tdma_slot_time_from_rxts(uint64_t rx_ts64,
		int rx_slot, int desired_slot) {
	uint64_t deltaT;	// time between slots in DWT
	if (desired_slot > rx_slot) {
		deltaT = ((uint64_t) (desired_slot - rx_slot + 1) * TDMA_WIDTH_UUS)
				<< 16;
	} else {
		// the desired slot wraps around
		deltaT = ((uint64_t) (N_TDMA_SLOTS - rx_slot + desired_slot)
				* TDMA_WIDTH_UUS) << 16;
	}
	PRINTF("dT: " TS64_XSTR "\r\n", TS64_TO_TS32(deltaT, 1),
			TS64_TO_TS32(deltaT, 0));
	return ((uint32_t) ((rx_ts64 + deltaT) >> 8) & 0xFFFFFFFEUL);
}

static inline uint32_t get_tdma_slot_time_from_txts(
		uint32_t programmed_send_time, int tx_slot, int desired_slot) {
	if (desired_slot > tx_slot) {
		return ((programmed_send_time
				+ (((uint32_t) (desired_slot - tx_slot) * TDMA_WIDTH_UUS) << 8))
				& 0xFFFFFFFE);
	} else {
		return ((programmed_send_time
				+ (((uint32_t) (N_TDMA_SLOTS - tx_slot + desired_slot)
						* TDMA_WIDTH_UUS) << 8)) & 0xFFFFFFFE);
	}
}

/*
 * Think of the radio as a state machine. A successful operation leads
 * to a transition to the next state
 *
 * A REFERENCE NODE has 2 states:
 * 1. Transmitting
 * 2. Receiving from RELAY
 *
 * A RELAY NODE with 2 children has 5 states:
 * 1. Listening to REFERENCE
 * 2. Transmit beacon
 * 3. Respond to REFERENCE
 * 4. Listen to 1st CHILD
 * 5. Listen to 2nd CHILD
 *
 * A CHILD NODE has 2 states:
 * 1. Listening to RELAY
 * 2. Respond to RELAY
 */

enum reference_states {
	REF_TRANSMIT_BEACON, REF_RECIEVE_RESP, REF_INVALID_MESSAGE, N_REF_STATES,
};

enum relay_states {
	RLY_RECIEVE_BEACON,
	RLY_TRANSMIT_BEACON,
	RLY_TRANSMIT_RESP,
	RLY_LISTEN_CHILD1,
	RLY_LISTEN_CHILD2,
	RLY_INVALID_BEACON,
	N_RELAY_STATES,
};

enum child_states {
	CH_SEARCH_BEACON, CH_TRANSMIT_RESP, CH_INVALID_MESSAGE, N_CHILD_STATES,
};

/*
 * Variables that are better off being global for various reasons
 */

volatile bool rx_message_good = false;

volatile uint32_t rx_msg_no = 0;
volatile uint64_t local_rx_timestamp = 0;
volatile uint64_t local_tx_timestamp = 0;
volatile uint64_t reference_tx_timestamp = 0;
volatile uint64_t previous_rx_timestamp = 0;
volatile uint64_t global_offset = 0;

//volatile uint64_t local_time_correction = 0;	// correction factor to apply
//volatile uint64_t local_time = 0;				// used to keep dw time value
//volatile uint64_t last_local_time = 0;			// used to keep dw time value

volatile uint64_t tx_timestamp;

uint32_t programmed_send_time;	// used for storing 31 bit DW delayed TX time
uint32_t programmed_receive_time;	// used to store 31 bit DW delayed RX time

enum child_states system_state;

/*
 * Callback functions
 */

// callback type for all events
typedef void (*dwt_cb_t)(const dwt_cb_data_t *);

/**
 * Sent TX frame callback function.
 * @param data
 */
void tx_good_callback(const dwt_cb_data_t *data) {
	local_tx_timestamp = 0;
	dwt_readtxtimestamp((uint8 *) &local_tx_timestamp);
}

/**
 * RX good frame received callback function. This is automatically run on
 * calling dwt_isr() after callbacks are registered
 * It will read the RX timestamp and place it in the local_rx_timestamp variable
 * @param data
 */
void rx_good_callback(const dwt_cb_data_t *data) {
	// try parsing the message

	rx_message_good = false;
	local_rx_timestamp = 0;
	if (data->datalength <= BEACON_SIZE) {
		// copy data to buffer
		dwt_readrxdata((uint8 *) beacon_message, data->datalength, 0);
		// check the message belongs to parent and is of beacon type
		if ((beacon_message[beacon_TYPE] == BEACON_MSG_ID)
				&& beacon_message[beacon_ORIGIN] == REFERENCE_ID
				&& beacon_message[beacon_TARGET] == NODE_ID) {// TODO: add reference node ID based filtering here
			// get timestamp
			dwt_readrxtimestamp((uint8 *) &local_rx_timestamp);
			rx_message_good = true;
		}
	}
}

void rx_timeout_callback(const dwt_cb_data_t *data) {
	local_rx_timestamp = 0;
//	PRINTF("[%u] RX timeout\r\n", xTaskGetTickCount());
}

void rx_error_callback(const dwt_cb_data_t *data) {
	local_rx_timestamp = 0;
	rx_message_good = false;
	PRINTF("[%u] RX error\r\n", xTaskGetTickCount());
}

static void parse_good_beacon_message() {
	// need to recover message number, reference_tx_timestamp, our previous RX timestamp from beacon message
	reference_tx_timestamp = 0;
	previous_rx_timestamp = 0;

	memcpy((void *) &reference_tx_timestamp,
			(void *) &beacon_message[beacon_TX_TS], 8);
	memcpy((void *) &previous_rx_timestamp,
			(void *) &beacon_message[beacon_CHILD_RX_TS], 5);// RX timestamp is sent back by the reference node
	memcpy((void *) &global_offset, (void *) &beacon_message[beacon_GOFFSET],
			8);

//	PRINTF("[%u] received beacon %u: " TS64_XSTR "->" TS40_XSTR "\r\n", xTaskGetTickCount(), rx_msg_no, TS64_TO_TS32(reference_tx_timestamp,1), TS64_TO_TS32(reference_tx_timestamp,0), TS64_TO_TS32(local_rx_timestamp,1), TS64_TO_TS32(local_rx_timestamp,0));
}

/**
 * Simple case where we just have a REFERENCE and CHILD node.
 *
 * @param pvParamaeters
 */
void messenger_task(void *pvParameters) {

//	bool first_time_measure = true;
//	uint32_t status_reg;

	// wait for chip start
	xSemaphoreTake(semaphore[DW_MSG_OK_SEM], portMAX_DELAY);

	// all previous systems have now been stabilized
	// begin messaging scheme

	/*
	 * This section is for COMMON configurations that are valid regardless of node type
	 * and must be EXECUTED ONCE per DW power-on
	 */

	// Set callbacks. This applies to all cases
	dwt_setcallbacks((dwt_cb_t) tx_good_callback, (dwt_cb_t) rx_good_callback,
			(dwt_cb_t) rx_timeout_callback, (dwt_cb_t) rx_error_callback);
	dwt_configure(&channel_config);		// configure decawave channel parameters
	dwt_setrxantennadelay(RX_ANT_DLY);// antenna delay value for 64 MHz PRF in DW ticks
	dwt_settxantennadelay(TX_ANT_DLY);// antenna delay value for 64 MHz PRF in DW ticks

	/*
	 * This section contains NODE SPECIFIC initialization code that runs once per DW power-on
	 */

	// configure IRQ pin interrupt
	PORT_SetPinInterruptConfig(BOARD_DW1000_GPIO_PORT,
			BOARD_DW1000_GPIO_IRQ_PIN, kPORT_InterruptRisingEdge);
	EnableIRQ(PORTC_IRQn);
	GPIO_PinInit(BOARD_DW1000_GPIO, BOARD_DW1000_GPIO_IRQ_PIN,
			&(gpio_pin_config_t ) { kGPIO_DigitalInput, 0 });

	/*
	 * Radio state machine starts here
	 */
	system_state = CH_SEARCH_BEACON;	// starting state

	while (true) {

		/*
		 * Local time factor update mode
		 */
//		dwt_readsystime((uint8 *) &local_time);
//		while(local_time + local_time_correction < last_local_time) {
//			local_time_correction += (1ULL<<40);
//		}
//		last_local_time = local_time + local_time_correction;

		// begin child state machine
		switch (system_state) {

		case CH_SEARCH_BEACON:

			// continuously listen till something is heard

			// first clear old states
			// enable the right interrupts
			dwt_setrxtimeout(0);

			dwt_setinterrupt(
					DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL
							| DWT_INT_RFTO | DWT_INT_SFDT | DWT_INT_RXPTO,
					(uint8_t) 1);
			xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ semaphore events
			dwt_isr(); 	// clear flags

			dwt_rxenable(DWT_START_RX_IMMEDIATE);	// start listening

			while (true) {
				if (xSemaphoreTake(semaphore[DW_IRQ_SEM], 2000) != pdTRUE) {
					rx_message_good = false;
					PRINTF("[%u] DW no reception\r\n", xTaskGetTickCount());
				} else {

					break;
				}
			}

			dwt_isr(); // process state and read timestamps etc

			// disable the used interrupts
			dwt_setinterrupt(
					DWT_INT_RFCG | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL
							| DWT_INT_RFTO | DWT_INT_SFDT | DWT_INT_RXPTO,
					(uint8_t) 0);

			if (rx_message_good) {

				// currently we just need the message number. The rest can be processed after the TX interrupt
				memcpy((void *) &rx_msg_no,
						(void *) &beacon_message[beacon_NUMBER], 4);

				system_state = CH_TRANSMIT_RESP;
			} else {
				system_state = CH_SEARCH_BEACON;
			}

			break;	// end state machine SEARCH_BEACON processing

		case CH_TRANSMIT_RESP:

			// for a child node, use the immediate TX mode

			response_message[response_TYPE] = RESPONSE_MSG_ID;// type of message
			response_message[response_ORIGIN] = NODE_ID; 		// our node ID
			response_message[response_TARGET] = REFERENCE_ID;	// ID of parent
			memcpy((void *) &response_message[response_NUMBER],
					(void *) &rx_msg_no, sizeof(uint32));
			memset((void *) &response_message[response_CHECKSUM], 0, 2);

			dwt_writetxdata(RESPONSE_SIZE, (uint8 *) response_message, 0);
			dwt_writetxfctrl(RESPONSE_SIZE, 0, 1);

			dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 1);// enable transmit interrupt
			xSemaphoreTake(semaphore[DW_IRQ_SEM], 0); // clear previous IRQ semaphore events
			dwt_isr(); 	// clear flags

//			dwt_setdelayedtrxtime(programmed_send_time);	// delayed send doesn't seem to work this close

			dwt_starttx(DWT_START_TX_IMMEDIATE);

			if (xSemaphoreTake(semaphore[DW_IRQ_SEM], 1200) != pdTRUE) {
				PRINTF("[%u] Failed to send resp\r\n", xTaskGetTickCount());
				dwt_isr();		// run ISR function to complete callbacks etc
				dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);// disable frame sent interrupt
			} else {

				// message sent successfully
				dwt_isr();
				dwt_setinterrupt(DWT_INT_TFRS, (uint8_t) 0);

				parse_good_beacon_message();

				// process things here before moving to next round of listening
//				PRINTF("[%u] sent response %u: " TS64_XSTR"\r\n", xTaskGetTickCount(), rx_msg_no, TS64_TO_TS32(local_tx_timestamp,1), TS64_TO_TS32(local_tx_timestamp,0));
				xSemaphoreGive(semaphore[MSG_DISC_OK_SEM]);
			}

			system_state = CH_SEARCH_BEACON;		// listen for next beacon
			break;

		case CH_INVALID_MESSAGE:

			// this implies the wrong message was received
			// go back to beacon searching mode

			system_state = CH_SEARCH_BEACON;
			break;

		default:
			system_state = CH_SEARCH_BEACON;
		}

	} // end of while loop

	// If a discipline task exists, it would stop the state machine here
}

/******************************************************************************
 * end of MESSENGER section
 *****************************************************************************/
