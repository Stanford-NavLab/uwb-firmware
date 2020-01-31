/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "port.h"
#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on Decaranging EVK1000 application features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (1) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.


#define USING_64BIT_ADDR (1) //when set to 0 - the DecaRanging application will use 16-bit addresses

//! callback events
#define DWT_SIG_RX_NOERR            0
#define DWT_SIG_TX_DONE             1       // Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE          6       // ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT       11      // Got preamble detection timeout (no preamble detected)

#define DWT_SIG_TX_PENDING          12      // TX is pending
#define DWT_SIG_TX_ERROR            13      // TX failed
#define DWT_SIG_RX_PENDING          14      // RX has been re-enabled
#define DWT_SIG_DW_IDLE             15      // DW radio is in IDLE (no TX or RX pending)

#define SIG_RX_UNKNOWN			99		// Received an unknown frame

// Existing frames type in ranging process.
enum
{
    BLINK = 0,
    RNG_INIT,
    POLL,
    RESP,
    FINAL,
    FRAME_TYPE_NB
};

//TWO WAY RANGING function codes
#define RTLS_DEMO_MSG_RNG_INIT              (0x20)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)

//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    1				// FunctionCode(1),
#define ANCH_RESPONSE_MSG_LEN               9               // FunctionCode(1), RespOption (1), OptionParam(2), Measured_TOF_Time(5)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define RANGINGINIT_MSG_LEN					7				// FunctionCode(1), Tag Address (2), Response Time (2) * 2

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

// Total frame lengths.
#if (USING_64BIT_ADDR == 1)
    #define RNG_INIT_FRAME_LEN_BYTES (RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
#else
    #define RNG_INIT_FRAME_LEN_BYTES (RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
#endif

#define BLINK_FRAME_CONTROL_BYTES       (1)
#define BLINK_FRAME_SEQ_NUM_BYTES       (1)
#define BLINK_FRAME_CRC					(FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS      (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES           (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)

#define TAG_LIST_SIZE				(1)	//anchor will range with 1st Tag it gets blink from

#define BLINK_SLEEP_DELAY					1000 //ms
#define POLL_SLEEP_DELAY					500 //ms
//#define POLL_SLEEP_DELAY					50 //ms	//NOTE 200 gives 5 Hz range period


#define IMMEDIATE_RESPONSE (1)

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        //which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               0   //this signifies that the instance is still processing the current event

// Function code byte offset (valid for all message types).
#define FCODE                               0               // Function code is 1st byte of messageData
// Final message byte offsets.
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11
// Length of ToF value in report message. Can be used as offset to put up to
// 4 ToF values in the report message.
#define TOFR                                4
// Anchor response byte offsets.
#define RES_R1                              1               // Response option octet 0x02 (1),
#define RES_R2                              2               // Response option parameter 0x00 (1) - used to notify Tag that the report is coming
#define RES_R3                              3               // Response option parameter 0x00 (1),
// Ranging init message byte offsets. Composed of tag short address, anchor
// response delay and tag response delay.
#define RNG_INIT_TAG_SHORT_ADDR_LO 1
#define RNG_INIT_TAG_SHORT_ADDR_HI 2
#define RNG_INIT_ANC_RESP_DLY_LO 3
#define RNG_INIT_ANC_RESP_DLY_HI 4
#define RNG_INIT_TAG_RESP_DLY_LO 5
#define RNG_INIT_TAG_RESP_DLY_HI 6

// Response delay values coded in ranging init message.
// This is a bitfield composed of:
//   - bits 0 to 14: value
//   - bit 15: unit
#define RESP_DLY_VAL_SHIFT 0
#define RESP_DLY_VAL_MASK 0x7FFF
#define RESP_DLY_UNIT_SHIFT 15
#define RESP_DLY_UNIT_MASK 0x8000

// Response time possible units: microseconds or milliseconds.
#define RESP_DLY_UNIT_US 0
#define RESP_DLY_UNIT_MS 1

// Response delays types present in ranging init message.
enum
{
    RESP_DLY_ANC = 0,
    RESP_DLY_TAG,
    RESP_DLY_NB
};

// Convert microseconds to symbols, float version.
// param  x  value in microseconds
// return  value in symbols.
#define US_TO_SY(x) ((x) / 1.0256)

// Convert microseconds to symbols, integer version.
// param  x  value in microseconds
// return  value in symbols.
// /!\ Due to the the multiplication by 10000, be careful about potential
// values and type for x to avoid overflows.
#define US_TO_SY_INT(x) (((x) * 10000) / 10256)

// Minimum delay between reception and following transmission.
#define RX_TO_TX_TIME_US 150
#define RXTOTXTIME          ((int)(150.0 / 1.0256)) //e.g. Poll RX to Response TX time

// Default anchor turn-around time: has to be RX_TO_TX_TIME_US when using
// immediate response, cannot be less than 170 us when not.
#define ANC_TURN_AROUND_TIME_US RX_TO_TX_TIME_US
#if (IMMEDIATE_RESPONSE == 1) && (ANC_TURN_AROUND_TIME_US != RX_TO_TX_TIME_US)
    #error "When using immediate response, anchor turn-around time has to be equal to RX to TX time!"
#endif
// Default tag turn-around time: cannot be less than 300 us. Defined as 500 us
// so that the tag is not transmitting more than one frame by millisecond (for
// power management purpose).
#define TAG_TURN_AROUND_TIME_US 500

// "Long" response delays value. Over this limit, special processes must be
// applied.
#define LONG_RESP_DLY_LIMIT_US 25000

// Delay between blink reception and ranging init message. This is the same for
// all modes.
#define RNG_INIT_REPLY_DLY_MS (150)

// Reception start-up time, in symbols.
#define RX_START_UP_SY 16

typedef uint64_t        uint64 ;

typedef int64_t         int64 ;


typedef enum instanceModes{LISTENER, TAG, ANCHOR, TAG_TDOA, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,    //4
    TA_TX_WAIT_CONF,            //6

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE,              //9
    TA_TXBLINK_WAIT_SEND,       //10
    TA_TXRANGINGINIT_WAIT_SEND  //11
} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8 frameCtrl;                         		//  frame control bytes 00
    uint8 seqNum;                               	//  sequence_number 01
    uint8 tagID[BLINK_FRAME_SOURCE_ADDRESS];        //  02-09 64 bit address
    uint8 fcs[2] ;                              	//  10-11  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} iso_IEEE_EUI64_blink_msg ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;


/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define MAX_EVENT_NUMBER (4)
//NOTE: Accumulators don't need to be stored as part of the event structure as when reading them only one RX event can happen...
//the receiver is singly buffered and will stop after a frame is received

typedef struct
{
	uint8  type;			// event type
	uint8  typeSave;		// holds the event type - does not clear (used to show what event has been processed)
	uint8  typePend;	    // set if there is a pending event (i.e. DW is not in IDLE (TX/RX pending)
	uint16 rxLength ;

	uint64 timeStamp ;		// last timestamp (Tx or Rx)

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
			iso_IEEE_EUI64_blink_msg rxblinkmsg;
	}msgu;

}event_data_t ;

#define RTD_MED_SZ          8      // buffer size for mean of 8

typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)
    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state

	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
	uint16			txAntennaDelay ; //DW1000 TX antenna delay
	uint16			rxAntennaDelay ; //DW1000 RX antenna delay
	uint8 antennaDelayChanged;
	// "MAC" features
    uint8 frameFilteringEnabled ;	//frame filtering is enabled

    // Is sleeping between frames enabled?
    uint8 sleepingEabled; //Ranging Init message can tell tag to stay in IDLE between ranging exchanges

	//timeouts and delays
	int tagSleepTime_ms; //in milliseconds
	int tagBlinkSleepTime_ms;
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64 rnginitReplyDelay ;
	uint64 finalReplyDelay ;
	uint64 responseReplyDelay ;
	int finalReplyDelay_ms ;

	// xx_sy the units are 1.0256 us
	uint32 txToRxDelayAnc_sy ;    // this is the delay used after sending a response and turning on the receiver to receive final
	uint32 txToRxDelayTag_sy ;    // this is the delay used after sending a poll and turning on the receiver to receive response
	int rnginitW4Rdelay_sy ;	// this is the delay used after sending a blink and turning on the receiver to receive the ranging init message

	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)
	int fwtoTimeB_sy ;	//this is the ranging init message duration

	uint32 delayedReplyTime;		// delayed reply time of delayed TX message - high 32 bits

    // Pre-computed frame lengths for frames involved in the ranging process,
    // in microseconds.
    uint32 frameLengths_us[FRAME_TYPE_NB];

	//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)
	srd_msg_dlsl rng_initmsg ;	// ranging init message (destination long, source long)
    srd_msg_dlsl msg ;			// simple 802.15.4 frame structure (used for tx message) - using long addresses
#else
	srd_msg_dlss rng_initmsg ;  // ranging init message (destination long, source short)
    srd_msg_dsss msg ;			// simple 802.15.4 frame structure (used for tx message) - using short addresses
#endif
	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

	//Tag function address/message configuration
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  tagShortAdd ;		    // Tag's short address (16-bit) used when USING_64BIT_ADDR == 0
    uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion
	uint16  panID ;					// panid used in the frames

    uint8 relpyAddress[8] ;         // address of the anchor the tag is ranging with

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64 txTimeStamp ;		   // last tx timestamp
		uint64 tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;
	uint64 anchorRespRxTime ;	    // receive time of response message
	uint64 tagPollRxTime ;          // receive time of poll message

	//application control parameters
    uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
	uint8   goToSleep;			// if set the instance will go to sleep before sending the blink/poll message
    uint8	instanceTimerEnabled;		// enable/start a timer
    uint32	instanceTimerTime;			// e.g. this timer is used to timeout Tag when in deep sleep so it can send the next poll message
    uint32	instanceTimerTimeSaved;
	uint8	gotTO;					// got timeout event


    //diagnostic counters/data, results and logging
    int32 tof32 ;
    int64 tof ;
    double clockOffset ;

    //counts for debug
	int txmsgcount;
	int	rxmsgcount;
	int lateTX;
	int lateRX;

    double adist[RTD_MED_SZ] ;
    double adist4[4] ;
    double longTermRangeSum ;
    int longTermRangeCount ;
    int tofIndex ;
    int tofCount ;

    double idistance ; // instantaneous distance
    int newRange;
    int newRangeAncAddress; //last 4 bytes of anchor address
    int newRangeTagAddress; //last 4 bytes of tag address

    //if set to 1 then it means that DW1000 is in DEEP_SLEEP
    //so the ranging has finished and micro can output on USB/LCD
    //if sending data to LCD during ranging this limits the speed of ranging
    uint8 canPrintInfo ;

	uint8 tagToRangeWith;	//it is the index of the tagList array which contains the address of the Tag we are ranging with
    uint8 tagListLen ;
    uint8 anchorListIndex ;
	uint8 tagList[TAG_LIST_SIZE][8];


	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;
	uint8 monitor;
	uint32 timeofTx ;
	uint8 smartPowerEn;

} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
int reportTOF(instance_data_t *inst);
// clear the status/ranging data 
void instanceclearcounts(void) ;
void instcleartaglist(void);
void instsettagtorangewith(int tagID);
int instaddtagtolist(instance_data_t *inst, uint8 *tagAddr);

void instance_readaccumulatordata(void);
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------
void setupmacframedata(instance_data_t *inst, int fcode);

// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(void);
int instance_init_s(int mode);

// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;  

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
void inst_processrxtimeout(instance_data_t *inst);

int instancesendpacket(uint16 length, uint8 txmode, uint32 dtime);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, int message);

// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxerrorcallback(const dwt_cb_data_t *rxd);
void instance_rxtimeoutcallback(const dwt_cb_data_t *rxd);
void instance_rxgoodcallback(const dwt_cb_data_t *rxd);
void instance_txcallback(const dwt_cb_data_t *txd);

// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesettagsleepdelay(int rangingsleep, int blinkingsleep);
void instancesetreplydelay(int datalength);

// Pre-compute frame lengths, timeouts and delays needed in ranging process.
// /!\ This function assumes that there is no user payload in the frame.
void instance_init_timings(void);

// set/get the instance roles e.g. Tag/Anchor/Listener
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for MP)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
double instance_get_adist(void);
double instance_get_idist(void);
double instance_get_idistraw(void);
int instance_get_lcount(void);

uint64 instance_get_addr(void); //get own address (8 bytes)
uint64 instance_get_tagaddr(void); //get tag address (8 bytes)
uint64 instance_get_anchaddr(void); //get anchor address (that sent the ToF)

int instancenewrangeancadd(void);
int instancenewrangetagadd(void);
int instancenewrange(void);
int instancesleeping(void);
int instanceanchorwaiting(void);

int instance_get_rxf(void);
int instance_get_txf(void); //get number of Txed frames

int instance_get_txl(void) ;
int instance_get_rxl(void) ;

uint32 convertmicrosectodevicetimeu32 (double microsecu);
uint64 convertmicrosectodevicetimeu (double microsecu);
double convertdevicetimetosec(int32 dt);
double convertdevicetimetosec8(uint8* dt);

int testapprun_af(instance_data_t *inst, int message);
int testapprun_tf(instance_data_t *inst, int message);

int instance_peekevent(void);

void instance_putevent(event_data_t newevent);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

// configure the antenna delays
void instanceconfigantennadelays(uint16 tx, uint16 rx);
void instancesetantennadelays(void);
uint16 instancetxantdly(void);
uint16 instancerxantdly(void);

int instance_starttxtest(int framePeriod);


instance_data_t* instance_get_local_structure_ptr(unsigned int x);

#ifdef __cplusplus
}
#endif

#endif
