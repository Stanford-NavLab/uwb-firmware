#ifndef APPLICATION_DEFINITIONS_H_
#define APPLICATION_DEFINITIONS_H_

#include "port.h"
#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* Definitions expected to be modified by end-user *********************************************
*******************************************************************************************************************/


#define UWB_LIST_SIZE		    127			//the maximum size of the UWB network
											//do not use a number larger than 80 if USING_64BIT_ADDR==1
											//do not use a number larger than 80 if USING_64BIT_ADDR==0
											//these are the largest UWB network sizes that the firmware will support.
											//USING_64BIT_ADDR==1: Limiting factor is the max length of the INF messages
											//USING_64BIT_ADDR==0: Limiting factor is the memory required to store TDMAInfo objects for each UWB

#define SET_TXRX_DELAY 			1           //when set to 1 - the DW1000 RX and TX delays are set to the TX_DELAY and RX_DELAY defines
#define TX_ANT_DELAY            16477
#define RX_ANT_DELAY            16477			//TODO put reasonable initial value? add explanation

#define USING_64BIT_ADDR 		0	 		//when set to 0 - the DecaRanging application will use 16-bit addresses

#define SPEED_OF_LIGHT      	299704644.539f //(299702547.0)     //  (299,792.458 km/s in vacuum) / (1.000293)  in m/s in air //TODO add explanation!


/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power

#define NUM_INST            1
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_42BIT          (0x000003FFFFFFFFFF) //stm32 microsecond timestamps are 42 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

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
    REPORT,
    SYNC,
    INF_MAX,
    FRAME_TYPE_NB
};

//TWO WAY RANGING function codes
#define RTLS_DEMO_MSG_RNG_INIT              (0x20)          // Ranging initiation message
#define RTLS_DEMO_MSG_TAG_POLL              (0x21)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x10)          // Anchor response to poll
#define RTLS_DEMO_MSG_TAG_FINAL             (0x29)          // Tag final massage back to Anchor (0x29 because of 5 byte timestamps needed for PC app)
#define RTLS_DEMO_MSG_INF_REG              	(0x13)          // TDMA coordination info packet
#define RTLS_DEMO_MSG_INF_INIT              (0x14)          // TDMA coordination info packet
#define RTLS_DEMO_MSG_INF_SUG              	(0x15)          // TDMA coordination info packet
#define RTLS_DEMO_MSG_INF_UPDATE            (0x16)          // TDMA coordination info packet
#define RTLS_DEMO_MSG_RNG_REPORT            (0x11)			// Report of calculated range to network
#define RTLS_DEMO_MSG_SYNC					(0x22)			// Inform other UWBs to sync their TDMA frame start times

//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    1				// FunctionCode(1)
#define ANCH_RESPONSE_MSG_LEN               1		 		// FunctionCode(1)
#define TAG_FINAL_MSG_LEN                   16              // FunctionCode(1), Poll_TxTime(5), Resp_RxTime(5), Final_TxTime(5)
#define RNG_INIT_MSG_LEN					1				// FunctionCode(1)
#define RNG_REPORT_MSG_LEN_SHORT		    9				// FunctionCode(1), time of flight (6), short address (2)
#define RNG_REPORT_MSG_LEN_LONG		    	15				// FunctionCode(1), time of flight (6), long address (8)
#define SYNC_MSG_LEN						8				// FunctionCode(1), framelength (1), time since frame start (6)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define BROADCAST_ADDRESS           0xFFFF					//address for broadcasting messages to all UWBs the network

#define STANDARD_FRAME_SIZE         127
#define EXTENDED_FRAME_SIZE			1023

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
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-FRAME_CRC) //127 - 21 - 2 = 104
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-FRAME_CRC) //127 - 9 - 2 = 116
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-FRAME_CRC) //127 - 15 - 2 = 110
#define MAX_EXTENDED_USER_PAYLOAD_STRING_LL     (EXTENDED_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-FRAME_CRC) //1023 - 21 - 2 = 100
#define MAX_EXTENDED_USER_PAYLOAD_STRING_SS     (EXTENDED_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-FRAME_CRC) //1023 - 9 - 2 = 112
#define MAX_EXTENDED_USER_PAYLOAD_STRING_LS     (EXTENDED_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-FRAME_CRC) //1023 - 15 - 2 = 1006

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

// Total frame lengths.
#if (USING_64BIT_ADDR == 1)
    #define RNG_INIT_FRAME_LEN_BYTES (RNG_INIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC)
	#define REPORT_FRAME_LEN_BYTES (RNG_REPORT_MSG_LEN_LONG + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
	#define SYNC_FRAME_LEN_BYTES (SYNC_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
	#define INF_FRAME_LEN_BYTES_MAX (11 + (ADDR_BYTE_SIZE_L + 4)*UWB_LIST_SIZE + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
#else
    #define RNG_INIT_FRAME_LEN_BYTES (RNG_INIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
    #define POLL_FRAME_LEN_BYTES (TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define RESP_FRAME_LEN_BYTES (ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
    #define FINAL_FRAME_LEN_BYTES (TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
	#define REPORT_FRAME_LEN_BYTES (RNG_REPORT_MSG_LEN_SHORT + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
	#define SYNC_FRAME_LEN_BYTES (SYNC_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC)
	#define INF_FRAME_LEN_BYTES_MAX (11 + (ADDR_BYTE_SIZE_S + 4)*UWB_LIST_SIZE + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC)
#endif

#define BLINK_FRAME_CONTROL_BYTES       (1)
#define BLINK_FRAME_SEQ_NUM_BYTES       (1)
#define BLINK_FRAME_CRC					(FRAME_CRC)
#define BLINK_FRAME_SOURCE_ADDRESS      (ADDR_BYTE_SIZE_L)
#define BLINK_FRAME_CTRLP				(BLINK_FRAME_CONTROL_BYTES + BLINK_FRAME_SEQ_NUM_BYTES) //2
#define BLINK_FRAME_CRTL_AND_ADDRESS    (BLINK_FRAME_SOURCE_ADDRESS + BLINK_FRAME_CTRLP) //10 bytes
#define BLINK_FRAME_LEN_BYTES           (BLINK_FRAME_CRTL_AND_ADDRESS + BLINK_FRAME_CRC)

//UWB_LIST types
#define UWB_LIST_SELF					0 //entry for self in list
#define UWB_LIST_NEIGHBOR				1 //uwb in list that is active, in range, and are slotted to range with
#define UWB_LIST_HIDDEN					2 //uwb in list that is active, out of range, and a neighbor is slotted to range with
#define UWB_LIST_TWICE_HIDDEN			3 //uwb in list that is active, out of range, and a hidden neighbor is slotted to range with
#define UWB_LIST_INACTIVE               4 //uwb in list that is not active (could have previously been neighbor, hidden, or twice hidden)

#define BLINK_SLEEP_DELAY				0  //ms //how long the tag should sleep after blinking
#define POLL_SLEEP_DELAY				25 //ms //how long the tag should sleep after ranging

#define INST_DONE_WAIT_FOR_NEXT_EVENT   	1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        		//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               	0   //this signifies that the instance is still processing the current event

// Function code byte offset (valid for all message types).
#define FCODE                               0               // Function code is 1st byte of messageData

//INF message byte offsets
#define TDMA_TSFS                           1				// offset to put time since TDMA frame start in the INF message
#define TDMA_NUMN							7				// offset to put the number of this UWB's neighbors in the INF message
#define TDMA_NUMH							8				// offset to put the number of this UWB's hidden neighbors in the INF message
#define TDMA_FRAMELENGTH                    9				// offset to put this UWB's TDMA framelength in the INF message
#define TDMA_NUMS							10				// offset to put the number of this UWB's TDMA slot assignments in the INF message

// Final message byte offsets.
#define PTXT                                1
#define RRXT                                6
#define FTXT                                11


// Range report byte offsets.
#define REPORT_TOF                          1               // Offset to put ToF values in the report message.			  (6 bytes)
#define REPORT_ADDR							7				// Offset to put address of other UWB involved in the range report (2 [short address] or 8 [long address] bytes)

//Sync
#define SYNC_FRAMELENGTH					1				// offset to put the framelength in the sync message
#define SYNC_TSFS							2				// offset to put the time since TDMA frame start in the sync message

// Response delays types present in ranging init message.
enum
{
    RESP_DLY_POLL = 0,
    RESP_DLY_ANCH_RESP,
    RESP_DLY_FINAL,
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


#define RX_TO_CB_DLY_US 			50
#define RX_CB_TO_TX_CMD_DLY_US		220
#define TX_CMD_TO_TX_CB_DLY_US		90	//doesn't include preamble, sfd, phr, and data. saw 81 to 87
#define MIN_DELAYED_TX_DLY_US   	90	//doesn't include preamble and sfd. //80 is the minimum delay for delayed tx. anything shorter will cause failure


#define SLOT_START_BUFFER_US				4000				  //set based on experimentally observed frame sync errors
#define SLOT_BUFFER_EXP_TO_POLL_CMD_US		3*UWB_LIST_SIZE + 108 //found experimentally
#define MEASURED_SLOT_DURATIONS_US			(float)(1.7814f*(float)(UWB_LIST_SIZE)*(float)(UWB_LIST_SIZE) + 29.39f*(float)(UWB_LIST_SIZE) + 1848.5f)	//found experimentally
#define LCD_ENABLE_BUFFER_US				5000
#define SLOT_END_BUFFER_US					0			          //increase if all messages do not fit into a slot
#define BLINK_RX_CB_TO_RESP_TX_CMD_DLY_US	1450				  //found experimentally



#define MAX(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })


#define BLINK_PERIOD_MS							900     //time to wait between sending blink messages
#define BLINK_PERIOD_RAND_MS					200


#define RANGE_INIT_RAND_US						1000


#define RX_CHECK_ON_PERIOD_MS					200

// Reception start-up time, in symbols.
#define RX_START_UP_SY 16

//TDMA defines
#define MIN_FRAMELENGTH							4 		//minimum size by TDMA E-ASAP

typedef uint64_t        uint64 ;
typedef int64_t         int64 ;


// S1 swtich setting definitions
#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)


typedef enum instanceModes{DISCOVERY, TAG, ANCHOR, NUM_MODES} INST_MODE;

typedef enum discovery_modes
{
	WAIT_INF_REG,
	COLLECT_INF_REG,
	WAIT_INF_INIT,
	WAIT_RNG_INIT,
	WAIT_SEND_SUG,
	SEND_SUG,
	EXIT
}
DISCOVERY_MODE;


//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above

typedef enum inst_states
{
    TA_INIT,                    //0
    TA_TXINF_WAIT_SEND,			//1
    TA_TXPOLL_WAIT_SEND,        //2
    TA_TXFINAL_WAIT_SEND,       //3
    TA_TXRESPONSE_WAIT_SEND,
    TA_TX_WAIT_CONF,
    TA_RXE_WAIT,
    TA_RX_WAIT_DATA,
    TA_SLEEP_DONE,
    TA_TXBLINK_WAIT_SEND,
    TA_TXRANGINGINIT_WAIT_SEND,
    TA_TX_SELECT,
    TA_TXREPORT_WAIT_SEND,
    TA_TXSUG_WAIT_SEND
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
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  21-124 (application data and any user payload)
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

///// extended message definitions

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_EXTENDED_USER_PAYLOAD_STRING_LL] ; //  21-1020 (application data and any user payload)
    uint8 fcs[2] ;                              	//  1021-1022  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_ext_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_EXTENDED_USER_PAYLOAD_STRING_SS] ; //  09-1020 (application data and any user payload)
    uint8 fcs[2] ;                              	//  1021-1022  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_ext_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_EXTENDED_USER_PAYLOAD_STRING_LS] ; //  15-1020 (application data and any user payload)
    uint8 fcs[2] ;                              	//  1021-1022  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_ext_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_EXTENDED_USER_PAYLOAD_STRING_LS] ; //  15-1020 (application data and any user payload)
    uint8 fcs[2] ;                              	//  1021-1022  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_ext_msg_dssl ;



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

#define MAX_EVENT_NUMBER (8)
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
			uint8   frame[EXTENDED_FRAME_SIZE];
			srd_ext_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_ext_msg_dssl rxmsg_sl ;
			srd_ext_msg_dlss rxmsg_ls ;
			srd_ext_msg_dsss rxmsg_ss ; //16 bit addresses
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





#endif
