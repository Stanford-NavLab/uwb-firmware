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
#ifndef INSTANCE_H_
#define INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "application_definitions.h"
#include "port.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "tdma_handler.h"


typedef struct
{
    INST_MODE mode;				        //instance mode (tag or anchor)
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
    uint32 storedPreLen;           //precomputed conversion of preamble and sfd
    uint64 storePreLen_us;		   //precomputed conversion of preamble and sfd in microseconds

	//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)
	srd_msg_dlsl rng_initmsg ;	// ranging init message (destination long, source long)
    srd_ext_msg_dlsl msg; // simple 802.15.4 frame structure (used for tx message) - using long addresses
	srd_ext_msg_dssl inf_msg;         	  // extended inf message containing frame lengths and slot assignments
    srd_ext_msg_dssl report_msg;          // extended report message containing the calculated range
#else
	srd_msg_dlss rng_initmsg ;  // ranging init message (destination long, source short)
    srd_ext_msg_dsss msg; // simple 802.15.4 frame structure (used for tx message) - using short addresses
    srd_ext_msg_dsss inf_msg;         	  // extended inf message containing frame lengths and slot assignments
    srd_ext_msg_dsss report_msg;			// extended report message containing the calculated range
#endif
	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

	//Tag function address/message configuration
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  uwbShortAdd;		    // UWB's short address (16-bit) used when USING_64BIT_ADDR == 0
    uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion
	uint16  panID ;					// panid used in the frames

    uint8 addrByteSize;             // The bytelength used for addresses. 

    uint32 resp_dly[RESP_DLY_NB];

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
    
    int64 tof[UWB_LIST_SIZE] ;
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

    uint8 newRangeUWBIndex; //index for most recent ranging exchange
    int newRange;
    uint64 newRangeAncAddress; //anchor address for most recent ranging exchange
    uint64 newRangeTagAddress; //tag address for most recent ranging exchange

    double idistance[UWB_LIST_SIZE];
    double idistanceraw[UWB_LIST_SIZE];
    
    //if set to 1 then it means that DW1000 is in DEEP_SLEEP
    //so the ranging has finished and micro can output on USB/LCD
    //if sending data to LCD during ranging this limits the speed of ranging
    uint8 canPrintInfo ;

	uint8 uwbToRangeWith;	//it is the index of the uwbList array which contains the address of the UWB we are ranging with
    uint8 uwbListLen ;

	uint8 uwbList[UWB_LIST_SIZE][8];		//index 0 reserved for self, rest for other tracked uwbs
	uint8 uwbListType[UWB_LIST_SIZE];       //UWB_LIST_SELF, UWB_LIST_NEIGHBOR, UWB_LIST_HIDDEN, UWB_LIST_INACTIVE

	uint8 uwbNumActive[UWB_LIST_SIZE];		//number of TAGs each tracked ANCHOR is actively ranging with. //TODO remove?

    // keep track of when final messages so we can drop uwbs that we haven't communicated with in a while
    uint32 lastCommTimeStamp[UWB_LIST_SIZE];
    uint8 uwbTimeout[UWB_LIST_SIZE] ;

    uint32 lastRangeTimeStamp[UWB_LIST_SIZE];

    uint8 time_till_next_reported[UWB_LIST_SIZE]; //used to keep track of whether we reported the RX_ACCEPT node. 0 if no, 1 if yes.

    uint32 blink_start;
    uint32 range_start;
    uint8 blink0range1;

    uint32 blink_duration;      //expected duration of a blink/response exchange  
    uint32 range_duration;      //expected duration of a range/response/final exchange

	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;
	uint8 monitor;
	uint32 timeofTx;
	uint64 timeofRxCallback;
	uint64 timeofRxCallback_dwtime;
	uint8 smartPowerEn;

	uint32 currentStateStartTime;
	INST_STATES lastState;

	uint32 rxCheckOnTime;

	uint32 buildFrameTime;

	uint8 ranging;

	uint64 testTimer;
	uint16 timerCounter;


} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
int reportTOF(instance_data_t *inst, uint8 uwb_index);
// clear the status/ranging data 
void instanceclearcounts(void) ;
void instclearuwblist(void);
// void instsetuwbtorangewith(int uwbID);
int instaddactivateuwbinlist(instance_data_t *inst, uint8 *uwbAddr);
int instcheckactiveuwbinlist(instance_data_t *inst, uint8 *uwbAddr);
int instfindfirstactiveuwbinlist(instance_data_t *inst, uint8 startindex);
int instfindnumactiveuwbinlist(instance_data_t *inst);
int instfindnumactiveneighbors(instance_data_t *inst);
int instfindnumactivehidden(instance_data_t *inst);
int instgetuwblistindex(instance_data_t *inst, uint8 *uwbAddr, uint8 addrByteSize);


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
int instance_init_s();
int tdma_init_s();

// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config) ;  

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
void inst_processtxrxtimeout(instance_data_t *inst);

int instancesendpacket(uint16 length, uint8 txmode, uint32 dtime);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, struct TDMAHandler *tdma_handler, int message);

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

uint32 instance_getmessageduration_us(int data_length_bytes);

// set/get the instance roles e.g. Tag/Anchor/Listener
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for MP)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime);
// double instance_get_adist(void);
double instance_get_idist(uint8 uwb_index);
double instance_get_idistraw(uint8 uwb_index);
// int instance_get_lcount(void);

uint64 instance_get_addr(void); //get own address (8 bytes)
uint64 instance_get_uwbaddr(uint8 uwb_index); //get uwb address (8 bytes)
// uint64 instance_get_anchaddr(void); //get anchor address (that sent the ToF)

uint64 instancenewrangeancadd(void);
uint64 instancenewrangetagadd(void);
int instanceisranging(void);
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

const char* get_instanceModes_string(enum instanceModes mode);
const char* get_discovery_modes_string(enum discovery_modes mode);

instance_data_t* instance_get_local_structure_ptr(unsigned int x);

uint32 get_dt32(uint32 t1, uint32 t2);
uint32 timestamp_add32(uint32 timestamp, uint32 duration);
uint32 timestamp_subtract32(uint32 timestamp, uint32 duration);
uint64 get_dt64(uint64 t1, uint64 t2);
uint64 timestamp_add64(uint64 timestamp, uint64 duration);
uint64 timestamp_subtract64(uint64 timestamp, uint64 duration);
uint16 get_dt16(uint16 t1, uint16 t2);

uint16 address64to16(uint8 *address);

//void send_statetousb(instance_data_t *inst);
void send_statetousb(instance_data_t *inst, struct TDMAHandler *tdma_handle);
void send_rxmsgtousb(char *data);
void send_txmsgtousb(char *data);
char* get_msg_fcode_string(int fcode);


#ifdef __cplusplus
}
#endif

#endif
