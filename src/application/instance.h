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
	uint16 			defaultAntennaDelay; //reasonable estimate of DW1000 TX/RX antenna delay
	
    uint8 antennaDelayChanged;
	// "MAC" features
    uint8 frameFilteringEnabled ;	//frame filtering is enabled

    bool lcdEnabled;
    bool canPrintInfo;	  //only print to usb and/or LCD display when TRUE
    bool canPrintLCD;
    bool canPrintUSB;

	//timeouts and delays
	//this is the delay used for the delayed transmit (when sending the ranging init, response, and final messages)
	uint64 rnginitReplyDelay;
	uint64 finalReplyDelay;
	uint64 finalReplyDelay_us;
	uint64 respReplyDelay;

	//Receive Frame Wait Timeout Periods, units are 1.0256us (aus stands for near microsecond)
	uint16 durationPollTimeout_nus;		//rx timeout duration after tx poll

	uint32 durationBlinkTxDoneTimeout_ms;   	//tx done timeout after tx blink
	uint32 durationRngInitTxDoneTimeout_ms;   	//tx done timeout after tx rng_init
	uint32 durationPollTxDoneTimeout_ms;   		//tx done timeout after tx poll
	uint32 durationRespTxDoneTimeout_ms;   		//tx done timeout after tx resp
	uint32 durationFinalTxDoneTimeout_ms;   	//tx done timeout after tx final
	uint32 durationReportTxDoneTimeout_ms;   	//tx done timeout after tx report
	uint32 durationSyncTxDoneTimeout_ms;   		//tx done timeout after tx sync

	uint32 durationUwbCommTimeout_ms;			//how long to wait before changing UWB_LIST_TYPE if we haven't communicated with or received an information about a UWB in a while

	uint32 durationWaitRangeInit_ms;			//discovery mode WAIT_RNG_INIT duration

	uint64 durationSlotMax_us;		// longest anticipated time required for a slot based on UWB_LIST_SIZE and S1 switch settings

	uint32 delayedReplyTime;		// delayed reply time of delayed TX message - high 32 bits

    // Pre-computed frame lengths for frames involved in the ranging process,
    // in microseconds.
    uint32 frameLengths_us[FRAME_TYPE_NB];
    uint32 storedPreLen;           //precomputed conversion of preamble and sfd in DW1000 time units
    uint64 storedPreLen_us;		   //precomputed conversion of preamble and sfd in microseconds

//message structures used for transmitted messages
#if (USING_64BIT_ADDR == 1)
	srd_msg_dlsl rng_initmsg ;				// ranging init message (destination long, source long)
    srd_ext_msg_dlsl msg; 					// simple 802.15.4 frame structure (used for tx message) - using long addresses
	srd_ext_msg_dssl inf_msg;         	  	// extended inf message containing frame lengths and slot assignments
    srd_ext_msg_dssl report_msg;          	// extended report message containing the calculated range
    srd_ext_msg_dssl sync_msg;		      	// extended message indicating the need to resync TDMA frame
#else
	srd_msg_dlss rng_initmsg ;  			// ranging init message (destination long, source short)
    srd_ext_msg_dsss msg; 				  	// simple 802.15.4 frame structure (used for tx message) - using short addresses
    srd_ext_msg_dsss inf_msg;         	  	// extended inf message containing frame lengths and slot assignments
    srd_ext_msg_dsss report_msg;		  	// extended report message containing the calculated range
    srd_ext_msg_dsss sync_msg;		      	// extended message indicating the need to resync TDMA frame
#endif

	iso_IEEE_EUI64_blink_msg blinkmsg ; // frame structure (used for tx blink message)

	//Tag function address/message configuration
	uint8   eui64[8];				// devices EUI 64-bit address
	uint16  uwbShortAdd;		    // UWB's short address (16-bit) used when USING_64BIT_ADDR == 0
    uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmittion
	uint16  panID ;					// panid used in the frames

    uint8 addrByteSize;             // The bytelength used for addresses. 

    uint32 resp_dly_us[RESP_DLY_NB];

	//64 bit timestamps
	uint64 anchorRespTxTime ;		// anchor's reponse tx timestamp
	uint64 anchorRespRxTime ;	    // receive time of response message
	uint64 tagPollRxTime ;          // receive time of poll message
	uint64 dwt_final_rx;			// receive time of the final message

	//application control parameters
    uint8	wait4ack ;					// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion
	uint8	gotTO;						// got timeout event

    //diagnostic counters/data, results and logging
    int64 tof[UWB_LIST_SIZE] ;
    double clockOffset ;

    //counts for debug
	int txmsgcount;
	int	rxmsgcount;
	int lateTX;
	int lateRX;

    uint8 newRangeUWBIndex; //index for most recent ranging exchange
    int newRange;
    uint64 newRangeAncAddress; //anchor address for most recent ranging exchange
    uint64 newRangeTagAddress; //tag address for most recent ranging exchange

    double idistance[UWB_LIST_SIZE];
    double idistanceraw[UWB_LIST_SIZE];

	uint8 uwbToRangeWith;	//it is the index of the uwbList array which contains the address of the UWB we are ranging with
    uint8 uwbListLen ;
	uint8 uwbList[UWB_LIST_SIZE][8];		//index 0 reserved for self, rest for other tracked uwbs

	uint32 timeofTx;						//used to calculate tx done callback timeouts
	uint32 txDoneTimeoutDuration;			//duration used for tx done callback timeouts. (set differently for each tx message)

	bool tx_poll;							//was the last tx message a POLL message?
	bool tx_anch_resp;						//was the last tx message an ANCH_RESP message?
	bool tx_final;							//was the last tx message a FINAL message?

	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;

	uint8 smartPowerEn;

	uint32 rxCheckOnTime;

	instanceConfig_t chConfig[8];

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
int instaddactivateuwbinlist(instance_data_t *inst, uint8 *uwbAddr);
int instcheckactiveuwbinlist(instance_data_t *inst, uint8 *uwbAddr);
int instfindfirstactiveuwbinlist(instance_data_t *inst, uint8 startindex);
int instfindnumactiveuwbinlist(instance_data_t *inst);
int instfindnumneighbors(instance_data_t *inst);
int instfindnumhidden(instance_data_t *inst);
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
int tdma_init_s(uint64 slot_duration);
int decarangingmode(uint8 mode_switch);

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
void instance_irqstuckcallback();

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
const char* get_inst_states_string(enum inst_states state);

instance_data_t* instance_get_local_structure_ptr(unsigned int x);
struct TDMAHandler* tdma_get_local_structure_ptr();

uint32 get_dt32(uint32 t1, uint32 t2);
uint32 timestamp_add32(uint32 timestamp, uint32 duration);
uint32 timestamp_subtract32(uint32 timestamp, uint32 duration);
uint64 get_dt64(uint64 t1, uint64 t2);
uint64 timestamp_add64(uint64 timestamp, uint64 duration);
uint64 timestamp_subtract64(uint64 timestamp, uint64 duration);

uint16 address64to16(uint8 *address);

void send_statetousb(instance_data_t *inst, struct TDMAHandler *tdma_handle);
void send_rxmsgtousb(char *data);
void send_txmsgtousb(char *data);
char* get_msg_fcode_string(int fcode);

#ifdef __cplusplus
}
#endif

#endif
