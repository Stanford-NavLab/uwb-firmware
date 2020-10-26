/*! ----------------------------------------------------------------------------
 *  @file    instance_common.c
 *  @brief   DecaWave application level common instance functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include "deca_spi.h"
#include "instance.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------

static instance_data_t instance_data[NUM_INST] ;
static struct TDMAHandler tdma_handler;

extern const uint16 rfDelays[2];
extern const tx_struct txSpectrumConfig[8];

extern double dwt_getrangebias(uint8 chan, float range, uint8 prf);

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
/* @fn 	  instance_get_local_structure_ptr
 * @brief function to return the pointer to local instance data structure
 * */
instance_data_t* instance_get_local_structure_ptr(unsigned int x)
{
	if (x >= NUM_INST)
	{
		return NULL;
	}

	return &instance_data[x];
}

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
/* @fn 	  instance_get_local_structure_ptr
 * @brief function to return the pointer to local instance data structure
 * */
struct TDMAHandler* tdma_get_local_structure_ptr()
{

	return &tdma_handler;
}

// -------------------------------------------------------------------------------------------------------------------
// function to initialise tdma structures
//
// Returns 0 on success and -1 on error
int tdma_init_s()
{

    tdma_handler = TDMAHandler.new();

    return 0 ;
}



// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint32 convertmicrosectodevicetimeu32 (double microsecu)
{
    uint32 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint32) (dtime) ;

    return dt;
}


double convertdevicetimetosec(int32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

double convertdevicetimetosec8(uint8* dt)
{
    double f = 0;

    uint32 lo = 0;
    int8 hi = 0;

    memcpy(&lo, dt, 4);
    hi = dt[4] ;

    f = ((hi * 65536.00 * 65536.00) + lo) * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}


int reportTOF(instance_data_t *inst, uint8 uwb_index)
{
	//not TOF to report if not a uwb that we are tracking
	if(uwb_index > UWB_LIST_SIZE)
	{
		return -1;
	}

	double distance ;
	double distance_to_correct ;
	double tof ;
	int64 tofi ;

	// check for negative results and accept them making them proper negative integers
	tofi = inst->tof[uwb_index] ;                      // make it signed

	if (tofi > 0x007FFFFFFFFF)                         // MP counter is 40 bits,  close up TOF may be negative
	{
		tofi -= 0x010000000000 ;                       // subtract fill 40 bit range to make it negative
	}

	// convert to seconds (as floating point)
	tof = convertdevicetimetosec(tofi);          //this is divided by 4 to get single time of flight
	inst->idistanceraw[uwb_index] = distance = tof * SPEED_OF_LIGHT;

#if (CORRECT_RANGE_BIAS == 1)
	//for the 6.81Mb data rate we assume gating gain of 6dB is used,
	//thus a different range bias needs to be applied
	//if(inst->configData.dataRate == DWT_BR_6M8)
	if(inst->smartPowerEn)
	{
		//1.31 for channel 2 and 1.51 for channel 5
		if(inst->configData.chan == 5)
		{
			distance_to_correct = distance/1.51;
		}
		else //channel 2
		{
			distance_to_correct = distance/1.31;
		}
	}
	else
	{
		distance_to_correct = distance;
	}
	distance = distance - dwt_getrangebias(inst->configData.chan, (float) distance_to_correct, inst->configData.prf);
#endif

	if ((distance < 0) || (distance > 20000.000))    // discount any items with error
	{
		inst->idistance[uwb_index] = 0;
		return -1;
	}

	inst->idistance[uwb_index] = distance;

    return 0;
}// end of reportTOF


// -------------------------------------------------------------------------------------------------------------------
//
// function to get the list index of a UWB. UWBs not already in the list are added
//
// -------------------------------------------------------------------------------------------------------------------
// return index of UWB
int instgetuwblistindex(instance_data_t *inst, uint8 *uwbAddr, uint8 addrByteSize)
{
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    char uwbChar[2];
    memcpy(&uwbChar[0], &uwbAddr[0], 2);

    //add the new UWB to the list, if not already there and there is space
    for(uint8 i=0; i<UWB_LIST_SIZE; i++)
    {
        if(memcmp(&inst->uwbList[i][0], &uwbAddr[0], addrByteSize) != 0)
        {
            if(memcmp(&inst->uwbList[i][0], &blank[0], addrByteSize) == 0) //blank entry
            {
                memcpy(&inst->uwbList[i][0], &uwbAddr[0], addrByteSize) ;
                inst->uwbListLen = i + 1 ;
				return i;
            }
        }
        else
        {
        	return i;
        }
    }

    return 255;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to find the number of neighbor UWBs in our list that are not in a timeout status
//
// -------------------------------------------------------------------------------------------------------------------
int instfindnumneighbors(instance_data_t *inst)
{
    uint8 num = 0;

    for(int i=1; i<inst->uwbListLen; i++)// 0 reserved for self, cant be neighbor
    {
		if(tdma_handler.uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR)
		{
			num++;
		}
    }

    return num;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to find the number of hidden neighbor UWBs in our list that are not in a timeout status
//
// -------------------------------------------------------------------------------------------------------------------
int instfindnumhidden(instance_data_t *inst)
{
    uint8 num = 0;

    for(int i=1; i<inst->uwbListLen; i++)//0 reserved for self, cannot be hidden
    {
		if(tdma_handler.uwbListTDMAInfo[i].connectionType == UWB_LIST_HIDDEN)
		{
			num++;
		}
    }

    return num;
}



// -------------------------------------------------------------------------------------------------------------------
#if (NUM_INST != 1)
#error These functions assume one instance only
#else

void instclearuwbList(void)
{
    int instance = 0 ;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    instance_data[instance].uwbListLen = 0 ;
    instance_data[instance].uwbToRangeWith = 255;
	
	for(int i=0; i<UWB_LIST_SIZE; i++)
	{
		memcpy(&instance_data[instance].uwbList[i][0], &blank[0], 8);
	}
}


// -------------------------------------------------------------------------------------------------------------------
// get this instance's role the Tag, Anchor or Listener
int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
    if(instance_data[0].newRange)
    {
        instance_data[0].newRange = 0;
        return 1;
    }

    return 0;
}

uint64 instancenewrangeancadd(void)
{
    return instance_data[0].newRangeAncAddress;
}

uint64 instancenewrangetagadd(void)
{
    return instance_data[0].newRangeTagAddress;
}

int instanceanchorwaiting(void)
{
	return instance_data[0].canPrintInfo;
}


int instancesleeping(void)
{
	if(instance_data[0].canPrintInfo == 1)
	{
		instance_data[0].canPrintInfo = 0; //clear flag
		return 1;
	}

	return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;

    instance_data[instance].frameSN = 0;

    dwt_configeventcounters(1); //enable and clear - NOTE: the counters are not preserved when in DEEP SLEEP

    instance_data[instance].frameSN = 0;
    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
    instance_data[instance].lateTX = 0;
    instance_data[instance].lateRX = 0;

    instclearuwbList();


} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{
    int instance = 0 ;

    int result;
    
    instance_data[instance].mode =  DISCOVERY;
//    instance_data[instance].ranging = 0;
    instance_data[instance].goToSleep = 0;

    for(uint8 i=0; i<UWB_LIST_SIZE; i++)
	{
		instance_data[instance].tof[i] = 0;
		instance_data[instance].idistance[i] = 0;
		instance_data[instance].idistanceraw[i] = 0;
	}
	instance_data[instance].newRangeUWBIndex = 0;



    // Reset the IC (might be needed if not getting here from POWER ON)
    dwt_softreset();

	//we can enable any configuration loading from OTP/ROM on initialization
    result = dwt_initialise(DWT_LOADUCODE) ;

    //this is platform dependent - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialize has failed
    }

    //enable TX, RX states on GPIOs 6 and 5
    dwt_setlnapamode(1,1);

    instanceclearcounts() ;

    instance_data[instance].sleepingEabled = 1;
    instance_data[instance].panID = 0xdeca ;
    instance_data[instance].wait4ack = 0;
    instance_data[instance].instanceTimerEnabled = 0;

    instance_clearevents();

    dwt_geteui(instance_data[instance].eui64);

    instance_data[instance].canPrintInfo = 0;
    instance_data[instance].clockOffset = 0;
    instance_data[instance].monitor = 0;

    return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//
uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}

#if (REG_DUMP == 1)
void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}
#endif

#define TXCFG_ADDRESS  (0x10) // OTP address at which the TX power calibration value is stored
							  // The TX power configuration read from OTP (6 channels consecutively with PRF16 then 64, e.g. Ch 1 PRF 16 is index 0 and PRF 64 index 1)
#define ANTDLY_ADDRESS (0x1C) // OTP address at which the antenna delay calibration value is stored

extern const uint8 chan_idx[];
// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device opetation
//
void instance_config(instanceConfig_t *config)
{
    int instance = 0 ;
    uint32 power = 0;

    instance_data[instance].txAntennaDelay = 0;
    instance_data[instance].rxAntennaDelay = 0;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_EXT ;
	instance_data[instance].configData.sfdTO = config->sfdTO;

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData) ;

    //NOTE: For EVK1000 the OTP stores calibrated antenna and TX power values for configuration modes 3 and 5,

    //check if to use the antenna delay calibration values as read from the OTP
    if(dwt_otprevision() <= 1) //in revision 0, 1 of EVB1000/EVK1000
    {
    	uint32 antennaDelay;
    	uint32 otpPower[12];

    	//MUST change the SPI to < 3MHz as the dwt_otpread will change to XTAL clock
    	port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

    	dwt_otpread(ANTDLY_ADDRESS, &antennaDelay, 1);

    	instance_data[instance].txAntennaDelay = ((antennaDelay >> (16*(config->pulseRepFreq - DWT_PRF_16M))) & 0xFFFF) >> 1;
    	instance_data[instance].rxAntennaDelay = instance_data[instance].txAntennaDelay ;

    	//read any data from the OTP for the TX power
    	dwt_otpread(TXCFG_ADDRESS, otpPower, 12);

    	port_set_dw1000_fastrate(); //increase SPI to max

        power = otpPower[(config->pulseRepFreq - DWT_PRF_16M) + (chan_idx[instance_data[instance].configData.chan] * 2)];
    }

#if (SET_TXRX_DELAY == 0)
    // if nothing was actually programmed then set a reasonable value anyway
    if(instance_data[instance].txAntennaDelay == 0)//otherwise a default values should be used
    {
    	instance_data[instance].rxAntennaDelay = instance_data[instance].txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
    }
#else
    instance_data[instance].txAntennaDelay = (uint16)TX_ANT_DELAY;
	instance_data[instance].rxAntennaDelay = (uint16)RX_ANT_DELAY;
#endif

    // -------------------------------------------------------------------------------------------------------------------
    // set the antenna delay, we assume that the RX is the same as TX.
    dwt_setrxantennadelay(instance_data[instance].txAntennaDelay);
    dwt_settxantennadelay(instance_data[instance].txAntennaDelay);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    instance_data[instance].configTX.power = power;
    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

    instance_data[instance].antennaDelayChanged = 0;

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	port_set_dw1000_slowrate(); //reduce SPI to < 3MHz

		dwt_loadopsettabfromotp(0);

		port_set_dw1000_slowrate(); //increase SPI to max
    }


#if (REG_DUMP == 1)
#define REG_BUF_SIZE    (100*30)
{
	char regDumpBuffer[REG_BUF_SIZE] ;
	dwt_dumpregisters(regDumpBuffer, REG_BUF_SIZE);

	{
		char* buff[REG_BUF_SIZE+10];
		int n = sprintf((char*)&buff[0], "%s", regDumpBuffer);
		send_usbmessage(&regDumpBuffer[0], n);
	}
}
#endif
}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(int sleepdelay, int blinksleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_ms = sleepdelay ;
    instance_data[instance].tagBlinkSleepTime_ms = blinksleepdelay ;
}


double instance_get_idist(uint8 uwb_index) //get instantaneous range
{
    double x = instance_data[0].idistance[uwb_index];

    return (x);
}

double instance_get_idistraw(uint8 uwb_index) //get instantaneous range
{
    double x = instance_data[0].idistanceraw[uwb_index];

    return (x);
}

int instance_get_rxf(void) //get number of Rxed frames
{
    int x = instance_data[0].rxmsgcount;

    return (x);
}

int instance_get_txf(void) //get number of Txed frames
{
    int x = instance_data[0].txmsgcount;

    return (x);
}

int instance_get_txl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateTX;

    return (x);
}

int instance_get_rxl(void) //get number of late Tx frames
{
    int x = instance_data[0].lateRX;

    return (x);
}

void inst_processtxrxtimeout(instance_data_t *inst)
{
	if(inst->mode == DISCOVERY)
	{
		inst->wait4ack = 0;
		inst->uwbToRangeWith = 255;

		if(tdma_handler.discovery_mode == WAIT_RNG_INIT || tdma_handler.discovery_mode == WAIT_INF_INIT)
		{
			uint32 time_now = portGetTickCnt();
			tdma_handler.set_discovery_mode(&tdma_handler, WAIT_INF_REG, time_now);
		}

		if(tdma_handler.check_blink(&tdma_handler) == TRUE)
		{
			inst->testAppState = TA_TX_SELECT;
		}
		else
		{
			inst->testAppState = TA_RXE_WAIT;
		}

		dwt_setrxtimeout(0);
	}
	else if(inst->mode == ANCHOR) //we did not receive the final/ACK - wait for next poll
    {
    	inst->wait4ack = 0;
    	inst->uwbToRangeWith = 255;
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);
    }
	else //if(inst->mode == TAG)
    {
		inst->testAppState = TA_TX_SELECT ;
    }

    inst->previousState = TA_INIT;
    //timeout - disable the radio (if using SW timeout the rx will not be off)
    dwt_forcetrxoff() ;
}

void instance_txcallback(const dwt_cb_data_t *txd)
{
	int instance = 0;
	uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
	event_data_t dw_event;

	//NOTE: we can only get TX good (done) while here
	dwt_readtxtimestamp(txTimeStamp) ;
	dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
	dw_event.timeStamp = txTimeStamp[4];
	dw_event.timeStamp <<= 32;
	dw_event.timeStamp += dw_event.timeStamp32l;
	dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

	dw_event.rxLength = 0;
	dw_event.typeSave = dw_event.type = DWT_SIG_TX_DONE ;

	instance_putevent(dw_event);

#if (DEEP_SLEEP == 1)
	if (instance_data[instance].sleepingEabled)
		instance_data[instance].txmsgcount++;
#endif

	instance_data[instance].monitor = 0;
}

void instance_rxtimeoutcallback(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	dw_event.typeSave = dw_event.type = DWT_SIG_RX_TIMEOUT;
	dw_event.rxLength = 0;
	dw_event.timeStamp = 0;
	dw_event.timeStamp32l = 0;
	dw_event.timeStamp32h = 0;

	instance_putevent(dw_event);
}

void instance_rxerrorcallback(const dwt_cb_data_t *rxd)
{
	int instance = 0;

	//error caused by automatic frame filtering, ignore it and immediately turn rx back on
	if(rxd->status & SYS_STATUS_AFFREJ){
		instancerxon(&instance_data[instance], 0, 0);
		return;
	}

	if(tdma_handler.discovery_mode == WAIT_RNG_INIT || tdma_handler.discovery_mode == WAIT_INF_INIT)
	{
		uint32 time_now = portGetTickCnt();
		tdma_handler.set_discovery_mode(&tdma_handler, WAIT_INF_REG, time_now);
	}


	event_data_t dw_event;
	//re-enable the receiver
	//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
	if(instance_data[instance].mode == TAG || instance_data[instance].mode == DISCOVERY)
	{
		dw_event.type = DWT_SIG_RX_TIMEOUT;
		dw_event.typeSave = 0x40 | DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;

		instance_putevent(dw_event);
	}
	else
	{
		instance_data[instance].uwbToRangeWith = 255;

		instancerxon(&instance_data[instance], 0, 0); //immediate enable if anchor or listener
	}

}


void instance_rxgoodcallback(const dwt_cb_data_t *rxd)
{

	int instance = 0;
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};
	uint8 srcAddr_index = 0;
    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	event_data_t dw_event;

	uint32 time_now = portGetTickCnt();
	uint64 time_now_us = portGetTickCntMicro();

	//if we got a frame with a good CRC - RX OK
	rxd_event = DWT_SIG_RX_OKAY;

	dw_event.rxLength = rxd->datalength;

	//need to process the frame control bytes to figure out what type of frame we have received
	switch(rxd->fctrl[0])
	{
		//blink type frame
		case 0xC5:
			if(rxd->datalength == 12)
			{
				rxd_event = DWT_SIG_RX_BLINK;
			}
			else
				rxd_event = SIG_RX_UNKNOWN;

			break;

		//ACK type frame - not supported in this SW - set as unknown (re-enable RX)
		case 0x02:
			rxd_event = SIG_RX_UNKNOWN;
			break;

		//data type frames (with/without ACK request) - assume PIDC is on.
		case 0x41:
		case 0x61:
			//read the frame
			if(rxd->datalength > EXTENDED_FRAME_SIZE)
				rxd_event = SIG_RX_UNKNOWN;
			
			//need to check the destination/source address mode
			if((rxd->fctrl[1] & 0xCC) == 0x88) //dest & src short (16 bits)
			{
				fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
				srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
			}
			else if((rxd->fctrl[1] & 0xCC) == 0xCC) //dest & src long (64 bits)
			{
				fcode_index = FRAME_CRTL_AND_ADDRESS_L; //function code is in first byte after source address
				srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_L;
			}
			else //using one short/one long
			{
				fcode_index = FRAME_CRTL_AND_ADDRESS_LS; //function code is in first byte after source address

				if(((rxd->fctrl[1] & 0xCC) == 0x8C)) //source short
				{
					srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_L;
				}
				else
				{
					srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
				}
			}

			break;

		//any other frame types are not supported by this application
		default:
			rxd_event = SIG_RX_UNKNOWN;
			break;
	}

	//read rx timestamp
	if((rxd_event == DWT_SIG_RX_BLINK) || (rxd_event == DWT_SIG_RX_OKAY))
	{
		dwt_readrxtimestamp(rxTimeStamp) ;
		dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
		dw_event.timeStamp = rxTimeStamp[4];
		dw_event.timeStamp <<= 32;
		dw_event.timeStamp += dw_event.timeStamp32l;
		dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

		dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
	}

	dw_event.typeSave = dw_event.type = rxd_event;

	//----------------------------------------------------------------------------------------------
	//TWR - here we check if we need to respond to a TWR Poll or Response Messages
	//----------------------------------------------------------------------------------------------

	//don't process unknown signals or non-broadcast messages that aren't addressed to this UWB
	if(rxd_event == DWT_SIG_RX_OKAY)
	{

		if((dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_INIT ) || 
		   (dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL ) ||
		   (dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP) ||
		   (dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_FINAL))
		{
			uint8 destAddr_index = FRAME_CTRLP;

			if(memcmp(&instance_data[instance].uwbList[0][0], &dw_event.msgu.frame[destAddr_index], instance_data[instance].addrByteSize) != 0)
    		{
				rxd_event = SIG_RX_UNKNOWN;
			}
		}
		else if(dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_INF_REG &&
				dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_INF_INIT &&
				dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_INF_SUG &&
				dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_INF_UPDATE &&
				dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_RNG_REPORT &&
				dw_event.msgu.frame[fcode_index] != RTLS_DEMO_MSG_SYNC)
		{
			rxd_event = SIG_RX_UNKNOWN;
		}
	}

	//first check if the address is already tracked.
	//if not, add it.
	//then get the index for that address
	uint8 uwb_index = 255;
	if(rxd_event == DWT_SIG_RX_BLINK)
	{
		uint8 blink_address[8] = {0,0,0,0,0,0,0,0};
#if (USING_64BIT_ADDR==0)
		memcpy(&blink_address, &dw_event.msgu.rxblinkmsg.tagID[0], instance_data[instance].addrByteSize);
#else
		uint16 blink_address_short = address64to16(&dw_event.msgu.rxblinkmsg.tagID[0]);
		memcpy(&blink_address, &blink_address_short, instance_data[instance].addrByteSize);
#endif

		//must be a neighbor
		uwb_index = instgetuwblistindex(&instance_data[instance], &blink_address[0], instance_data[instance].addrByteSize);

		if(uwb_index > 0 && uwb_index < UWB_LIST_SIZE)
		{
			tdma_handler.uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_NEIGHBOR;
			tdma_handler.uwbListTDMAInfo[uwb_index].lastCommNeighbor = time_now;
		}

		//TODO remove
		uint8 debug_msg[100];
//		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK RECEIVED: BLINK uwb_index: %d, uwbToRangeWith: %d ", uwb_index, instance_data[instance].uwbToRangeWith);
		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK RECEIVED: BLINK,%llX,xxxx", instance_get_addr());
		send_usbmessage(&debug_msg[0], n);
		usb_run();
	}
	else if(rxd_event == DWT_SIG_RX_OKAY)
	{
		//must be a neighbor
		uwb_index = instgetuwblistindex(&instance_data[instance], &dw_event.msgu.frame[srcAddr_index], instance_data[instance].addrByteSize);

		if(uwb_index > 0 && uwb_index < UWB_LIST_SIZE)
		{
			tdma_handler.uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_NEIGHBOR;
			tdma_handler.uwbListTDMAInfo[uwb_index].lastCommNeighbor = time_now;
		}

		//TODO remove
		uint8 debug_msg[100];
		uint16 my_addr = instance_data[instance].uwbShortAdd;
//		int n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
//		 send_usbmessage(&debug_msg[0], n);
//		 usb_run();
		int n = 0;

		 if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_INIT )
		 {
			n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL )
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_FINAL)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_REG)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_INIT)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_SUG)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_UPDATE)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_REPORT)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }
		 else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_SYNC)
		 {
			 n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,uwb_index: %04d, uwb_trw: %04d", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr,uwb_index, instance_data[instance].uwbToRangeWith);
		 }

		 send_usbmessage(&debug_msg[0], n);
		 usb_run();


	}

	bool accept_inf = FALSE;
	//check if the incoming message indicates that we should range with the source UWB or just accept and process the message
	if(rxd_event != SIG_RX_UNKNOWN)
	{
		if(instance_data[instance].uwbToRangeWith == 255)
		{
			if(rxd_event == DWT_SIG_RX_OKAY)
			{
				if(instance_data[instance].mode == ANCHOR)
				{
					if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL)
					{
						instance_data[instance].uwbToRangeWith = uwb_index;
					}
					else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_REG || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_SUG || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_UPDATE)//only anchor if already received INF_INIT or collected regular INF messages and joined with SUG message
					{
						accept_inf = TRUE;
					}
				}
				else if (instance_data[instance].mode == DISCOVERY)
				{
					if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_INIT)
					{
						if(tdma_handler.discovery_mode == WAIT_RNG_INIT)
						{
							instance_data[instance].uwbToRangeWith = uwb_index;
						}
					}
					else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_INIT)
					{
						//only accept if we are waiting for INF_INIT
						if(tdma_handler.discovery_mode == WAIT_INF_INIT)
						{
							instance_data[instance].uwbToRangeWith = uwb_index;
							accept_inf = TRUE;
						}
					}
					else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_REG ||
							dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_UPDATE ||
							dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_SUG)
					{
						if(tdma_handler.discovery_mode == WAIT_INF_REG || tdma_handler.discovery_mode == COLLECT_INF_REG || tdma_handler.discovery_mode == WAIT_INF_INIT)
						{
							accept_inf = TRUE;
						}
					}
				}
			}
			else if(rxd_event == DWT_SIG_RX_BLINK)
			{
				//only accept BLINK if in DISCOVERY mode and waiting for INF_REG
				if(instance_data[instance].mode == DISCOVERY)
				{
					if(tdma_handler.discovery_mode == WAIT_INF_REG)
					{
						instance_data[instance].uwbToRangeWith = uwb_index;
					}
				}
			}
		}
	}

	int place_event = 0;

	if(rxd_event == DWT_SIG_RX_OKAY && uwb_index != 255)
	{
		//always accept.
		if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_REPORT || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_SYNC)
		{
			place_event = 1;
		}
	}

	if(accept_inf == TRUE)
	{
		place_event = 1;
	}
	else if(uwb_index != 255 && instance_data[instance].uwbToRangeWith == uwb_index)
	{
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			//process RTLS_DEMO_MSG_TAG_POLL immediately.
			if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL)
			{
				uint16 frameLength = 0;

				instance_data[instance].tagPollRxTime = dw_event.timeStamp ; //Poll's Rx time

#if (IMMEDIATE_RESPONSE == 0)
				instance_data[instance].delayedReplyTime = (instance_data[instance].tagPollRxTime + instance_data[instance].responseReplyDelay) >> 8 ;  // time we should send the response
#else
				instance_data[instance].delayedReplyTime = 0;
#endif

#if (USING_64BIT_ADDR == 1)
				frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
				frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

				instance_data[instance].msg.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

				memcpy(&instance_data[instance].msg.destAddr[0], &dw_event.msgu.frame[srcAddr_index], instance_data[instance].addrByteSize); //remember who to send the reply to (set destination address)
				
				instance_data[instance].msg.seqNum = instance_data[instance].frameSN++;

				//TODO
				//set the delayed rx on time (the final message will be sent after this delay)
//				dwt_setrxaftertxdelay((uint32)instance_data[instance].txToRxDelayAnc_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
				 dwt_setrxaftertxdelay(0);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				instance_data[instance].wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_writetxfctrl(frameLength, 0, 1);
				dwt_writetxdata(frameLength, (uint8 *)  &instance_data[instance].msg, 0) ;	// write the frame data

#if (IMMEDIATE_RESPONSE == 1)
				dwt_starttx(DWT_START_TX_IMMEDIATE | instance_data[instance].wait4ack);
#else
				if(instancesendpacket(frameLength, DWT_START_TX_DELAYED | instance_data[instance].wait4ack, instance_data[instance].delayedReplyTime))
				{
					dw_event.typePend = DWT_SIG_TX_ERROR ;
					dwt_setrxaftertxdelay(0);
					instance_data[instance].wait4ack = 0; //clear the flag as the TX has failed the TRX is off
					instance_data[instance].lateTX++;
				}
				else
#endif
				{
					dw_event.typePend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
					instance_data[instance].timeofTx = time_now;
					instance_data[instance].txDoneTimeoutDuration = 200; //NOTE timeout duration found experimentally, may need to be changed if the delays in instance.h are modified
				}
			}

			place_event = 1;
		}
		else if (rxd_event == DWT_SIG_RX_BLINK)
		{
			place_event = 1;
		}
	}

	if(place_event)
	{
		instance_putevent(dw_event);

#if (DEEP_SLEEP == 1)
		if (instance_data[instance].sleepingEabled)
		{
			instance_data[instance].rxmsgcount++;
		}
#endif
	}
	else
	{
		instancerxon(&instance_data[instance], 0, 0); //immediately reenable RX
	}
}

int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

void instance_putevent(event_data_t newevent)
{
	int instance = 0;
	uint8 etype = newevent.type;

	newevent.type = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn].type = etype;

	instance_data[instance].dweventIdxIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxIn)
		instance_data[instance].dweventIdxIn = 0;
}

event_data_t dw_event_g;

event_data_t* instance_getevent(int x)
{
	int instance = 0;
	int indexOut = instance_data[instance].dweventIdxOut;

	if(instance_data[instance].dwevent[indexOut].type == 0) //exit with "no event"
	{
		dw_event_g.type = 0;
		dw_event_g.typeSave = 0;
		return &dw_event_g;
	}

	//copy the event
	dw_event_g.typeSave = instance_data[instance].dwevent[indexOut].typeSave ;
	dw_event_g.typePend = instance_data[instance].dwevent[indexOut].typePend ;
	dw_event_g.rxLength = instance_data[instance].dwevent[indexOut].rxLength ;
	dw_event_g.timeStamp = instance_data[instance].dwevent[indexOut].timeStamp ;
	dw_event_g.timeStamp32l = instance_data[instance].dwevent[indexOut].timeStamp32l ;
	dw_event_g.timeStamp32h = instance_data[instance].dwevent[indexOut].timeStamp32h ;

	memcpy(&dw_event_g.msgu, &instance_data[instance].dwevent[indexOut].msgu, sizeof(instance_data[instance].dwevent[indexOut].msgu));

	dw_event_g.type = instance_data[instance].dwevent[indexOut].type ;

	instance_data[instance].dwevent[indexOut].type = 0; //clear the event

	instance_data[instance].dweventIdxOut++;
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxOut) //wrap the counter
	{
		instance_data[instance].dweventIdxOut = 0;
	}

	instance_data[instance].dweventPeek = instance_data[instance].dweventIdxOut; //set the new peek value


	return &dw_event_g;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventIdxIn = 0;
	instance_data[instance].dweventIdxOut = 0;
	instance_data[instance].dweventPeek = 0;

}


// -------------------------------------------------------------------------------------------------------------------
int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR

	while(done == INST_NOT_DONE_YET)
	{

		done = testapprun(&instance_data[instance], &tdma_handler, message) ; // run the communications application

		message = 0;
	}

	uint32 time_now = portGetTickCnt();

	//check timeouts.
	if(tdma_handler.check_timeouts(&tdma_handler, time_now))
	{
		//handle case where we timeout and no longer have any neighbors
		instance_data[instance].mode = DISCOVERY;
		tdma_handler.enter_discovery_mode(&tdma_handler);
		inst_processtxrxtimeout(&instance_data[instance]);

		instance_data[instance].canPrintInfo = 0;
	}

    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
    	if(instance_data[instance].mode == DISCOVERY)
    	{
    		if(instance_data[instance].previousState == TA_TXBLINK_WAIT_SEND)
			{
				instance_data[instance].instanceTimerTime += instance_data[instance].tagBlinkSleepTime_ms; //set timeout time
				instance_data[instance].instanceTimerEnabled = 1; //start timer
			}
    	}

        if(instance_data[instance].mode == TAG) //Tag (is either in RX or sleeping)
        {
			instance_data[instance].instanceTimerTime = instance_data[instance].instanceTimerTimeSaved + instance_data[instance].tagSleepTime_ms; //set timeout time
			instance_data[instance].instanceTimerEnabled = 1; //start timer
        }
    }

    //check if timer has expired
    if(instance_data[instance].instanceTimerEnabled == 1)
    {
        if(instance_data[instance].instanceTimerTime < portGetTickCnt())
        {
			event_data_t dw_event;
            instance_data[instance].instanceTimerEnabled = 0;
			dw_event.rxLength = 0;
			dw_event.type = DWT_SIG_RX_TIMEOUT;
			dw_event.typeSave = 0x80 | DWT_SIG_RX_TIMEOUT;
			instance_putevent(dw_event);
        }
    }

    return ((done != INST_NOT_DONE_YET) ? 1 : 0);
}

void instanceconfigantennadelays(uint16 tx, uint16 rx)
{
	instance_data[0].txAntennaDelay = tx ;
	instance_data[0].rxAntennaDelay = rx ;

	instance_data[0].antennaDelayChanged = 1;
}

void instancesetantennadelays(void)
{
	if(instance_data[0].antennaDelayChanged == 1)
	{
		dwt_setrxantennadelay(instance_data[0].rxAntennaDelay);
		dwt_settxantennadelay(instance_data[0].txAntennaDelay);

		instance_data[0].antennaDelayChanged = 0;
	}
}


uint16 instancetxantdly(void)
{
	return instance_data[0].txAntennaDelay;
}

uint16 instancerxantdly(void)
{
	return instance_data[0].rxAntennaDelay;
}

#endif
