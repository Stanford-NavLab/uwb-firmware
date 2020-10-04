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
	tofi = inst->tof[uwb_index] ;                          // make it signed

//	memcpy(&instance_data[instance].msg[uwb_index].messageData[TOFR], &instance_data[instance].tof[uwb_index], sizeof(int64));

//	int64 mtof = 0;
//	memcpy(&mtof, &tofi, sizeof(int64));
//	uint8 debug_msg[100];
//	int n = sprintf((char*)&debug_msg[0], "reported TOF: %lld, mtof %lld", tofi, mtof);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

	if (tofi > 0x007FFFFFFFFF)                          // MP counter is 40 bits,  close up TOF may be negative
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

    //TODO remove the following!
    bool match = FALSE;
	char test_addr[2] = {0x95, 0x15};
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x50;
	test_addr[1] = 0x59;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x36;
	test_addr[1] = 0x18;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x39;
	test_addr[1] = 0x16;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x27;
	test_addr[1] = 0x59;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x20;
	test_addr[1] = 0x1B;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x8C;
	test_addr[1] = 0x11;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x1D;
	test_addr[1] = 0x14;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0x8C;
	test_addr[1] = 0x19;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}
	test_addr[0] = 0xD6;
	test_addr[1] = 0x0C;
	if(memcmp(&uwbChar[0], &test_addr[0], 2) == 0)
	{
		match = TRUE;
	}

	if(match == FALSE)
	{
//		instance_data_t *inst = &instance_data[instance];
		uint8 debug_msg[100];
		int n = sprintf((char*)&debug_msg[0], "match not found");
		 send_usbmessage(&debug_msg[0], n);
		 usb_run();
	}

    //add the new UWB to the list, if not already there and there is space
    for(uint8 i=0; i<UWB_LIST_SIZE; i++)
    {
        if(memcmp(&inst->uwbList[i][0], &uwbAddr[0], addrByteSize) != 0)
        {
            if(memcmp(&inst->uwbList[i][0], &blank[0], addrByteSize) == 0) //blank entry
            {
                memcpy(&inst->uwbList[i][0], &uwbAddr[0], addrByteSize) ;
                inst->uwbListLen = i + 1 ;
				inst->uwbTimeout[i] = 1;
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
// function to add new UWBs to the uwb list or remove timeout status from UWBs already in uwb list
//
// -------------------------------------------------------------------------------------------------------------------
// return 1 if UWB added to list or removed from timeout status 
// return 0 if UWB not added list or removed from timeout status
int instaddactivateuwbinlist(instance_data_t *inst, uint8 *uwbAddr)
{
    uint8 i;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    //add the new UWB to the list, if not already there and there is space
    for(i=1; i<UWB_LIST_SIZE; i++)//0 reserved for self. timeout status not used for self
    {
        if(memcmp(&inst->uwbList[i][0], &uwbAddr[0], inst->addrByteSize) != 0)
        {
            if(memcmp(&inst->uwbList[i][0], &blank[0], inst->addrByteSize) == 0) //blank entry
            {
                memcpy(&inst->uwbList[i][0], &uwbAddr[0], inst->addrByteSize) ;
                inst->uwbListLen = i + 1 ;
				inst->uwbToRangeWith = i;
				inst->uwbTimeout[i] = 0;
				return 1;
            }
        }
        else
        {
			if(inst->uwbTimeout[i])
			{
				//uwb has timed out, wake it up
				inst->uwbToRangeWith = i;
				inst->uwbTimeout[i] = 0;
				return 1;
			}
			else
			{
				//we already have this uwb in the list and it has not timed out
				break; 
			}
        }
    }

    return 0;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to check if a UWB is already in our list and not in a timeout status
//
// -------------------------------------------------------------------------------------------------------------------
// return index if UWB in list and not timed out
// return 255 if UWB not in list or is but has timed out
int instcheckactiveuwbinlist(instance_data_t *inst, uint8 *uwbAddr)
{
    uint8 i;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    //add the new UWB to the list, if not already there and there is space
    for(i=1; i<UWB_LIST_SIZE; i++)//0 reserved for self, timeout not used for self
    {
        if(memcmp(&inst->uwbList[i][0], &uwbAddr[0], inst->addrByteSize) == 0)
        {
			if(inst->uwbTimeout[i])
			{
				//UWB in list, but timed out 
				break;	
			}
			else
			{
				return i; //we already have this UWB in the list
			}
        }
        else
        {
			if(memcmp(&inst->uwbList[i][0], &blank[0], 8) == 0) //blank entry
            {
                break;
            }
        }
    }

    return 255;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to find the first UWB in our list that is not in a timeout status, starting with the given index
//
// -------------------------------------------------------------------------------------------------------------------
// return index for first UWB in list that is not timed out
// return 255 if all UWBs in list (from the starting index) are timed out
int instfindfirstactiveuwbinlist(instance_data_t *inst, uint8 startindex)
{
    uint8 i;
    
    for(i=startindex; i<inst->uwbListLen; i++)
    {
		if(!inst->uwbTimeout[i])
		{
			return i;
		}
    }

    return 255;
}



// -------------------------------------------------------------------------------------------------------------------
//
// function to find the number of UWBs in our list that are not in a timeout status
//
// -------------------------------------------------------------------------------------------------------------------
int instfindnumactiveuwbinlist(instance_data_t *inst)
{
    uint8 num = 0;
    
    for(int i=1; i<inst->uwbListLen; i++) //0 reserved for self, timeout status not applicable
    {
		if(!inst->uwbTimeout[i])
		{
			num++;
		}
    }

    return num;
}

// -------------------------------------------------------------------------------------------------------------------
//
// function to find the number of neighbor UWBs in our list that are not in a timeout status
//
// -------------------------------------------------------------------------------------------------------------------
int instfindnumactiveneighbors(instance_data_t *inst)
{
    uint8 num = 0;

    for(int i=1; i<inst->uwbListLen; i++)// 0 reserved for self, cant be neighbor
    {
//		if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR)
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
int instfindnumactivehidden(instance_data_t *inst)
{
    uint8 num = 0;

    for(int i=1; i<inst->uwbListLen; i++)//0 reserved for self, cannot be hidden
    {
//		if(inst->uwbListType[i] == UWB_LIST_HIDDEN)
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
		instance_data[instance].lastCommTimeStamp[i] = 0;
//		instance_data[instance].lastRangeTimeStamp[i] = 0;
		tdma_handler.uwbListTDMAInfo[i].lastRange = 0;
		instance_data[instance].uwbTimeout[i] = 0;
//		instance_data[instance].time_till_next_reported[i] = 0;
		
		memcpy(&instance_data[instance].uwbList[i][0], &blank[0], 8);
//		instance_data[instance].uwbListType[i] = UWB_LIST_INACTIVE;
		tdma_handler.uwbListTDMAInfo[i].connectionType = UWB_LIST_INACTIVE;
	}
}


// -------------------------------------------------------------------------------------------------------------------
// get this instance's role the Tag, Anchor or Listener
int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instanceisranging(void)
{
    if(instance_data[0].ranging == 1)
    {
        return 1;
    }

    return 0;
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

    instance_data[instance].tofCount = 0 ;
    instance_data[instance].tofIndex = 0 ;

    instance_data[instance].txmsgcount = 0;
    instance_data[instance].rxmsgcount = 0;
    instance_data[instance].lateTX = 0;
    instance_data[instance].lateRX = 0;

    instance_data[instance].longTermRangeSum  = 0;
    instance_data[instance].longTermRangeCount  = 0;

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
    instance_data[instance].ranging = 0;
    instance_data[instance].goToSleep = 0;

    instance_data[instance].tofIndex = 0;
    instance_data[instance].tofCount = 0;
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

//	dwt_setdblrxbuffmode(1);
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
//    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
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


	// send_statetousb(inst);

//	 uint8 debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "inst_processrxtimeout(), uwbToRangeWith: %i, uwbListlen: %i", inst->uwbToRangeWith, inst->uwbListLen);
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();
	
//	tdma_handler.waitForInf = FALSE;
//	tdma_handler.waitForRngInit = FALSE;


	if(inst->mode == DISCOVERY)
	{
		inst->wait4ack = 0;
		inst->uwbToRangeWith = 255;
//		uint32 maxFrameDuration = tdma_handler.maxFramelength*tdma_handler.slotDuration;

		if(tdma_handler.discovery_mode == WAIT_RNG_INIT || tdma_handler.discovery_mode == WAIT_INF_INIT)
		{
//			tdma_handler.discovery_mode = WAIT_INF_REG;
			uint32 time_now = portGetTickCnt();
			tdma_handler.set_discovery_mode(&tdma_handler, WAIT_INF_REG, time_now);
		}

//		if(inst->previousState == TA_TXBLINK_WAIT_SEND)
//		{
//			tdma_handler.waitForRngInit = FALSE;
//		}
//		else if(inst->previousState == TA_TXRANGINGINIT_WAIT_SEND)
//		{
//			tdma_handler.waitForInf = FALSE;
//		}

		//TODO should I keep/remove this...?
		if(tdma_handler.check_blink(&tdma_handler) == TRUE)
		{
			inst->testAppState = TA_TX_SELECT;
		}
		else
		{
			inst->testAppState = TA_RXE_WAIT;
		}
//		uint32 maxFrameDuration = tdma_handler.maxFramelength*tdma_handler.slotDuration;
//		if(portGetTickCnt() - tdma_handler.discoveryStartTime > maxFrameDuration) //TODO handle number wrapping!
//		{
//			inst->testAppState = TA_TX_SELECT ;
//		}
//		else
//		{
//			inst->testAppState = TA_RXE_WAIT;
//		}

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
		// initiate the re-transmission of the poll that was not responded to
		inst->testAppState = TA_TX_SELECT ;
//	 uint8 debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "inst_processtxrxtimeout");
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();
//		if(inst->previousState == TA_TXBLINK_WAIT_SEND)
//		{
//			inst->uwbToRangeWith = instfindfirstactiveuwbinlist(inst, 0);
//		}
//		else if(inst->previousState == TA_TXPOLL_WAIT_SEND)
//		{
//			uint8 idx = inst->uwbToRangeWith + 1;
//			inst->uwbToRangeWith = instfindfirstactiveuwbinlist(inst, idx);
//		}
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

	//NOTE - we can only get TX good (done) while here
	dwt_readtxtimestamp(txTimeStamp) ;
	dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
	dw_event.timeStamp = txTimeStamp[4];
	dw_event.timeStamp <<= 32;
	dw_event.timeStamp += dw_event.timeStamp32l;
	dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

	dw_event.rxLength = 0;
	dw_event.typeSave = dw_event.type = DWT_SIG_TX_DONE ;

//	 uint32 dt = portGetTickCnt() - instance_data[instance].timeofTx;
//	 char debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "TX CALLBACK: DWT_SIG_TX_DONE. dt: %lu ", dt);
//	 send_usbmessage((uint8*)&debug_msg[0], n);
//	 usb_run();

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

//	int instance = 0;
//	uint32 response_time = portGetTickCnt() - instance_data[instance].range_start;

//	uint8 debug_msg[200];
//	int n = 0;
//	n = sprintf((char*)&debug_msg[0], "RX TIMEOUT CALLBACK");// duration: %lu", response_time);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();



	dw_event.typeSave = dw_event.type = DWT_SIG_RX_TIMEOUT;
	dw_event.rxLength = 0;
	dw_event.timeStamp = 0;
	dw_event.timeStamp32l = 0;
	dw_event.timeStamp32h = 0;

	instance_putevent(dw_event);
}

void instance_rxerrorcallback(const dwt_cb_data_t *rxd)
{
//	uint8 debug_msg[200];
//	int n = 0;
//	n = sprintf((char*)&debug_msg[0], "RX ERROR CALLBACK");
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();


	int instance = 0;

	//error caused by automatic frame filtering, ignore it and immediately turn rx back on
	if(rxd->status & SYS_STATUS_AFFREJ){
		instancerxon(&instance_data[instance], 0, 0);
		return;
	}


//	tdma_handler.waitForInf = FALSE;
//	tdma_handler.waitForRngInit = FALSE;
	if(tdma_handler.discovery_mode == WAIT_RNG_INIT || tdma_handler.discovery_mode == WAIT_INF_INIT)
	{
//		tdma_handler.discovery_mode = WAIT_INF_REG;
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
//		uint8 debug_msg[100];
//		 int n = sprintf((char*)&debug_msg[0], "instancerxon called from  case instance_rxerrorcallback :");
//		 send_usbmessage(&debug_msg[0], n);
//		 usb_run();

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
	

//	int num_neighbors = instfindnumactiveneighbors(&instance_data[instance]);
//	uint8 debug_msg[150];
//	int n = sprintf((char*)&debug_msg[0], "RX CALLBACK, mode: %s, num_neighbors %d, discovery_mode %s", get_instanceModes_string(instance_data[instance].mode), num_neighbors, get_discovery_modes_string(tdma_handler.discovery_mode));
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

//		uint8 debug_msg[150];
//		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK");
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();


	uint32 time_now = portGetTickCnt();
	uint32 time_now_us = portGetTickCntMicro();

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

//				uint8 debug_msg[150];
//				int n = sprintf((char*)&debug_msg[0], "RX CALLBACK: DWT_SIG_RX_BLINK");
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();
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

//#if (USING_64BIT_ADDR==0)
//			if(memcmp(&instance_data[instance].uwbShortAdd, &dw_event.msgu.frame[destAddr_index], instance_data[instance].addrByteSize) != 0)
//#else
//    		if(memcmp(&instance_data[instance].eui64[0], &dw_event.msgu.frame[destAddr_index], instance_data[instance].addrByteSize) != 0)
//#endif
			if(memcmp(&instance_data[instance].uwbList[0][0], &dw_event.msgu.frame[destAddr_index], instance_data[instance].addrByteSize) != 0)
    		{
//				uint8 debug_msg[150];
//				int n = sprintf((char*)&debug_msg[0], "RX Message not addressed to me");
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
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
//			uint8 debug_msg[150];
//			int n = sprintf((char*)&debug_msg[0], "not INIT/POLL/RESP/FINAL");
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();
		}
	}

//	if(rxd_event == SIG_RX_UNKNOWN){
//		uint8 debug_msg[150];
//		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK: SIG_RX_UNKNOWN");
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();
//	}

	//ANCHOR RX
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
//		uwb_index = instgetuwblistindex(&instance_data[instance], &dw_event.msgu.rxblinkmsg.tagID[0], instance_data[instance].addrByteSize);
		uwb_index = instgetuwblistindex(&instance_data[instance], &blink_address[0], instance_data[instance].addrByteSize);
//		instance_data[instance].uwbListType[uwb_index] = UWB_LIST_NEIGHBOR;


//		uint8 debug_msg[100];
////		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK RECEIVED: BLINK uwb_index: %d, uwbToRangeWith: %d ", uwb_index, instance_data[instance].uwbToRangeWith);
//		int n = sprintf((char*)&debug_msg[0], "RX CALLBACK RECEIVED: BLINK,%llX,xxxx", instance_get_addr());
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();
	}
	else if(rxd_event == DWT_SIG_RX_OKAY)
	{
		//must be a neighbor
		uwb_index = instgetuwblistindex(&instance_data[instance], &dw_event.msgu.frame[srcAddr_index], instance_data[instance].addrByteSize);

		//TODO maybe do this somewhere else...
//		instance_data[instance].uwbListType[uwb_index] = UWB_LIST_NEIGHBOR;
		tdma_handler.uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_NEIGHBOR;
		instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
		instance_data[instance].uwbTimeout[uwb_index] = 0;



//		uint8 debug_msg[100];
////		 int n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s uwb_index %d, uwbToRangeWith: %d ", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), uwb_index, instance_data[instance].uwbToRangeWith);
//		uint16 my_addr = instance_data[instance].uwbShortAdd;
//////		uint64 my_addr = (uint64)instance_data[instance].uwbShortAdd;
////		uint16 *addr_ptr = &instance_data[instance].uwbShortAdd;
////		int n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%u,xxxx,%p", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr, (void *) addr_ptr);
////		int n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%u,xxxx", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr);
//		int n = sprintf((char*)&debug_msg[0], "RX CB RX: DWT_SIG_RX_OKAY-%s,%04X,xxxx", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), my_addr);
////		int n = sprintf((char*)&debug_msg[0], "%u,%p", my_addr, (void *)addr_ptr);
//		 send_usbmessage(&debug_msg[0], n);
//		 usb_run();

	}

	if(uwb_index > UWB_LIST_SIZE - 1)
	{
		instance_data_t *inst = &instance_data[instance];

		uint8 debug_msg[100];
		uint16 my_addr = instance_data[instance].uwbShortAdd;
		int n = sprintf((char*)&debug_msg[0], "uwb_index:%u of %u",uwb_index, instance_data[instance].uwbListLen);
		 send_usbmessage(&debug_msg[0], n);
		 usb_run();

		 //TODO solve this problem. other, (incorrect) addresses are making it into our UWB list for some reason...
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
						//TODO would be good to check if the UWB sending this message was supposed to be ranging in this timeslot
						instance_data[instance].uwbToRangeWith = uwb_index;
					}
					else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_REG || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_SUG || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_UPDATE)//only anchor if already received INF_INIT or collected regular INF messages and joined with SUG message
					{
						accept_inf = TRUE;
					}
				}
				else if (instance_data[instance].mode == DISCOVERY)
				{
					//TODO use switch case?
					if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_INIT)
					{
						if(tdma_handler.discovery_mode == WAIT_RNG_INIT)
						{
							//only accept RNG_INIT if no other active neighbors exist (and we are waiting for RNG_INIT)
							int num_active = instfindnumactiveneighbors(&instance_data[instance]);
							if(num_active <= 1)
							{
								instance_data[instance].uwbToRangeWith = uwb_index;
							}
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
						if(tdma_handler.discovery_mode == WAIT_INF_REG || tdma_handler.discovery_mode == COLLECT_INF_REG)
						{
							accept_inf = TRUE;
						}
					}
				}
			}
			else if(rxd_event == DWT_SIG_RX_BLINK)
			{
				//only accept BLINK if in DISCOVERY mode and no other active neighbors exist
				//and not waiting for inf or rng_init
				if(instance_data[instance].mode == DISCOVERY)
				{
					if(tdma_handler.discovery_mode == WAIT_INF_REG)
					{
						int num_neighbors = instfindnumactiveneighbors(&instance_data[instance]);
//						uint8 debug_msg[100];
//						int n = sprintf((char*)&debug_msg[0], "num_neighbors %d", num_neighbors);
//						send_usbmessage(&debug_msg[0], n);
//						usb_run();

						if(num_neighbors <= 1)
						{
							instance_data[instance].uwbToRangeWith = uwb_index;
						}
					}
				}
			}
		}
	}


//	n = sprintf((char*)&debug_msg[0], "uwb_index %d, uwbToRangeWith %d", uwb_index, instance_data[instance].uwbToRangeWith);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();


	int place_event = 0;


	if(rxd_event == DWT_SIG_RX_OKAY && uwb_index != 255)
	{
		//always accept.
		if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_REPORT || dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_SYNC)
		{
//			uint8 debug_msg[100];
//			 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: RNG_REPORT <- uwb_index %d", uwb_index);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();
			instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
			instance_data[instance].uwbTimeout[uwb_index] = 0;
			place_event = 1;
		}
	}

	if(accept_inf == TRUE)
	{
		//NOTE: these could get messed up if other messages trigger rxcallback before being processed by frame sync...
		//should these be folded into the dw_event?
		//TODO only do this when we need to (we are accepting and INF message)
		instance_data[instance].timeofRxCallback = time_now_us;

		//TODO only do this when we need to (we are accepting and INF message)
		uint8 sys_time_arr[5] = {0, 0, 0, 0, 0};
		dwt_readsystime(sys_time_arr);
		instance_data[instance].timeofRxCallback_dwtime = (uint64)sys_time_arr[0] + ((uint64)sys_time_arr[1] << 8) + ((uint64)sys_time_arr[2] << 16) + ((uint64)sys_time_arr[3] << 24) + ((uint64)sys_time_arr[4] << 32);


		instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
		instance_data[instance].uwbTimeout[uwb_index] = 0;
		place_event = 1;


		if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_INF_SUG)
		{
//			uint8 debug_msg[100];
//			int n = sprintf((char*)&debug_msg[0], "RX CB: DWT_SIG_RX_OKAY-%s, %llX, xxxx", get_msg_fcode_string(dw_event.msgu.frame[fcode_index]), instance_get_addr());
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();
		}




	}
	else if(uwb_index != 255 && instance_data[instance].uwbToRangeWith == uwb_index)
	{
		
		if(rxd_event == DWT_SIG_RX_OKAY)
		{
			//process RTLS_DEMO_MSG_TAG_POLL immediately.
			if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_POLL)
			{
//				uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, "RX_POLL");
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();

				instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
				instance_data[instance].uwbTimeout[uwb_index] = 0;

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

//				instance_data[instance].msg[uwb_index].messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)
//				//program option octet and parameters (not used currently)
//				instance_data[instance].msg[uwb_index].messageData[RES_R1] = 0x2; // "activity"
//				instance_data[instance].msg[uwb_index].messageData[RES_R2] = 0x0; //
//				instance_data[instance].msg[uwb_index].messageData[RES_R3] = 0x0;
//
//
//				memcpy(&instance_data[instance].msg[uwb_index].destAddr[0], &dw_event.msgu.frame[srcAddr_index], instance_data[instance].addrByteSize); //remember who to send the reply to (set destination address)
//
//				//if tof not zero, set tof, else memset 0
//				//also include next TX_ACCEPT node time (offset from now)
//				// Write calculated TOF into response message
//				memcpy(&instance_data[instance].msg[uwb_index].messageData[TOFR], &instance_data[instance].tof[uwb_index], 6); //TODO fix number of bytes...
//
//				uint8 num_active = 0;
//				for(int i = 0; i < instance_data[instance].uwbListLen; i++){
//					if(!instance_data[instance].uwbTimeout[i]){
//						num_active++;
//					}
//				}
//				memcpy(&instance_data[instance].msg[uwb_index].messageData[NTAG], &num_active, 1);
//
//				//get time till the next RX_ACCEPT
////				uint32 time_till = 0;
////				if(instance_data[instance].time_till_next_reported[uwb_index] == 0){
////					time_till =	instance_data[instance].rx_scheduler.get_time_till_next_rx_accept(&instance_data[instance].rx_scheduler);
////					instance_data[instance].time_till_next_reported[uwb_index] = 1;
////				}
////				memcpy(&instance_data[instance].msg[uwb_index].messageData[TIME_TILL], &time_till, 4);
//
//
////				uint8 debug_msg[100];
////				uint64 mtof = instance_data[instance].msg[uwb_index].messageData[TOFR];
////				int n = sprintf((char *)&debug_msg, "TOF in resp %llu", mtof);
////				send_usbmessage(&debug_msg[0], n);
////				usb_run();
//
//
////				instance_data[instance].tof[uwb_index] = 0; //clear ToF ..
////
//				instance_data[instance].msg[uwb_index].seqNum = instance_data[instance].frameSN++;

				instance_data[instance].msg.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

				memcpy(&instance_data[instance].msg.destAddr[0], &dw_event.msgu.frame[srcAddr_index], instance_data[instance].addrByteSize); //remember who to send the reply to (set destination address)
				
				instance_data[instance].msg.seqNum = instance_data[instance].frameSN++;


				//set the delayed rx on time (the final message will be sent after this delay)
//				dwt_setrxaftertxdelay((uint32)instance_data[instance].txToRxDelayAnc_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)
				 dwt_setrxaftertxdelay(0);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

				//response is expected
				instance_data[instance].wait4ack = DWT_RESPONSE_EXPECTED;

				dwt_writetxfctrl(frameLength, 0, 1);
//				dwt_writetxdata(frameLength, (uint8 *)  &instance_data[instance].msg[uwb_index], 0) ;	// write the frame data
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
				}
				//report out which message is being sent!
//				send_txmsgtousb(get_msg_fcode_string((int)instance_data[instance].msg[uwb_index].messageData[FCODE]));

//				int64 mtof = 0;
//				memcpy(&mtof, &instance_data[instance].msg[uwb_index].messageData[TOFR], sizeof(int64));
//				int64 mtof2 = 0;
//				memcpy(&mtof2, &instance_data[instance].tof[uwb_index], sizeof(int64));
//				uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, ".instance_data[instance].uwbToRangeWith %d, tof[uwb_index] %lld, .msg[uwb_index].messageData[TOFR] %lld",instance_data[instance].uwbToRangeWith, mtof2, mtof);
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();


//				uint32 time_till_cpy = 0;
//				memcpy(&time_till_cpy, &instance_data[instance].msg[uwb_index].messageData[TIME_TILL], sizeof(uint32));
//
//				uint8 debug_msg[200];
//				int n = 0;
//				n = sprintf((char*)&debug_msg[0], "time_till %lu, memcpy time_till %lu", time_till, time_till_cpy);
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();

				instance_data[instance].tof[uwb_index] = 0; //clear ToF ..


//				uint8 debug_msg[100];
//				 n = sprintf((char*)&debug_msg[0], "RX TAG_POLL ACCEPTED ANCH_RESP sent <- uwb %d", uwb_index);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
			}
			else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_ANCH_RESP)
			{
//				uint32 response_time = portGetTickCnt() - instance_data[instance].range_start;
				instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
				instance_data[instance].uwbTimeout[uwb_index] = 0;
//				uint8 debug_msg[100];
//				 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: ANCH_RESP <- uwb_index %d", uwb_index);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
			}
			else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_INIT)
			{
				instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
				instance_data[instance].uwbTimeout[uwb_index] = 0;
//				uint8 debug_msg[100];
//				 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: RNG_INIT <- uwb_index %d", uwb_index);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
			}
			else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_TAG_FINAL)
			{
				instance_data[instance].lastCommTimeStamp[uwb_index] = time_now;
				instance_data[instance].uwbTimeout[uwb_index] = 0;
//				uint8 debug_msg[100];
//				 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: TAG_FINAL <- uwb_index %d", uwb_index);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
			}
//			else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_RNG_REPORT)
//			{
//				instance_data[instance].lastCommTimeStamp[uwb_index] = portGetTickCnt();
//				instance_data[instance].uwbTimeout[uwb_index] = 0;
//				uint8 debug_msg[100];
//				 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: RNG_REPORT <- uwb_index %d", uwb_index);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
//			}
//			else if(dw_event.msgu.frame[fcode_index] == RTLS_DEMO_MSG_SYNC)
//			{
//
//			}


			place_event = 1;
		}
		else if (rxd_event == DWT_SIG_RX_BLINK)
		{
//			uint8 debug_msg[100];
//			 int n = sprintf((char*)&debug_msg[0], "RX CALLBACK ACCEPTED: BLINK <- uwb %d", uwb_index);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();
			
			place_event = 1;
		}
	}

	// TODO figure out a better way to do this, I'd like to keep it where it was if possible
	// doing it here because it needs to be toggled before toggling the rx buffer pointer
	// Toggle the Host side Receive Buffer Pointer
//	dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
        
//	uint8 buff = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3);
//	uint8 debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "RX BUFFER TOGGLE: %d", buff);
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();

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
		// instance_rxerrorcallback(rxd);
		
//		uint8 debug_msg[100];
//		 int n = sprintf((char*)&debug_msg[0], "instancerxon called from !event_placed :");
//		 send_usbmessage(&debug_msg[0], n);
//		 usb_run();

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
		instance_data[instance].dweventIdxOut = 0;

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
//		uint64 time_now_us = portGetTickCntMicro();

		done = testapprun(&instance_data[instance], &tdma_handler, message) ; // run the communications application

//		uint64 duration = get_dt64(time_now_us, portGetTickCntMicro());
//		uint8 debug_msg[100];
//		int n = sprintf((char *)&debug_msg, "TAR, duration, %llu", duration);
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();

		//we've processed message
		message = 0;
	}

	//check if lastCommTimeStamp has expired for any of the uwbs in our list
	for(int i=1; i < instance_data[instance].uwbListLen; i++)//0 reserved for self, timeout not applicable
	{
		//TODO could also have a timer to put into DISCOVERY if we haven't spoken to anyone in a while
		uint32 time_now = portGetTickCnt();

		uint32 delta_t = get_dt32(instance_data[instance].lastCommTimeStamp[i], time_now);
		if(delta_t > UWB_COMM_TIMEOUT) //TODO handle number wrapping
		{
//			if(instance_data[instance].uwbListType[i] == UWB_LIST_NEIGHBOR) //what about hidden?
			if(tdma_handler.uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR) //what about hidden?
			{
//				instance_data[instance].uwbListType[i] = UWB_LIST_INACTIVE; //TODO release TDMA slots as well
				tdma_handler.uwbListTDMAInfo[i].connectionType = UWB_LIST_INACTIVE; //TODO release TDMA slots as well
				instance_data[instance].uwbTimeout[i] = 1;

				//NEW
				//if no more active neighbors exist, transition to DISCOVERY
				uint8 numNeighbors = instfindnumactiveneighbors(&instance_data[instance]);
				if(numNeighbors <= 0)
				{
					tdma_handler.tdma_free_all_slots(&tdma_handler);



//					uint8 debug_msg[100];
//					int n = sprintf((char *)&debug_msg, "inst_processtxrxtimeout(inst) after free_all_slots");
//					send_usbmessage(&debug_msg[0], n);
//					usb_run();

					instance_data[instance].mode = DISCOVERY;						//TODO clear the TDMA slot information!
					tdma_handler.discoveryStartTime = portGetTickCnt();
					tdma_handler.enter_discovery_mode(&tdma_handler);
					inst_processtxrxtimeout(&instance_data[instance]);

					instance_data[instance].canPrintInfo = 0;
					instance_data[instance].ranging = 0;
				}
				else
				{
					//TODO check if able to reconfigure tdma assignements. shrink frame size, reassign emptied slots, etc
					tdma_handler.uwblist_free_slots(&tdma_handler, i);
				}


//				 uint8 debug_msg[100];
//				 int n = sprintf((char*)&debug_msg[0], "TIMEOUT: uwb %i", i);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();
			}

			if(instance_data[instance].uwbToRangeWith == i)
			{	
				instance_data[instance].uwbToRangeWith = 255;	
				//NOTE this might need to be changed for TAG operations
				if(instance_data[instance].mode == ANCHOR || instance_data[instance].mode == DISCOVERY) //TODO maybe send to TA_MODE_SELECT?
				{
					instance_data[instance].testAppState = TA_RXE_WAIT;
				}
				else
				{
//					 uint8 debug_msg[100];
//					 int n = sprintf((char*)&debug_msg[0], "instance_run(void) TIMEOUT");
//					 send_usbmessage(&debug_msg[0], n);
//					 usb_run();
					instance_data[instance].testAppState = TA_TX_SELECT;
				}
			}
		}
		
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
//        	uint8 debug_msg[200];
//			int n = 0;
//			n = sprintf((char*)&debug_msg[0], "instance timer expired");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

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


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
