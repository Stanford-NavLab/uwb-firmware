/*! ----------------------------------------------------------------------------
 *  @file    instance.c
 *  @brief   DecaWave application level message exchange for ranging demo
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
#include "deca_spi.h"
#include "deca_regs.h"


#include "lib.h"
#include "instance.h"

//#include "llist.h"



extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

//TODO remove
//NOTE: my added USB debug values/functions
#define USB_DEBUG_BUFF_LEN (100)
uint8 usbdebugdata[USB_DEBUG_BUFF_LEN]; //data to be sent over usb for debug purposes
int usbdebugdata_size = 0;     
uint8 usbdebugdataprev[USB_DEBUG_BUFF_LEN]; //previous message sent
int usbdebugdataprev_size = 0;
uint8 usbrxdebugdata[USB_DEBUG_BUFF_LEN];
int usbrxdebugdata_size = 0;
uint8 usbrxdebugdataprev[USB_DEBUG_BUFF_LEN];
int usbrxdebugdataprev_size = 0;        
uint8 usbtxdebugdata[USB_DEBUG_BUFF_LEN];
int usbtxdebugdata_size = 0;
uint8 usbtxdebugdataprev[USB_DEBUG_BUFF_LEN];
int usbtxdebugdataprev_size = 0;        
 
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------



// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the message/frame header bytes
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigframeheader(instance_data_t *inst)
{
	//configure ranging message
	inst->msg.panID[0] = (inst->panID) & 0xff;
	inst->msg.panID[1] = inst->panID >> 8;

	//set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
	inst->msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
#if (USING_64BIT_ADDR==1)
		//source/dest addressing modes and frame version
	inst->msg.frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
	inst->msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif


	//configure RNG_INIT message
    inst->rng_initmsg.frameCtrl[0] = 0x41;

#if (USING_64BIT_ADDR == 1)
    inst->rng_initmsg.frameCtrl[1] = 0xCC;
#else
    inst->rng_initmsg.frameCtrl[1] = 0x8C;
#endif
    inst->rng_initmsg.panID[0] = (inst->panID) & 0xff;
    inst->rng_initmsg.panID[1] = inst->panID >> 8;


	//configure INF message
	inst->inf_msg.panID[0] = (inst->panID) & 0xff;
	inst->inf_msg.panID[1] = inst->panID >> 8;

	//set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
	inst->inf_msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
#if (USING_64BIT_ADDR==1)
	//source/dest addressing modes and frame version
	inst->inf_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
	inst->inf_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif


	//configure RNG_REPORT
	inst->report_msg.panID[0] = (inst->panID) & 0xff;
	inst->report_msg.panID[1] = inst->panID >> 8;

	//set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
	inst->report_msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
#if (USING_64BIT_ADDR==1)
	//source/dest addressing modes and frame version
	inst->report_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
	inst->report_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif


	//configure SYNC message
	inst->sync_msg.panID[0] = (inst->panID) & 0xff;
	inst->sync_msg.panID[1] = inst->panID >> 8;

	//set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
	inst->sync_msg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
#if (USING_64BIT_ADDR==1)
	//source/dest addressing modes and frame version
	inst->sync_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
	inst->sync_msg.frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif


	//configure BLINK
	//blink frames with IEEE EUI-64 tag ID
	inst->blinkmsg.frameCtrl = 0xC5 ;

}


// -------------------------------------------------------------------------------------------------------------------
//
// function to construct the fixed portions of the message definitions
//
// -------------------------------------------------------------------------------------------------------------------
//
void instanceconfigmessages(instance_data_t *inst)
{
	//initialize ranging message
	//set source address into the message structure
	memcpy(&inst->msg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);

	//initialize RNG_INIT message
	//set source address into the message structure
	memcpy(&inst->rng_initmsg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
	inst->rng_initmsg.messageData[FCODE] = RTLS_DEMO_MSG_RNG_INIT;

	//configure INF message
	uint16 broadcast_address = BROADCAST_ADDRESS;
	memcpy(&inst->inf_msg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
	memcpy(&inst->inf_msg.destAddr[0], &broadcast_address, 2);
	inst->inf_msg.messageData[FCODE] = 0; //message function code (specifies if message is a poll, response or other...)

	//configure RNG_REPORT
	memcpy(&inst->report_msg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
	memcpy(&inst->report_msg.destAddr[0], &broadcast_address, 2);
	inst->report_msg.messageData[FCODE] = RTLS_DEMO_MSG_RNG_REPORT; //message function code (specifies if message is a poll, response or other...)

	//configure SYNC message
	memcpy(&inst->sync_msg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
	memcpy(&inst->sync_msg.destAddr[0], &broadcast_address, 2);
	inst->sync_msg.messageData[FCODE] = RTLS_DEMO_MSG_SYNC; //message function code (specifies if message is a poll, response or other...)

	//configure BLINK message
	memcpy(&inst->blinkmsg.tagID[0], &inst->eui64[0], ADDR_BYTE_SIZE_L);
}



// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
// -------------------------------------------------------------------------------------------------------------------
//
void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime)
{
    if (delayed)
    {
        uint32 dtime;
        dtime =  (uint32) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

    int dwt_rx_enable_return = dwt_rxenable(delayed);
    inst->lateRX -= dwt_rx_enable_return;

} // end instancerxon()


int instancesendpacket(uint16 length, uint8 txmode, uint32 dtime)
{
    int result = 0;

    dwt_writetxfctrl(length, 0, 1);
    if(txmode & DWT_START_TX_DELAYED)
    {
        dwt_setdelayedtrxtime(dtime) ;
    }

    //begin delayed TX of frame
    if (dwt_starttx(txmode))  // delayed start was too late
    {
        result = 1; //late/error
    }

    return result;                                              // state changes

}

//debug helper function to print the testAppState
const char* get_inst_states_string(enum inst_states state)
{
    switch (state)
    {
        case TA_INIT : return "TA_INIT";
        case TA_TXE_WAIT : return "TA_TXE_WAIT";       
        case TA_TXPOLL_WAIT_SEND : return "TA_TXPOLL_WAIT_SEND";       
        case TA_TXFINAL_WAIT_SEND : return "TA_TXFINAL_WAIT_SEND";  
        case TA_TXRESPONSE_WAIT_SEND : return "TA_TXRESPONSE_WAIT_SEND";
        case TA_TX_WAIT_CONF : return "TA_TX_WAIT_CONF";
        case TA_RXE_WAIT : return "TA_RXE_WAIT";
        case TA_RX_WAIT_DATA : return "TA_RX_WAIT_DATA";
        case TA_SLEEP_DONE : return "TA_SLEEP_DONE";
        case TA_TXINF_WAIT_SEND : return "TA_TXINF_WAIT_SEND";
        case TA_TXBLINK_WAIT_SEND : return "TA_TXBLINK_WAIT_SEND";
        case TA_TXRANGINGINIT_WAIT_SEND : return "TA_TXRANGINGINIT_WAIT_SEND";
        case TA_TX_SELECT : return "TA_TX_SELECT";
        case TA_TXREPORT_WAIT_SEND : return "TA_TXREPORT_WAIT_SEND";
        case TA_TXSUG_WAIT_SEND : return "TA_TXSUG_WAIT_SEND";
        default: return "NONE";
    }
}

//debug helper function to print the mode
const char* get_instanceModes_string(enum instanceModes mode)
{
    switch (mode)
    {
    	case DISCOVERY : return "DISCOVERY";
        case TAG : return "TAG";       
        case ANCHOR : return "ANCHOR";
        case NUM_MODES : return "NUM_MODES";
        default: return "NONE";
    }
}

//debug helper function to print the mode
const char* get_discovery_modes_string(enum discovery_modes mode)
{
    switch (mode)
    {
    	case WAIT_INF_REG : return "WAIT_INF_REG";
    	case COLLECT_INF_REG : return "COLLECT_INF_REG";
        case WAIT_INF_INIT : return "WAIT_INF_INIT";
        case WAIT_RNG_INIT : return "WAIT_RNG_INIT";
        case WAIT_SEND_SUG : return "WAIT_SEND_SUG";
        case SEND_SUG : return "SEND_SUG";
        case EXIT : return "EXIT";
        default: return "NONE";
    }
}


char* get_msg_fcode_string(int fcode)
{
    if (fcode == (int)RTLS_DEMO_MSG_RNG_INIT)
    {
        return "RTLS_DEMO_MSG_RNG_INIT";
    }
    else if(fcode == (int)RTLS_DEMO_MSG_TAG_POLL)
    {
        return "RTLS_DEMO_MSG_TAG_POLL";
    }
    else if(fcode == (int)RTLS_DEMO_MSG_ANCH_RESP)
    {
        return "RTLS_DEMO_MSG_ANCH_RESP";
    }
    else if(fcode == (int)RTLS_DEMO_MSG_TAG_FINAL)
    {
        return "RTLS_DEMO_MSG_TAG_FINAL";
    }
    else if(fcode == (int)RTLS_DEMO_MSG_INF_REG)
    {
    	return "RTLS_DEMO_MSG_INF_REG";
    }
    else if(fcode == (int)RTLS_DEMO_MSG_INF_INIT)
	{
		return "RTLS_DEMO_MSG_INF_INIT";
	}
    else if(fcode == (int)RTLS_DEMO_MSG_INF_SUG)
	{
		return "RTLS_DEMO_MSG_INF_SUG";
	}
	else if(fcode == (int)RTLS_DEMO_MSG_INF_UPDATE)
	{
		return "RTLS_DEMO_MSG_INF_UPDATE";
	}
    else if(fcode == (int)RTLS_DEMO_MSG_RNG_REPORT)
	{
		return "RTLS_DEMO_MSG_RNG_REPORT";
	}
    else
    {
        return "NONE";
    }
}

void send_statetousb(instance_data_t *inst, struct TDMAHandler *tdma_handler)
{
    
    usbdebugdata_size = sprintf((char*)&usbdebugdata[0], "%s , %s , %s , %s", get_inst_states_string(inst->testAppState), get_inst_states_string(inst->previousState), get_inst_states_string(inst->nextState), get_instanceModes_string(inst->mode));
     
    if (memcmp(usbdebugdataprev, usbdebugdata, usbdebugdata_size) != 0 || usbdebugdata_size != usbdebugdataprev_size)
    {
        send_usbmessage(&usbdebugdata[0], usbdebugdata_size);
        usb_run();
        usbdebugdataprev_size = usbdebugdata_size;
        memcpy(usbdebugdataprev, usbdebugdata, usbdebugdata_size);
    }
}

void send_rxmsgtousb(char *data)
{
    usbrxdebugdata_size = sprintf((char*)&usbrxdebugdata[0], "%s", data);
     
    if (memcmp(usbrxdebugdataprev, usbrxdebugdata, usbrxdebugdata_size) != 0 || usbrxdebugdata_size != usbrxdebugdataprev_size)
    {
        send_usbmessage(&usbrxdebugdata[0], usbrxdebugdata_size);
        usb_run();
        usbrxdebugdataprev_size = usbrxdebugdata_size;
        memcpy(usbrxdebugdataprev, usbrxdebugdata, usbrxdebugdata_size);
    }
}

void send_txmsgtousb(char *data)
{
    usbtxdebugdata_size = sprintf((char*)&usbtxdebugdata[0], "TX message: %s", data);
     
    send_usbmessage(&usbtxdebugdata[0], usbtxdebugdata_size);
    usb_run();
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag or Anchor use the same state machine)
//
// -------------------------------------------------------------------------------------------------------------------
//
int testapprun(instance_data_t *inst, struct TDMAHandler *tdma_handler, int message)
{
    int done = INST_NOT_DONE_YET;

    if(tdma_handler->slot_transition(tdma_handler))//TODO move these types of things to the end????
    {
		message = 0;
    }

//    send_statetousb(inst, tdma_handler); //TODO remove

    tdma_handler->check_discovery_mode_expiration(tdma_handler);


    if(message ==  DWT_SIG_TX_DONE && inst->testAppState != TA_TX_WAIT_CONF)
	{
		//It is possible to get an interrupt which takes the UWB out of TX_WAIT_CONF
		//before we have to process it a DWT_SIG_TX_DONE event.
    	//Clear the event in this case
		instance_getevent(11);
		done = INST_DONE_WAIT_FOR_NEXT_EVENT;
	}

    switch (inst->testAppState)
    {
        case TA_INIT :
        {
            switch (inst->mode)
            {
                case DISCOVERY:
                {
                    int mode = 0;
                    dwt_forcetrxoff();

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_RSVD_EN);
					inst->frameFilteringEnabled = 1 ;
					dwt_seteui(inst->eui64);
					dwt_setpanid(inst->panID);

					//seed random number generator with our 64-bit address
				    uint64 seed = 0;
				    seed |= (uint64) inst->eui64[0];
				    seed |= (uint64) inst->eui64[1] << 8;
				    seed |= (uint64) inst->eui64[2] << 16;
				    seed |= (uint64) inst->eui64[3] << 24;
				    seed |= (uint64) inst->eui64[4] << 32;
				    seed |= (uint64) inst->eui64[5] << 40;
				    seed |= (uint64) inst->eui64[6] << 48;
				    seed |= (uint64) inst->eui64[7] << 56;
				    srand(seed);


					inst->uwbShortAdd = inst->eui64[0] + (inst->eui64[1] << 8);//TODO use a hashing algorithm

#if (USING_64BIT_ADDR==0)
					dwt_setaddress16(inst->uwbShortAdd);
					memcpy(&inst->uwbList[0][0], &inst->uwbShortAdd, inst->addrByteSize);
#else
					memcpy(&inst->uwbList[0][0], &inst->eui64, inst->addrByteSize);
#endif
					inst->uwbListLen = 1;
					tdma_handler->uwbListTDMAInfo[0].connectionType = UWB_LIST_SELF;

                    mode = (DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

                    if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
                    {
                        mode |= DWT_LOADOPSET;
                    }
#if (DEEP_SLEEP == 1)
                    if (inst->sleepingEabled)
                        dwt_configuresleep(mode, DWT_WAKE_WK|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif

                    instanceconfigframeheader(inst);
                    instanceconfigmessages(inst);

                    // First time listening, do not delay RX
					 dwt_setrxaftertxdelay(0);

					//change to next state - wait to receive a message
					tdma_handler->discoveryStartTime = portGetTickCnt();
					tdma_handler->last_blink_time = portGetTickCnt();
					inst->testAppState = TA_RXE_WAIT ;

					dwt_setrxtimeout(0);
					inst->wait4ack = 0;
					inst->canPrintInfo = TRUE;

					//TODO revisit
					// If we are using long response delays, disable sleep.
//					if (inst->resp_dly_us[RESP_DLY_POLL] >= LONG_RESP_DLY_LIMIT_US
//						|| inst->resp_dly_us[RESP_DLY_ANCH_RESP] >= LONG_RESP_DLY_LIMIT_US)
//					{
//						inst->sleepingEabled = 0;
//					}

                }
                break;
                default:
                break;
            }
            break;
        }// end case TA_INIT
        case TA_TX_SELECT :
        {
          	//select a TX action, return TRUE if we should move on to another state
        	if(tdma_handler->tx_select(tdma_handler) == TRUE)
        	{
        		dwt_forcetrxoff();

        	}
        	else
        	{
        		done = INST_DONE_WAIT_FOR_NEXT_EVENT;
        	}

            break;
        }// end case TA_TX_SELECT
        case TA_SLEEP_DONE :
        {
            event_data_t* dw_event = instance_getevent(10); //clear the event from the queue
            // waiting for timeout from application to wakeup IC
            if (dw_event->type != DWT_SIG_RX_TIMEOUT)
            {
                // if no pause and no wake-up timeout continue waiting for the sleep to be done.
                done = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                break;
            }

            done = INST_NOT_DONE_YET;
            inst->goToSleep = 0;
            inst->testAppState = inst->nextState;
            inst->nextState = TA_INIT; //clear
            inst->instanceTimerTimeSaved = inst->instanceTimerTime = portGetTickCnt(); //set timer base
#if (DEEP_SLEEP == 1)
            if (inst->sleepingEabled)
            {
                //wake up device from low power mode
                led_on(LED_PC9);

                port_wakeup_dw1000_fast();

                led_off(LED_PC9);

                //this is platform dependent - only program if DW EVK/EVB
                dwt_setleds(1);

                //MP bug - TX antenna delay needs reprogramming as it is not preserved after DEEP SLEEP
                dwt_settxantennadelay(inst->txAntennaDelay) ;
            }
            instancesetantennadelays();
#endif
            //TODO should this be here? or only if DEEP_SLEEP is enabled (above) putting only above for now

            break;
        }
        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
        {
        	//TODO rework sleep functionality
        	//should sleep between blinks?
        	//should blink when not own slot?
        	//should sleep after everything is done in own slot?
        	//does sleep still allow RX?

            //if we are scheduled to go to sleep before next sending then sleep first.
//            if(((inst->nextState == TA_TXPOLL_WAIT_SEND)
//                || (inst->nextState == TA_TXBLINK_WAIT_SEND) || (inst->nextState == TA_TX_SELECT))
			if(inst->nextState == TA_TX_SELECT
                    && (inst->goToSleep)  //go to sleep before sending the next poll
                    )
            {
                //the app should put chip into low power state and wake up in tagSleepTime_ms time...
                //the app could go to *_IDLE state and wait for uP to wake it up...
                done = INST_DONE_WAIT_FOR_NEXT_EVENT_TO; //don't sleep here but kick off the TagTimeoutTimer (instancetimer)
                inst->testAppState = TA_SLEEP_DONE;

//                inst->canPrintInfo = 1;

#if (DEEP_SLEEP == 1)
                if (inst->sleepingEabled)
                {
                    //put device into low power mode
                    dwt_entersleep(); //go to sleep
                }
#endif
                
                //inst->deviceissleeping = 1; //this is to stop polling device status register (as it will wake it up)
            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = TA_INIT; //clear
            }
            break ;
        }// end case TA_TXE_WAIT
        case TA_TXBLINK_WAIT_SEND :
		{
			int psduLength = BLINK_FRAME_LEN_BYTES;

            //blink frames with IEEE EUI-64 tag ID
			inst->blinkmsg.seqNum = inst->frameSN++;


			dwt_setrxtimeout(0);
			dwt_setrxaftertxdelay(0);
			inst->wait4ack = DWT_RESPONSE_EXPECTED;

        	dwt_writetxdata(psduLength, (uint8 *)  (&inst->blinkmsg), 0) ; // write the frame data
			if(instancesendpacket(psduLength, DWT_START_TX_IMMEDIATE | inst->wait4ack, 0))
			{
				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new blink or poll message
				inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			}
			else
			{
				uint8 debug_msg[100];//TODO remove
//					int n = sprintf((char *)&debug_msg, "TX_BLINK,%llX,NULL", instance_get_addr());
				int n = sprintf((char *)&debug_msg, "TX_BLINK,%04llX,NULL", instance_get_addr());
				send_usbmessage(&debug_msg[0], n);
				usb_run();

				inst->goToSleep = 1; //go to Sleep after this blink
				inst->testAppState = TA_TX_WAIT_CONF ; // wait confirmation
				inst->previousState = TA_TXBLINK_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

				inst->blink_start = portGetTickCnt(); //TODO remove
				inst->timeofTx = portGetTickCnt();
				//TODO check this duration!
				inst->txDoneTimeoutDuration = inst->durationBlinkTxDoneTimeout_ms;
				tdma_handler->last_blink_time = portGetTickCnt();
				tdma_handler->blinkPeriodRand = (uint32)rand()%BLINK_PERIOD_RAND_MS;
			}

			break ;
		}// end case TA_TXBLINK_WAIT_SEND
        case TA_TXRANGINGINIT_WAIT_SEND :
        {
        	int psduLength = RNG_INIT_FRAME_LEN_BYTES;
//            int psduLength = RNG_INIT_MSG_LEN; //TODO remove
//#if (USING_64BIT_ADDR == 1)
//            psduLength += FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
//#else
//            psduLength += FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;
//#endif

            inst->rng_initmsg.seqNum = inst->frameSN++;

            dwt_setrxaftertxdelay(0);
            dwt_setrxtimeout((uint16)0);
            inst->wait4ack = DWT_RESPONSE_EXPECTED;

            dwt_writetxdata(psduLength, (uint8 *)  &inst->rng_initmsg, 0) ; // write the frame data
			if(instancesendpacket(psduLength, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, inst->delayedReplyTime))
			{
				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new blink or poll message
				inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
				inst->lateTX++;
			}
			else
			{
				inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
				inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout

				uint32 time_now = portGetTickCnt();
				inst->timeofTx = time_now;
				//TODO check below
				inst->txDoneTimeoutDuration = inst->durationRngInitTxDoneTimeout_ms;
				tdma_handler->set_discovery_mode(tdma_handler, WAIT_INF_INIT, time_now);
			}

			break;
        }
        case TA_TXINF_WAIT_SEND :
		{
			//NOTE: handles INF_SUG, INF_INIT, INF_UPDATE, and INF_REG
			int psduLength = tdma_handler->infMessageLength;

			inst->inf_msg.seqNum = inst->frameSN++;
			//update time since frame start!
			tdma_handler->update_inf_tsfs(tdma_handler);

			dwt_setrxtimeout((uint16)0);//no timeout (keep RX on until instructed otherwise)
			inst->wait4ack = 0;

			dwt_writetxdata(psduLength, (uint8 *)  &inst->inf_msg, 0) ; // write the frame data
			if(instancesendpacket(psduLength, DWT_START_TX_IMMEDIATE | inst->wait4ack, 0))
			{
				//get the message FCODE
				uint8 fcode;
				memcpy(&fcode, &inst->inf_msg.messageData[FCODE], sizeof(uint8));

				if(fcode == RTLS_DEMO_MSG_INF_SUG)
				{
					tdma_handler->set_discovery_mode(tdma_handler, WAIT_SEND_SUG, portGetTickCnt());
				}

				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT;
			}
			else
			{
				inst->testAppState = TA_TX_WAIT_CONF ; //TODO should only do this if we don't have a problem with the INF send...
				inst->previousState = TA_TXINF_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

				inst->timeofTx = portGetTickCnt();
				//TODO check below (INF message changes size, recalculate every time!)
				uint64 margin_us = 1000;
				uint64 framelength_us = instance_getmessageduration_us(psduLength);
				inst->txDoneTimeoutDuration = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + framelength_us + margin_us, 1000);			//tx cmd to tx cb
			}

			break;
		}
        case TA_TXPOLL_WAIT_SEND :
        {
        	int psduLength = POLL_FRAME_LEN_BYTES;
//        	int psduLength = 0;//TODO remove
//#if (USING_64BIT_ADDR==1)
//            psduLength = TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
//#else
//            psduLength = TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
//#endif

            inst->msg.seqNum = inst->frameSN++;
			inst->msg.messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL; //message function code (specifies if message is a poll, response or other...)
			memcpy(&inst->msg.destAddr[0], &inst->uwbList[inst->uwbToRangeWith], inst->addrByteSize);

            //enable rx on after tx
            dwt_setrxaftertxdelay(0);
            dwt_setrxtimeout(inst->durationPollTimeout);
            inst->wait4ack = DWT_RESPONSE_EXPECTED;

            inst->cmd_poll_us = portGetTickCntMicro();//TODO remove

            dwt_writetxdata(psduLength, (uint8 *)  &inst->msg, 0) ; // write the frame data
            if(instancesendpacket(psduLength, DWT_START_TX_IMMEDIATE | inst->wait4ack, 0))
			{
            	inst->tx_poll = FALSE;
            	//failed
				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT;

				inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			}
			else
			{
				//succeded
				uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, "TX_POLL,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
				int n = sprintf((char *)&debug_msg, "TX_POLL,%04llX,%04llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
				send_usbmessage(&debug_msg[0], n);
				usb_run();

				inst->tx_poll = TRUE;
				inst->testAppState = TA_TX_WAIT_CONF ;
				inst->previousState = TA_TXPOLL_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

				inst->range_start = portGetTickCnt(); //TODO remove
				inst->timeofTx = portGetTickCnt();
				//TODO check below
				inst->txDoneTimeoutDuration = inst->durationPollTxDoneTimeout_ms;
				inst->canPrintInfo = FALSE; //don't print to LCD or USB while ranging
			}

            break;
        }
        case TA_TXREPORT_WAIT_SEND :
		{
			int psduLength = REPORT_FRAME_LEN_BYTES;
//			int psduLength = 0;
//#if (USING_64BIT_ADDR==1)
//			psduLength = RNG_REPORT_MSG_LEN_LONG + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;
//#else
//			psduLength = RNG_REPORT_MSG_LEN_SHORT + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
//#endif

			// Write calculated TOF into response message
			memcpy(&inst->report_msg.messageData[REPORT_TOF], &inst->tof[inst->uwbToRangeWith], 6);
			memcpy(&inst->report_msg.messageData[REPORT_ADDR], &inst->uwbList[inst->uwbToRangeWith], inst->addrByteSize);
			inst->report_msg.seqNum = inst->frameSN++;

			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(0);
			inst->wait4ack = 0;

			dwt_writetxdata(psduLength, (uint8 *)  &inst->report_msg, 0) ; // write the frame data
			if(instancesendpacket(psduLength, DWT_START_RX_IMMEDIATE, 0))
			{
				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT;
				inst->wait4ack = 0;
			}
			else
			{
				inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
				inst->previousState = TA_TXREPORT_WAIT_SEND;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out  (set below)

				inst->timeofTx = portGetTickCnt();
				//TODO check below
				inst->txDoneTimeoutDuration = inst->durationReportTxDoneTimeout_ms;
			}

			inst->canPrintInfo = TRUE; //ranging exchange complete, allow print to USB and LCD

			break;
		}
        case TA_TX_WAIT_CONF :
        {
            event_data_t* dw_event = instance_getevent(11); //get and clear this event

            //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
            //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
            //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
            if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
            {
                if(dw_event->type == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
                {
                    inst->gotTO = 1;
                }

                //sometimes the DW1000 tx callback (TXFRS) fails to trigger and the the SYS_STATE register
                //reads IDLE for for PMSC, RX, and TX so we need another way to timeout since RX FWTO won't be triggered.
                uint32 dt = get_dt32(inst->timeofTx, portGetTickCnt());
                if(dt > inst->txDoneTimeoutDuration) //duration set at time of tx
                {
                	inst->gotTO = 1;
                }

                done = INST_DONE_WAIT_FOR_NEXT_EVENT;

                if(inst->gotTO == 0)
                {
                	break;
                }
            }

            done = INST_NOT_DONE_YET;

            if (inst->gotTO) //timeout
			{
            	inst_processtxrxtimeout(inst);
				inst->gotTO = 0;
				inst->wait4ack = 0 ; //clear this

				break;
			}
            else
            {
                if(inst->previousState == TA_TXINF_WAIT_SEND)
                {
        			//get the message FCODE
        			uint8 fcode;
        			memcpy(&fcode, &inst->inf_msg.messageData[FCODE], sizeof(uint8));

                	//exit discovery mode if we successfully send INF_SUG
					if(fcode == RTLS_DEMO_MSG_INF_SUG)
					{
						tdma_handler->set_discovery_mode(tdma_handler, EXIT, portGetTickCnt());
						inst->mode = ANCHOR;
					}

					//if we successfully send out INF_INIT, INF_SUG, or INF_UPDATE, switch to INF_REG
					if(fcode == RTLS_DEMO_MSG_INF_INIT ||
					   fcode == RTLS_DEMO_MSG_INF_SUG ||
					   fcode == RTLS_DEMO_MSG_INF_UPDATE)
					{
						fcode = RTLS_DEMO_MSG_INF_REG;
						memcpy(&inst->inf_msg.messageData[FCODE], &fcode, sizeof(uint8));
					}
                }

                inst->testAppState = TA_RXE_WAIT ;       // After sending, tag expects response/report, anchor waits to receive a final/new poll
                
                //fall into the next case (turn on the RX)
                message = 0;
            } 
        }// end case TA_TX_WAIT_CONF
        case TA_RXE_WAIT :
        {
            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
                //turn RX on
                instancerxon(inst, 0, 0) ;   // turn RX on, without delay
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

			//we are going to use anchor/tag timeout
			done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO

            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it
            inst->rxCheckOnTime = portGetTickCnt();


            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) 
            {
                break;
            }
        }
        case TA_RX_WAIT_DATA :
        {     
            // Wait RX data
            switch (message)
            {
                case DWT_SIG_RX_BLINK :
                {
                    event_data_t* dw_event = instance_getevent(12); //get and clear this event
                    
					if(inst->mode == DISCOVERY)
					{
                        //add a small random number to this to reduce chance of collisions
                        inst->delayedReplyTime = (dw_event->timeStamp + inst->rnginitReplyDelay + convertmicrosectodevicetimeu(rand()%RANGE_INIT_RAND_US)) >> 8 ;  // time we should send the blink response
                        
                        //set destination address
                        memcpy(&inst->rng_initmsg.destAddr[0], &(dw_event->msgu.rxblinkmsg.tagID[0]), BLINK_FRAME_SOURCE_ADDRESS); //remember who to send the reply to

                        inst->testAppState = TA_TXE_WAIT;
                        inst->nextState = TA_TXRANGINGINIT_WAIT_SEND ;

                    }
                    else //not initiating ranging - continue to receive
                    {
                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                        done = INST_NOT_DONE_YET;
                    }

                    break;
                }
                case DWT_SIG_RX_OKAY :
                {
                    //if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used

                    event_data_t* dw_event = instance_getevent(15); //get and clear this event
                    uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
                    int fcode = 0;
                    int fn_code = 0;
                    uint8 *messageData;

                    // 16 or 64 bit addresses
                    switch(dw_event->msgu.frame[1])
                    {
                        case 0xCC:
                            memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ll.sourceAddr[0]), ADDR_BYTE_SIZE_L);
                            fn_code = dw_event->msgu.rxmsg_ll.messageData[FCODE];
                            messageData = &dw_event->msgu.rxmsg_ll.messageData[0];
                            break;
                        case 0xC8:
                            memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_sl.sourceAddr[0]), ADDR_BYTE_SIZE_L);
                            fn_code = dw_event->msgu.rxmsg_sl.messageData[FCODE];
                            messageData = &dw_event->msgu.rxmsg_sl.messageData[0];
                            break;
                        case 0x8C:
                            memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ls.sourceAddr[0]), ADDR_BYTE_SIZE_S);
                            fn_code = dw_event->msgu.rxmsg_ls.messageData[FCODE];
                            messageData = &dw_event->msgu.rxmsg_ls.messageData[0];
                            break;
                        case 0x88:
                            memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
                            fn_code = dw_event->msgu.rxmsg_ss.messageData[FCODE];
                            messageData = &dw_event->msgu.rxmsg_ss.messageData[0];
                            break;
                    }
                    
                    fcode = fn_code;

                    switch(fcode)
                    {
                        case RTLS_DEMO_MSG_RNG_INIT:
                        {
                        	//NOTE: WAIT_RNG_INIT checked in RX callback

                        	uint8 debug_msg[100];
                        	int n = sprintf((char *)&debug_msg, "BLINK_COMPLETE,%04llX,%04llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
                        	send_usbmessage(&debug_msg[0], n);
							usb_run();

							uint32 time_now = portGetTickCnt();
                            tdma_handler->build_new_network(tdma_handler);
                            tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_INIT);
							tdma_handler->set_discovery_mode(tdma_handler, EXIT, time_now);

							inst->testAppState = TA_TX_SELECT;
							inst->mode = TAG;
                            inst->goToSleep = 0; //don't go to sleep - start ranging instead and then sleep after 1 range is done or poll times out
                            inst->instanceTimerTimeSaved = inst->instanceTimerTime = portGetTickCnt(); //set timer base
                        
                            break; 
                        } //RTLS_DEMO_MSG_RNG_INIT
                        case RTLS_DEMO_MSG_SYNC :
                        {
                        	uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);
                        	uint8 framelength;
							uint64 timeSinceFrameStart_us = 0;
							memcpy(&framelength, &messageData[SYNC_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[SYNC_TSFS], 6);

							if(inst->mode == ANCHOR || inst->mode == TAG)
							{
								//evaluate our frame synchronization to see if we need to snap to the incoming value
								//and rebroadcast a SYNC message
								tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_EVAL);
							}

                        	break;
                        }
                        case RTLS_DEMO_MSG_INF_UPDATE : //fall through
                        case RTLS_DEMO_MSG_INF_SUG :    //fall through
                        case RTLS_DEMO_MSG_INF_REG :
                        {
							uint32 time_now = portGetTickCnt();
							uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);

							uint8 framelength;
							uint64 timeSinceFrameStart_us = 0;
							memcpy(&framelength, &messageData[TDMA_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[TDMA_TSFS], 6);

							//return to dicovery mode if no slots assigned to this UWB
							if(inst->mode == ANCHOR || inst->mode == TAG)
							{
								if(tdma_handler->uwbListTDMAInfo[0].slotsLength == 0)
								{
									inst->mode = DISCOVERY;
									tdma_handler->set_discovery_mode(tdma_handler, WAIT_INF_REG, time_now);
								}
							}

                        	if(inst->mode == DISCOVERY)
                        	{
                        		//NOTE: RX callback only accepts INF_UPDATE/INF_SUG/INF_REG for discovery modes WAIT_INF_REG and COLLECT_INF_REG.

                        		//1.) sync our frame start time to the local network
                        		//2.) collect and combine tdma info so we can construct a SUG packet and send it out

                        		if(tdma_handler->discovery_mode == WAIT_INF_INIT)
                        		{
                        			//if we receive network traffic while waiting for INF_INIT, transition to collecting INF messages
                        			tdma_handler->set_discovery_mode(tdma_handler, COLLECT_INF_REG, time_now);
                        		}

                        		if(tdma_handler->discovery_mode == WAIT_INF_REG) //treat INF_UPDATE and INF_SUG the same
								{
                        			//synchronize the frames
                        			tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_ADOPT);
                        			//initialize collection of tdma info, clear any previously stored info
                        			tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, CLEAR_ALL_COPY);
                        			//set discovery mode to COLLECT_INF_REG
                        			tdma_handler->set_discovery_mode(tdma_handler, COLLECT_INF_REG, time_now);
								}
                        		else if(tdma_handler->discovery_mode == COLLECT_INF_REG)
                        		{
                        			//synchronize the frames
									tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_COLLECT);
									//collecting tdma info, append to previously stored info
                        			tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, COPY);
                        		}
                        		else if(tdma_handler->discovery_mode == WAIT_SEND_SUG)
								{
                        			//process frame sync while waiting to send sug so we maintain syn with selected (sub)network
                        			//also give ourselves the opportunity to detect the need to transmit frame sync rebase messages
									tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_AVERAGE);
								}
                        	}
                        	else if(inst->mode == ANCHOR || inst->mode == TAG)
                        	{
                        		//if we are a TAG or ANCHOR
								//1.) sync our frame start time to the local network
                        		//2.) check for and adopt any tdma changes, sending an INF_UPDATE or INF_REG accordingly

                        		//synchronize the frames
								tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_AVERAGE);

								//collecting tdma info, append to previously stored info
								bool tdma_modified = tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, CLEAR_LISTED_COPY);

								if(tdma_modified)
								{
									//only repopulate the INF message if there was a modification to the TDMA configuration
									tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_UPDATE);
								}
                        	}

							//wait for next RX
							inst->testAppState = TA_RXE_WAIT ;

                        	break;
                        }
                        case RTLS_DEMO_MSG_INF_INIT :
						{
							//NOTE: discovery mode WAIT_INF_INIT checked in RX callback

							//process the INF packet
							uint32 time_now = portGetTickCnt();
							uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);

							uint8 framelength;
							uint64 timeSinceFrameStart_us = 0;
							memcpy(&framelength, &messageData[TDMA_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[TDMA_TSFS], 6);

							//synchronize the frames
							tdma_handler->frame_sync(tdma_handler, dw_event, framelength, timeSinceFrameStart_us, srcIndex, FS_ADOPT);
							//copy the TDMA network configuration directly
							tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, CLEAR_ALL_COPY);
							//copy new TDMA configuration into the INF message that this UWB will send out
							tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_UPDATE);
							//set discovery mode to EXIT
							tdma_handler->set_discovery_mode(tdma_handler, EXIT, time_now);

							inst->mode = ANCHOR;

							//stay in RX wait for next frame...
							inst->testAppState = TA_RXE_WAIT ;              // wait for next frame

							break;
						}//RTLS_DEMO_MSG_INF
                        case RTLS_DEMO_MSG_TAG_POLL:
                        {
                            if(dw_event->typePend == DWT_SIG_TX_PENDING)
                            {
                                inst->testAppState = TA_TX_WAIT_CONF;              // wait confirmation
                                inst->previousState = TA_TXRESPONSE_WAIT_SEND ;
                            }
                            else
                            {
                                //stay in RX wait for next frame...
                                inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                            }

                            break; 
                        }//RTLS_DEMO_MSG_TAG_POLL
                        case RTLS_DEMO_MSG_ANCH_RESP:
                        {
                        	if(dw_event->typePend == DWT_SIG_TX_PENDING)
							{
								inst->testAppState = TA_TX_WAIT_CONF;              // wait confirmation
								inst->previousState = TA_TXFINAL_WAIT_SEND ;
							}
							else
							{
								//stay in RX wait for next frame...
								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
							}

							break;
						} //RTLS_DEMO_MSG_ANCH_RESP
                        case RTLS_DEMO_MSG_TAG_FINAL :
                        {
                            int64 Rb, Da, Ra, Db ;
                            uint64 tagFinalTxTime  = 0;
                            uint64 tagFinalRxTime  = 0;
                            uint64 tagPollTxTime  = 0;
                            uint64 anchorRespRxTime  = 0;

                            double RaRbxDaDb = 0;
                            double RbyDb = 0;
                            double RayDa = 0;


                            // time of arrival of Final message
                            tagFinalRxTime = dw_event->timeStamp ; //Final's Rx time

                            inst->delayedReplyTime = 0 ;

                            // times measured at Tag extracted from the message buffer
                            // extract 40bit times
                            memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
                            memcpy(&anchorRespRxTime, &(messageData[RRXT]), 5);
                            memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

                            // poll response round trip delay time is calculated as
                            // (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
                            Ra = (int64)((anchorRespRxTime - tagPollTxTime) & MASK_40BIT);
                            Db = (int64)((inst->anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT);

                            // response final round trip delay time is calculated as
                            // (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
                            Rb = (int64)((tagFinalRxTime - inst->anchorRespTxTime) & MASK_40BIT);
							Da = (int64)((tagFinalTxTime - anchorRespRxTime) & MASK_40BIT);

                            RaRbxDaDb = (((double)Ra))*(((double)Rb)) - (((double)Da))*(((double)Db));
                            RbyDb = ((double)Rb + (double)Db);
                            RayDa = ((double)Ra + (double)Da);

                            //time-of-flight
                            inst->tof[inst->uwbToRangeWith] = (int64) ( RaRbxDaDb/(RbyDb + RayDa) );
                            inst->newRangeUWBIndex = inst->uwbToRangeWith;

                            if(inst->tof[inst->newRangeUWBIndex] > 0) //if ToF == 0 - then no new range to report
		                    {
		                        if(reportTOF(inst, inst->newRangeUWBIndex)==0)
		                        {
		                            inst->newRange = 1;
		                        }
		                    }

                            tdma_handler->uwbListTDMAInfo[inst->uwbToRangeWith].lastRange = portGetTickCnt();

                            inst->newRangeTagAddress = instance_get_uwbaddr(inst->uwbToRangeWith);
                            inst->newRangeAncAddress = instance_get_addr();

                            inst->testAppState = TA_TXREPORT_WAIT_SEND;

                            break; 
                        } //RTLS_DEMO_MSG_TAG_FINAL
                        case RTLS_DEMO_MSG_RNG_REPORT:
						{
							uint8 tag_index = instgetuwblistindex(inst, &messageData[REPORT_ADDR], inst->addrByteSize);
							uint8 anchor_index = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);

							//for now only process if we are the TAG that ranged with the reporting ANCHOR
							//TODO make it so all are processed
							if(tag_index == 0)
							{
								inst->tof[anchor_index] = 0;

								//copy previously calculated ToF
								memcpy(&inst->tof[anchor_index], &messageData[REPORT_TOF], 6);

								inst->newRangeAncAddress = instance_get_uwbaddr(anchor_index);
								inst->newRangeTagAddress = instance_get_uwbaddr(tag_index);

								inst->newRangeUWBIndex = anchor_index;
								if(inst->tof[inst->newRangeUWBIndex] > 0) //if ToF == 0 - then no new range to report
								{
									if(reportTOF(inst, inst->newRangeUWBIndex)==0)
									{
										inst->newRange = 1;
									}
								}

								tdma_handler->uwbListTDMAInfo[inst->uwbToRangeWith].lastRange = portGetTickCnt();

								uint8 debug_msg[100];
//								int n = sprintf((char *)&debug_msg, "POLL_COMPLETE,%llX,%llX", inst->newRangeTagAddress, inst->newRangeAncAddress);
								int n = sprintf((char *)&debug_msg, "POLL_COMPLETE,%04llX,%04llX", inst->newRangeTagAddress, inst->newRangeAncAddress);
								send_usbmessage(&debug_msg[0], n);
								usb_run();

								tdma_handler->firstPollComplete = TRUE;
								inst->testAppState = TA_TX_SELECT;
								inst->previousState = TA_INIT;
								inst->nextState = TA_INIT;
								inst->uwbToRangeWith = 255;
							}
							else
							{
								inst->testAppState = TA_RXE_WAIT ;
								inst->previousState = TA_INIT;
								inst->nextState = TA_INIT;
								inst->uwbToRangeWith = 255;
							}

                            dwt_setrxaftertxdelay(0);
							instancesetantennadelays(); //this will update the antenna delay if it has changed, TODO remove?

                            done = INST_DONE_WAIT_FOR_NEXT_EVENT;

                            inst->canPrintInfo = TRUE; //ranging exchange complete, allow print to USB and LCD

							break;
						} //RTLS_DEMO_MSG_RNG_REPORT
                        default:
                        {
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                            dwt_setrxaftertxdelay(0);
                            
                            break;    
                        }
                    } //end switch (fcode)

                
                    break ; 
                } //end of DWT_SIG_RX_OKAY
                case DWT_SIG_RX_TIMEOUT :
                {
                	if(tdma_handler->discovery_mode == WAIT_RNG_INIT || tdma_handler->discovery_mode == WAIT_INF_INIT)
					{
                		uint32 time_now = portGetTickCnt();
						tdma_handler->set_discovery_mode(tdma_handler, WAIT_INF_REG, time_now);
					}

                	//TODO remove if/else chain below
                    int n;
                    if(inst->previousState == TA_TXBLINK_WAIT_SEND)
					{
						uint8 debug_msg[100];
						n = sprintf((char *)&debug_msg, "TX_BLINK_TIMEOUT,%04llX,NULL", instance_get_addr());
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}
                    else if(inst->previousState == TA_TXRANGINGINIT_WAIT_SEND)
                    {
                    }
                    else if(inst->previousState == TA_TXINF_WAIT_SEND)
					{
					}
					else if(inst->previousState == TA_TXFINAL_WAIT_SEND)
					{
						uint8 debug_msg[100];
//						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%04llX,%04llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}
					else if(inst->previousState == TA_TXPOLL_WAIT_SEND)
					{
						uint8 debug_msg[100];
//						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%04llX,%04llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}

                    instance_getevent(17); //get and clear this event
                    inst_processtxrxtimeout(inst);
                    message = 0; //clear the message as we have processed the event
                    
                    break;
                }
                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
                case 0:
                default:
                {
                	//check if RX is on every so often. Turn it on if it isn't.
                	uint32 time_now = portGetTickCnt();
					uint32 timeSinceRxCheck = get_dt32(inst->rxCheckOnTime, time_now);
					if(timeSinceRxCheck >= RX_CHECK_ON_PERIOD_MS)
					{
						inst->rxCheckOnTime = time_now;

						//read SYS_STATE, getting second byte
						uint8 regval = dwt_read8bitoffsetreg(SYS_STATE_ID,1);
						//get the first 5 bytes
						regval &= 0x1F;
						if(regval == 0){//RX IDLE
							dwt_forcetrxoff();
							instancerxon(inst, 0, 0);
						}
					}

					//check if it's time to BLINK
					if(tdma_handler->check_blink(tdma_handler) == TRUE)
					{
						inst->testAppState = TA_TX_SELECT;
					}

                    if(done == INST_NOT_DONE_YET)
                    {
                        done = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    } 
                    
                    break;
                }
            } // end of switch on message 

            break;
        } // end case TA_RX_WAIT_DATA
        default:
        {
            break;
        }
    } // end switch on testAppState

    return done;
} // end testapprun()

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init_s()
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);

    inst->mode = DISCOVERY;
    inst->testAppState = TA_INIT;

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    dwt_setinterrupt(SYS_MASK_VAL, 1);

    //this is platform dependent - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the LEDs on EVBs

    dwt_setcallbacks(instance_txcallback, instance_rxgoodcallback, instance_rxtimeoutcallback, instance_rxerrorcallback);

#if (USING_64BIT_ADDR==0)
    inst->addrByteSize = ADDR_BYTE_SIZE_S;
#else
    inst->addrByteSize = ADDR_BYTE_SIZE_L;
#endif

    inst->uwbToRangeWith = 255;

    return 0 ;
}


extern uint8 dwnsSFDlen[];

// Pre-compute frame lengths, timeouts and delays needed in ranging process.
// /!\ This function assumes that there is no user payload in the frame.
void instance_init_timings(void)
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint32 pre_len;
    int sfd_len;

	static const int data_len_bytes[FRAME_TYPE_NB] = {
            BLINK_FRAME_LEN_BYTES, RNG_INIT_FRAME_LEN_BYTES, POLL_FRAME_LEN_BYTES,
            RESP_FRAME_LEN_BYTES, FINAL_FRAME_LEN_BYTES, REPORT_FRAME_LEN_BYTES, SYNC_FRAME_LEN_BYTES};


    // Margin used for timeouts computation.
//    const int margin_sy = 50;
    uint64 margin_us = 200;

    // All internal computations are done in tens of picoseconds before
    // conversion into microseconds in order to ensure that we keep the needed
    // precision while not having to use 64 bits variables.

    // Compute frame lengths.
    // First step is preamble plus SFD length.
    sfd_len = dwnsSFDlen[inst->configData.dataRate];
    switch (inst->configData.txPreambLength)
    {
    case DWT_PLEN_4096:
        pre_len = 4096;
        break;
    case DWT_PLEN_2048:
        pre_len = 2048;
        break;
    case DWT_PLEN_1536:
        pre_len = 1536;
        break;
    case DWT_PLEN_1024:
        pre_len = 1024;
        break;
    case DWT_PLEN_512:
        pre_len = 512;
        break;
    case DWT_PLEN_256:
        pre_len = 256;
        break;
    case DWT_PLEN_128:
        pre_len = 128;
        break;
    case DWT_PLEN_64:
    default:
        pre_len = 64;
        break;
    }
    pre_len += sfd_len;
    // Convert preamble length from symbols to time. Length of symbol is defined
    // in IEEE 802.15.4 standard.
    if (inst->configData.prf == DWT_PRF_16M)
        pre_len *= 99359;
    else
        pre_len *= 101763;

    inst->storedPreLen = pre_len; //store to be used later with inf messages and frame_sync
    inst->storedPreLen_us = CEIL_DIV(pre_len, 100000);

    // Second step is data length for all frame types.
    for (int i = 0; i < FRAME_TYPE_NB; i++)
    {
    	inst->frameLengths_us[i] = instance_getmessageduration_us(data_len_bytes[i]);
    }

    uint64 duration = 0;
    duration += inst->frameLengths_us[POLL] - inst->storedPreLen_us; 					//poll tx ts to poll tx cb
    duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;			  					//poll tx cb to resp tx cmd
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RESP] + RX_TO_CB_DLY_US; //resp tx cmd to resp rx cb
    duration += margin_us;
    inst->durationPollTimeout = (uint16)convertmicrosectodevicetimeu(duration);

    duration = 0;
    duration += inst->frameLengths_us[RESP] - inst->storedPreLen_us; 					 //resp tx ts to resp tx cb
    duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;			  					 //resp tx cb to final tx cmd
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[FINAL] + RX_TO_CB_DLY_US; //final tx cmd to final rx cb
    duration += margin_us;
	inst->durationRespTimeout = (uint16)convertmicrosectodevicetimeu(duration);

	duration = 0;
	duration += inst->frameLengths_us[FINAL] - inst->storedPreLen_us; 					 //final tx ts to final tx cb
	duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;			  					 //final tx cb to report tx cmd
	duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[REPORT] + RX_TO_CB_DLY_US;//report tx cmd to report rx cb
	duration += margin_us;
	inst->durationFinalTimeout = (uint16)convertmicrosectodevicetimeu(duration);

//    // Update delay between poll transmission and final transmission.
//    	uint64 final_delay_us = 0;
//    	//poll tx ts to poll tx cb
//    	final_delay_us += inst->frameLengths_us[POLL] - inst->storedPreLen_us;
//    	//poll tx cb to resp tx cmd
//    	final_delay_us += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;
//    	//resp tx cmd to final rx cb
//    	final_delay_us += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RESP] + RX_TO_CB_DLY_US;

    //TODO fix TO values below
    // Final frame wait timeout time.
//    inst->durationFwToFinal_sy = US_TO_SY_INT(inst->frameLengths_us[FINAL])
//                        + RX_START_UP_SY + margin_sy;
//    // Ranging init frame wait timeout time.
//    inst->durationFwToRngInit_sy = US_TO_SY_INT(inst->frameLengths_us[RNG_INIT])
//                         + RX_START_UP_SY + margin_sy;

    // Delay between blink transmission and ranging init reception.
//    inst->durationTxBlink2RxRngInit_sy =
//        US_TO_SY_INT((RNG_INIT_REPLY_DLY_US) - inst->frameLengths_us[BLINK])
//        - RX_START_UP_SY;
    // Delay between anchor's response transmission and final reception.
//    inst->durationTxAnchResp2RxFinal_sy = US_TO_SY_INT(TAG_TURN_AROUND_TIME_US) - RX_START_UP_SY; //TODO remove?


	// POLL response delay. //TODO remove
//	inst->resp_dly_us[RESP_DLY_POLL] = (ANC_TURN_AROUND_TIME_US + inst->frameLengths_us[POLL]) & 0xFFFF;
//
//	// ANCH_RESP response delay.
//	inst->resp_dly_us[RESP_DLY_ANCH_RESP] = (TAG_TURN_AROUND_TIME_US + inst->frameLengths_us[RESP]) & 0xFFFF;

//	// FINAL response delay.			//TODO different turnaround time?
//	inst->resp_dly_us[RESP_DLY_FINAL] = (ANCH_TURN_AROUND_TIME_US + inst->frameLengths_us[FINAL]) & 0xFFFF;

//	// Update delay between poll transmission and rx enable
//	inst->txToRxDelayPoll_sy = US_TO_SY_INT((uint64)inst->resp_dly_us[RESP_DLY_POLL] - inst->frameLengths_us[POLL]) - RX_START_UP_SY;

	duration = 0;
	duration += inst->frameLengths_us[POLL] - inst->storedPreLen_us;						//poll tx ts to poll tx cb
	duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US; 									//poll tx cb to resp tx cmd
	duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RESP] + RX_TO_CB_DLY_US;		//resp tx cmd to resp rx cb
	duration += RX_CB_TO_TX_CMD_DLY_US + MIN_DELAYED_TX_DLY_US + inst->storedPreLen_us;		//resp rx cb to final tx timestamp
	inst->finalReplyDelay = convertmicrosectodevicetimeu(duration);

    // Delay between blink reception and ranging init message transmission.
    inst->rnginitReplyDelay = convertmicrosectodevicetimeu(RNG_INIT_REPLY_DLY_US);


    margin_us = 1000;
    inst->durationBlinkTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[BLINK] + margin_us, 1000);			//tx cmd to tx cb
    inst->durationRngInitTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RNG_INIT] + margin_us, 1000);		//tx cmd to tx cb
    inst->durationPollTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[POLL] + margin_us, 1000);				//tx cmd to tx cb
    inst->durationRespTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RESP] + margin_us, 1000);				//tx cmd to tx cb
    inst->durationFinalTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[FINAL] + margin_us, 1000);			//tx cmd to tx cb
    inst->durationReportTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[REPORT] + margin_us, 1000);			//tx cmd to tx cb
    inst->durationSyncTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[SYNC] + margin_us, 1000);				//tx cmd to tx cb



    inst->smartPowerEn = 0;
}

uint32 instance_getmessageduration_us(int data_length_bytes)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	// Compute the number of symbols for the given length.
	uint32 framelength_us = data_length_bytes * 8
				 + CEIL_DIV(data_length_bytes * 8, 330) * 48;
	// Convert from symbols to time and add PHY header length.
	if(inst->configData.dataRate == DWT_BR_110K)
	{
		framelength_us *= 820513;
		framelength_us += 17230800;
	}
	else if (inst->configData.dataRate == DWT_BR_850K)
	{
		framelength_us *= 102564;
		framelength_us += 2153900;
	}
	else
	{
		framelength_us *= 12821;
		framelength_us += 2153900;
	}
	// Last step: add preamble length and convert to microseconds.
	framelength_us += inst->storedPreLen;
	framelength_us = CEIL_DIV(framelength_us, 100000);

	return framelength_us;
}

uint64 instance_get_addr(void) //get own address
{
    return instance_get_uwbaddr(0);
}

uint64 instance_get_uwbaddr(uint8 uwb_index) //get uwb address by index
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint64 x = 0;
    x |= (uint64) inst->uwbList[uwb_index][0];
    x |= (uint64) inst->uwbList[uwb_index][1] << 8;
#if (USING_64BIT_ADDR == 1)
    x |= (uint64) inst->uwbList[uwb_index][2] << 16;
    x |= (uint64) inst->uwbList[uwb_index][3] << 24;
    x |= (uint64) inst->uwbList[uwb_index][4] << 32;
    x |= (uint64) inst->uwbList[uwb_index][5] << 40;
    x |= (uint64) inst->uwbList[uwb_index][6] << 48;
    x |= (uint64) inst->uwbList[uwb_index][7] << 56;
#endif

    return (x);
}

void instance_readaccumulatordata(void)
{
#if DECA_SUPPORT_SOUNDING==1
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint16 len = 992 ; //default (16M prf)

    if (inst->configData.prf == DWT_PRF_64M)  // Figure out length to read
        len = 1016 ;

    inst->buff.accumLength = len ;                                       // remember Length, then read the accumulator data

    len = len*4+1 ;   // extra 1 as first byte is dummy due to internal memory access delay

    dwt_readaccdata((uint8*)&(inst->buff.accumData->dummy), len, 0);
#endif  // support_sounding
}

//get the time difference between two between two 32-bit unsigned timestamps
//t1 is the first timestamp
//t2 is the second timetamp that occured after t1 
uint32 get_dt32(uint32 t1, uint32 t2)
{
    if(t2 >= t1)
    {
        return t2 - t1;
    }
    else
    {
        //handle timestamp roleover
        return 4294967295 - t1 + t2;
    }
}

//add a duration to a 32 bit timestamp. This function handles number wrapping
uint32 timestamp_add32(uint32 timestamp, uint32 duration)
{
	uint32 to_wrap = 4294967295 - timestamp;
	if(duration > to_wrap)
	{
		return to_wrap + duration;
	}
	else
	{
		return timestamp + duration;
	}
}

//subtract a duration from a 32 bit timestamp. This function handles number wrapping
uint32 timestamp_subtract32(uint32 timestamp, uint32 duration)
{
	if(duration > timestamp)
	{
		return 4294967295 - (duration - timestamp);
	}
	else
	{
		return timestamp - duration;
	}
}

//get the time difference between two between two 64-bit unsigned timestamps
//t1 is the first timestamp
//t2 is the second timetamp that occured after t1
uint64 get_dt64(uint64 t1, uint64 t2)
{
    if(t2 >= t1)
    {
        return t2 - t1;
    }
    else
    {
        //handle timestamp rollover
        return 4294967295999 - t1 + t2;
    }
}

//add a duration to a 64 bit timestamp. This function handles number wrapping
uint64 timestamp_add64(uint64 timestamp, uint64 duration)
{
	uint64 to_wrap = (uint64)4294967295999 - timestamp;
	if(duration > to_wrap)
	{
		return to_wrap + duration;
	}
	else
	{
		return timestamp + duration;
	}
}

//subtract a duration from a 64 bit timestamp. This function handles number wrapping
uint64 timestamp_subtract64(uint64 timestamp, uint64 duration)
{
	if(duration > timestamp)
	{
		return (uint64)4294967295999 - (duration - timestamp);
	}
	else
	{
		return timestamp - duration;
	}
}

//TODO implement a hashing function to reduce chance of collisions
//https://stackoverflow.com/questions/31710074/how-to-generate-smaller-unique-number-from-larger-11-bytes-unique-number-gene
//https://en.m.wikipedia.org/wiki/Pearson_hashing
uint16 address64to16(uint8 *address)
{
	return address[0] + (address[1] << 8);
}


#endif



/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
