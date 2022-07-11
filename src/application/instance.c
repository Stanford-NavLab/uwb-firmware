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

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);
 
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
        case TA_TXPOLL_WAIT_SEND : return "TA_TXPOLL_WAIT_SEND";       
        case TA_TXFINAL_WAIT_SEND : return "TA_TXFINAL_WAIT_SEND";  
        case TA_TXRESPONSE_WAIT_SEND : return "TA_TXRESPONSE_WAIT_SEND";
        case TA_TX_WAIT_CONF : return "TA_TX_WAIT_CONF";
        case TA_RXE_WAIT : return "TA_RXE_WAIT";
        case TA_RX_WAIT_DATA : return "TA_RX_WAIT_DATA";
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
    else if(fcode == (int)RTLS_DEMO_MSG_SYNC)
    {
    	return "RTLS_DEMO_MSG_SYNC";
    }
    else
    {
        return "NONE";
    }
}




// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag or Anchor use the same state machine)
//
//
int testapprun(instance_data_t *inst, struct TDMAHandler *tdma_handler, int message)
{
//	send_statetousb(inst, tdma_handler);


    int done = INST_NOT_DONE_YET;

    if(tdma_handler->slot_transition(tdma_handler))
    {
    	done = INST_DONE_WAIT_FOR_NEXT_EVENT;
		message = 0;
    }

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
                    dwt_forcetrxoff();

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_RSVD_EN);
					inst->frameFilteringEnabled = 1 ;
					dwt_setpanid(inst->panID);
					dwt_seteui(inst->eui64);


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


					inst->uwbShortAdd = inst->eui64[0] + (inst->eui64[1] << 8);//NOTE a hashing algorithm could be used instead

#if (USING_64BIT_ADDR==0)
					dwt_setaddress16(inst->uwbShortAdd);
					memcpy(&inst->uwbList[0][0], &inst->uwbShortAdd, inst->addrByteSize);
#else
					memcpy(&inst->uwbList[0][0], &inst->eui64, inst->addrByteSize);
#endif
					inst->uwbListLen = 1;
					tdma_handler->uwbListTDMAInfo[0].connectionType = UWB_LIST_SELF;


                    instanceconfigframeheader(inst);
                    instanceconfigmessages(inst);

					//change to next state - wait to receive a message
					tdma_handler->discoveryStartTime = portGetTickCnt();
					tdma_handler->last_blink_time = portGetTickCnt();
					inst->testAppState = TA_RXE_WAIT ;

					dwt_setrxtimeout(0);
					inst->wait4ack = 0;
					inst->canPrintUSB = TRUE;
					inst->canPrintLCD = TRUE;
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
        case TA_TXBLINK_WAIT_SEND :
		{
			int psduLength = BLINK_FRAME_LEN_BYTES;

            //blink frames with IEEE EUI-64 tag ID
			inst->blinkmsg.seqNum = inst->frameSN++;
			inst->wait4ack = 0;

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
				inst->testAppState = TA_TX_WAIT_CONF ; // wait confirmation
				inst->previousState = TA_TXBLINK_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

				inst->timeofTx = portGetTickCnt();

				inst->txDoneTimeoutDuration = inst->durationBlinkTxDoneTimeout_ms;
				tdma_handler->last_blink_time = portGetTickCnt();
				tdma_handler->blinkPeriodRand = (uint32)rand()%BLINK_PERIOD_RAND_MS;
			}

			break ;
		}// end case TA_TXBLINK_WAIT_SEND
        case TA_TXRANGINGINIT_WAIT_SEND :
        {
        	int psduLength = RNG_INIT_FRAME_LEN_BYTES;

            inst->rng_initmsg.seqNum = inst->frameSN++;

            inst->wait4ack = 0;

            //add a small random number to this to reduce chance of collisions
			uint8 sys_time_arr[5] = {0, 0, 0, 0, 0};
			dwt_readsystime(sys_time_arr);
			uint64 dwt_time_now = 0;
			dwt_time_now = (uint64)sys_time_arr[0] + ((uint64)sys_time_arr[1] << 8) + ((uint64)sys_time_arr[2] << 16) + ((uint64)sys_time_arr[3] << 24) + ((uint64)sys_time_arr[4] << 32);
			inst->delayedReplyTime = (dwt_time_now + inst->rnginitReplyDelay + convertmicrosectodevicetimeu(rand()%RANGE_INIT_RAND_US)) >> 8 ;  // time we should send the blink response


            dwt_writetxdata(psduLength, (uint8 *)  &inst->rng_initmsg, 0) ; // write the frame data
			if(instancesendpacket(psduLength, DWT_START_TX_DELAYED | inst->wait4ack, inst->delayedReplyTime))
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
			//update slot assignment
			if(tdma_handler->reassigSlots == TRUE)
			{
				tdma_handler->free_slots(&tdma_handler->uwbListTDMAInfo[0]);
				tdma_handler->find_assign_slot(tdma_handler);
				tdma_handler->reassigSlots = FALSE;
				tdma_handler->tdmaIsDirty = TRUE;
			}

			if(tdma_handler->tdmaIsDirty == TRUE)
			{
				tdma_handler->tdmaIsDirty = FALSE;
				tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_UPDATE);
			}

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
				inst->testAppState = TA_TX_WAIT_CONF ;
				inst->previousState = TA_TXINF_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

				inst->timeofTx = portGetTickCnt();

				uint64 margin_us = 1000;
				uint64 framelength_us = instance_getmessageduration_us(psduLength);
				inst->txDoneTimeoutDuration = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + framelength_us + margin_us, 1000);			//tx cmd to tx cb

			}

			break;
		}
        case TA_TXPOLL_WAIT_SEND :
        {
        	int psduLength = POLL_FRAME_LEN_BYTES;

            inst->msg.seqNum = inst->frameSN++;
			inst->msg.messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL; //message function code (specifies if message is a poll, response or other...)
			memcpy(&inst->msg.destAddr[0], &inst->uwbList[inst->uwbToRangeWith], inst->addrByteSize);

			tdma_handler->nthOldest++;

            inst->wait4ack = 0;

			dwt_writetxdata(psduLength, (uint8 *)  &inst->msg, 0) ; // write the frame data

            if(instancesendpacket(psduLength, DWT_START_TX_IMMEDIATE | inst->wait4ack, 0))
			{
            	//failed
            	inst->tx_poll = FALSE;

				inst->previousState = TA_INIT;
				inst->nextState = TA_INIT;
				inst->testAppState = TA_RXE_WAIT;

				inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			}
			else
			{
				//succeeded

				inst->tx_poll = TRUE;
				inst->testAppState = TA_TX_WAIT_CONF ;
				inst->previousState = TA_TXPOLL_WAIT_SEND ;
				done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)
				inst->canPrintUSB = FALSE;
				inst->canPrintLCD = FALSE;

				inst->timeofTx = portGetTickCnt();
				inst->timeofTxPoll = portGetTickCnt();
				inst->txDoneTimeoutDuration = inst->durationPollTxDoneTimeout_ms;
			}

            break;
        }
        case TA_TXREPORT_WAIT_SEND :
		{
			int psduLength = REPORT_FRAME_LEN_BYTES;

			// Write calculated TOF into response message
			memcpy(&inst->report_msg.messageData[REPORT_TOF], &inst->tof[inst->uwbToRangeWith], 6);
			memcpy(&inst->report_msg.messageData[REPORT_RSL], &inst->rxPWR, sizeof(double));
			memcpy(&inst->report_msg.messageData[REPORT_ADDR], &inst->uwbList[inst->uwbToRangeWith], inst->addrByteSize);
			inst->report_msg.seqNum = inst->frameSN++;

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

				inst->txDoneTimeoutDuration = inst->durationReportTxDoneTimeout_ms;
				inst->canPrintLCD = FALSE;
			}

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
					inst->canPrintUSB = TRUE;
					inst->canPrintLCD = TRUE;

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
                        //set destination address
                        memcpy(&inst->rng_initmsg.destAddr[0], &(dw_event->msgu.rxblinkmsg.tagID[0]), BLINK_FRAME_SOURCE_ADDRESS); //remember who to send the reply to


                        inst->testAppState = TA_TXRANGINGINIT_WAIT_SEND;
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

							uint32 time_now = portGetTickCnt();
                            tdma_handler->build_new_network(tdma_handler);
                            tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_INIT);
							tdma_handler->set_discovery_mode(tdma_handler, EXIT, time_now);

							inst->testAppState = TA_TX_SELECT;
							inst->mode = TAG;

                            break;
                        } //RTLS_DEMO_MSG_RNG_INIT
                        case RTLS_DEMO_MSG_SYNC :
                        {
                        	uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);
                        	uint8 largestFramelength;
							uint64 timeSinceFrameStart_us = 0;

							memcpy(&largestFramelength, &messageData[SYNC_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[SYNC_TSFS], 6);

							if(inst->mode == ANCHOR || inst->mode == TAG)
							{
								//evaluate our frame synchronization to see if we need to snap to the incoming value
								//and rebroadcast a SYNC message
								tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_EVAL);
							}

                        	break;
                        }
                        case RTLS_DEMO_MSG_INF_UPDATE : //fall through
                        case RTLS_DEMO_MSG_INF_SUG :    //fall through
                        case RTLS_DEMO_MSG_INF_REG :
                        {
                        	uint32 time_now = portGetTickCnt();
							uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);

							uint8 largestFramelength;
							uint64 timeSinceFrameStart_us = 0;
							memcpy(&largestFramelength, &messageData[TDMA_LARGEST_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[TDMA_TSFS], 6);

							//return to discovery mode if no slots assigned to this UWB
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
                        			tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_ADOPT);
                        			//initialize collection of tdma info, clear any previously stored info
                        			tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, CLEAR_ALL_COPY);
                        			//set discovery mode to COLLECT_INF_REG
                        			tdma_handler->set_discovery_mode(tdma_handler, COLLECT_INF_REG, time_now);
								}
                        		else if(tdma_handler->discovery_mode == COLLECT_INF_REG)
                        		{
                        			//synchronize the frames
									tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_COLLECT);
									//collecting tdma info, append to previously stored info
                        			tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, COPY);
                        		}
                        		else if(tdma_handler->discovery_mode == WAIT_SEND_SUG)
								{
                        			//process frame sync while waiting to send sug so we maintain syn with selected (sub)network
                        			//also give ourselves the opportunity to detect the need to transmit frame sync rebase messages
									tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_AVERAGE);
								}
                        	}
                        	else if(inst->mode == ANCHOR || inst->mode == TAG)
                        	{
                        		//if we are a TAG or ANCHOR
								//1.) sync our frame start time to the local network
                        		//2.) check for and adopt any tdma changes, sending an INF_UPDATE or INF_REG accordingly

                        		//synchronize the frames
								tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_AVERAGE);

								//collecting tdma info, append to previously stored info
								bool tdma_modified = tdma_handler->process_inf_msg(tdma_handler, messageData, srcIndex, CLEAR_LISTED_COPY);

								if(tdma_modified)
								{
									tdma_handler->tdmaIsDirty = TRUE;
									//TODO
									//only repopulate the INF message if there was a modification to the TDMA configuration
//									tdma_handler->populate_inf_msg(tdma_handler, RTLS_DEMO_MSG_INF_UPDATE);
								}
                        	}


                        	//INF is last message in slot, we can print after processing
                        	inst->canPrintUSB = TRUE;
							inst->canPrintLCD = TRUE;

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

							uint8 largestFramelength;
							uint64 timeSinceFrameStart_us = 0;
							memcpy(&largestFramelength, &messageData[TDMA_LARGEST_FRAMELENGTH], sizeof(uint8));
							memcpy(&timeSinceFrameStart_us, &messageData[TDMA_TSFS], 6);

							//synchronize the frames
							tdma_handler->frame_sync(tdma_handler, dw_event, largestFramelength, timeSinceFrameStart_us, srcIndex, FS_ADOPT);
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
                            tagFinalRxTime = inst->dwt_final_rx;

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

							if(reportTOF(inst, inst->newRangeUWBIndex, inst->rxPWR)==0)
							{
								inst->newRange = 1;
							}

                            tdma_handler->uwbListTDMAInfo[inst->uwbToRangeWith].lastRange = portGetTickCnt();

                            inst->newRangeTagAddress = instance_get_uwbaddr(inst->uwbToRangeWith);
                            inst->newRangeAncAddress = instance_get_addr();

                            inst->testAppState = TA_TXREPORT_WAIT_SEND;
                            inst->delayedReplyTime = 0 ;

                            break;
                        } //RTLS_DEMO_MSG_TAG_FINAL
                        case RTLS_DEMO_MSG_RNG_REPORT :
						{
							uint8 tag_index = instgetuwblistindex(inst, &messageData[REPORT_ADDR], inst->addrByteSize);
							uint8 anchor_index = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);

							//for now only process if we are the TAG that ranged with the reporting ANCHOR
							inst->tof[anchor_index] = 0;

							//copy previously calculated ToF
							memcpy(&inst->tof[anchor_index], &messageData[REPORT_TOF], 6);
							memcpy(&inst->rxPWR, &messageData[REPORT_RSL], sizeof(double));


							inst->newRangeAncAddress = instance_get_uwbaddr(anchor_index);
							inst->newRangeTagAddress = instance_get_uwbaddr(tag_index);

							inst->newRangeUWBIndex = anchor_index;
							if(inst->tof[inst->newRangeUWBIndex] > 0) //if ToF == 0 - then no new range to report
							{
								if(reportTOF(inst, inst->newRangeUWBIndex, inst->rxPWR)==0)
								{
									inst->newRange = 1;
								}
							}

							if(tag_index == 0)
							{
								tdma_handler->nthOldest = 1;
								tdma_handler->uwbListTDMAInfo[inst->uwbToRangeWith].lastRange = portGetTickCnt();
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

							inst->canPrintUSB = TRUE;
							inst->canPrintLCD = FALSE;

							break;
						} //RTLS_DEMO_MSG_RNG_REPORT
                        default:
                        {
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame

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

                    instance_getevent(17); //get and clear this event
                    inst_processtxrxtimeout(inst);
                    message = 0; //clear the message as we have processed the event

                    break;
                }
                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
                case 0:
                default:
                {
                	if(inst->mode == TAG){
                		//get the message FCODE
						uint8 fcode;
						memcpy(&fcode, &inst->msg.messageData[FCODE], sizeof(uint8));

						if(fcode == RTLS_DEMO_MSG_TAG_POLL)
						{
							uint32 dt = get_dt32(inst->timeofTxPoll, portGetTickCnt());
							if(dt > inst->durationPollTimeout_ms)
							{
								inst_processtxrxtimeout(inst);
							}
						}
						else if(fcode == RTLS_DEMO_MSG_TAG_FINAL)
						{
							uint32 dt = get_dt32(inst->timeofTxFinal, portGetTickCnt());
							if(dt > inst->durationFinalTimeout_ms)
							{
								inst_processtxrxtimeout(inst);
							}
						}
                	}

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

    dwt_setcallbacks(instance_txcallback, instance_rxgoodcallback, instance_rxtimeoutcallback, instance_rxerrorcallback, instance_irqstuckcallback);

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
            RESP_FRAME_LEN_BYTES, FINAL_FRAME_LEN_BYTES, REPORT_FRAME_LEN_BYTES, SYNC_FRAME_LEN_BYTES, INF_FRAME_LEN_BYTES_MAX};


    // Margin used for timeouts computation.
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

    //delayed tx durations
    uint8 reply_margin_us = 25;
    uint64 duration = 0;
	duration += inst->frameLengths_us[POLL] - inst->storedPreLen_us + RX_TO_CB_DLY_US;		//poll rx timestamp to poll rx cb
	duration += RX_CB_TO_TX_CMD_DLY_US + MIN_DELAYED_TX_DLY_US + inst->storedPreLen_us;		//poll rx cb to resp tx timestamp
	uint64 respDelay_us = duration + reply_margin_us;
	uint64 respDelay = convertmicrosectodevicetimeu(respDelay_us);

	duration = 0;
	duration += inst->frameLengths_us[RESP] - inst->storedPreLen_us + RX_TO_CB_DLY_US;		//resp rx timestamp to resp rx cb
	duration += RX_CB_TO_TX_CMD_DLY_US + MIN_DELAYED_TX_DLY_US + inst->storedPreLen_us;		//resp rx cb to final tx timestamp
	uint64 finalDelay_us = duration + reply_margin_us;
	uint64 finalDelay = convertmicrosectodevicetimeu(finalDelay_us);

	//make reply times the same to minimize clock drift error. See Application Note APS011 for more information
	inst->respReplyDelay = inst->finalReplyDelay = MAX(respDelay, finalDelay);
	uint64 replyDelay_us = MAX(respDelay_us, finalDelay_us);


	//POLL TX TS TO FINAL TX TS
	duration = 0;
	duration += respDelay_us + finalDelay_us;												//poll tx ts to final tx ts
	uint64 pollTxToFinalTx = duration;

    // Delay between blink reception and ranging init message transmission.
	inst->rnginitReplyDelay = convertmicrosectodevicetimeu(MIN_DELAYED_TX_DLY_US + inst->storedPreLen_us); //rng_init tx cmd to rng_init tx ts

	margin_us = 1000;
    //rx timeout durations (_nus units are 1.0256us)
    duration = 0;
    duration +=	TX_CMD_TO_TX_CB_DLY_US + replyDelay_us;						//poll tx cmd to resp tx ts
    duration +=	inst->frameLengths_us[RESP] - inst->storedPreLen_us;		//resp tx ts to resp tx cb
    duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;					//resp tx cb to final tx cmd
    duration += margin_us;
    inst->durationPollTimeout_nus = (uint16)(duration/1.0256) + 1;
	inst->durationPollTimeout_ms = (uint16)CEIL_DIV(duration, 1000);

	duration = 0;
	duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[FINAL]; 	                                           //final tx cmd to final tx cb
	duration += RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;			  	                                           //final tx cb to place final
	duration += (uint64)MEASURED_SLOT_DURATIONS_US/2;                                   						   //place final to report tx cmd
	duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[REPORT] + RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US; //report tx cmd to place report
	duration += margin_us;
	inst->durationFinalTimeout_ms = (uint16)CEIL_DIV(duration, 1000);

	margin_us = 1000;
    //tx conf timeout durations
    inst->durationBlinkTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[BLINK] + margin_us, 1000);			//tx cmd to tx cb
    inst->durationRngInitTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[RNG_INIT] + margin_us, 1000);		//tx cmd to tx cb
    inst->durationPollTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[POLL] + margin_us, 1000);				//tx cmd to tx cb
    inst->durationReportTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[REPORT] + margin_us, 1000);			//tx cmd to tx cb
    inst->durationSyncTxDoneTimeout_ms = CEIL_DIV(TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[SYNC] + margin_us, 1000);				//tx cmd to tx cb

    uint32 fl = 0;
    if(inst->frameLengths_us[RESP] < inst->frameLengths_us[FINAL])
    {
    	fl = inst->frameLengths_us[RESP] - inst->storedPreLen_us;
    }
    else
    {
    	fl = inst->frameLengths_us[FINAL] - inst->storedPreLen_us;
    }
    duration = 0;
    duration += replyDelay_us - fl;
    duration -= RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;
    inst->durationRespTxDoneTimeout_ms = CEIL_DIV(duration + inst->frameLengths_us[RESP] - inst->storedPreLen_us + margin_us, 1000);	//tx cmd to tx cb
    inst->durationFinalTxDoneTimeout_ms = CEIL_DIV(duration + inst->frameLengths_us[FINAL] - inst->storedPreLen_us + margin_us, 1000);	//tx cmd to tx cb

    //figure maximum duration of a TDMA slot in microseconds
    duration = 0;
    duration += SLOT_START_BUFFER_US;	//frame start buffer
    duration += SLOT_BUFFER_EXP_TO_POLL_CMD_US; //buffer expiration to cmd poll
    duration += TX_CMD_TO_TX_CB_DLY_US + pollTxToFinalTx + inst->frameLengths_us[FINAL] + RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US;//poll cmd to place final
    //duration += B //place final to cmd report
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[REPORT] + RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US; //cmd report to place report
    //duration += C //place report to cmd inf
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[INF_MAX] + RX_TO_CB_DLY_US + RX_CB_TO_TX_CMD_DLY_US; //cmd INF to place INF
    //duration += D place inf to process inf

    //MEASURED_SLOT_DURATIONS_US is experimentally found value found for B+C+D described above
    duration += (uint64)MEASURED_SLOT_DURATIONS_US;

    //add some time to account for possibly timing out on first poll
    duration += SLOT_BUFFER_EXP_TO_POLL_CMD_US; //assume this is the same amount of time go get from timeout to poll command
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[POLL] - inst->storedPreLen_us + (uint64)(inst->durationPollTimeout_nus*1.0256) + 200; //add small margin of 200


    //if LCD is on, add time to allow for Sleep calls in the LCD display logic
    bool enableLCD = FALSE;
    if(port_is_switch_on(TA_SW1_4) == S1_SWITCH_ON)
	{
    	enableLCD = TRUE;
		duration += LCD_ENABLE_BUFFER_US;
	}

    duration += SLOT_END_BUFFER_US;

    inst->durationSlotMax_us = duration;

    duration = 0;
    //from set discovery to cmd tx blink, to rx blink cb to cmd resp to rx resp ts
    duration += TX_CMD_TO_TX_CB_DLY_US + inst->frameLengths_us[BLINK] + RX_TO_CB_DLY_US + BLINK_RX_CB_TO_RESP_TX_CMD_DLY_US + (uint64)(convertdevicetimetosec(inst->rnginitReplyDelay)*1000000.0)  + RANGE_INIT_RAND_US + margin_us;
    if(enableLCD == TRUE)
    {
    	duration += LCD_ENABLE_BUFFER_US*2;
    }
    inst->durationWaitRangeInit_ms = CEIL_DIV(duration, 1000);


    // Smart Power is automatically applied by DW chip for frame of which length
	// is < 1 ms. Let the application know if it will be used depending on the
	// length of the longest frame.
	if (inst->frameLengths_us[FINAL] <= 1000)
	{
		inst->smartPowerEn = 1;
	}
	else
	{
		inst->smartPowerEn = 0;
	}
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
	// Last step: add preamble and SFD length and convert to microseconds.
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

//NOTE could implement a hashing function to reduce chance of collisions among UWB addresses in network
//https://stackoverflow.com/questions/31710074/how-to-generate-smaller-unique-number-from-larger-11-bytes-unique-number-gene
//https://en.m.wikipedia.org/wiki/Pearson_hashing
uint16 address64to16(uint8 *address)
{
	return address[0] + (address[1] << 8);
}


#endif
