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

#include "llist.h"



extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

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
    inst->msg[inst->uwbToRangeWith].panID[0] = (inst->panID) & 0xff;
    inst->msg[inst->uwbToRangeWith].panID[1] = inst->panID >> 8;

    //set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
    inst->msg[inst->uwbToRangeWith].frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;
#if (USING_64BIT_ADDR==1)
    //source/dest addressing modes and frame version
    inst->msg[inst->uwbToRangeWith].frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0xC0 /*src extended address (64bits)*/;
#else
    inst->msg[inst->uwbToRangeWith].frameCtrl[1] = 0x8 /*dest short address (16bits)*/ | 0x80 /*src short address (16bits)*/;
#endif
}


// -------------------------------------------------------------------------------------------------------------------
//
// Turn on the receiver with/without delay
//
void instancerxon(instance_data_t *inst, int delayed, uint64 delayedReceiveTime)
{
    if (delayed)
    {
        uint32 dtime;
        dtime =  (uint32) (delayedReceiveTime>>8);
        dwt_setdelayedtrxtime(dtime) ;
    }

//    inst->lateRX -= dwt_rxenable(delayed) ;  //- as when fails -1 is returned             // turn receiver on, immediate/delayed

    uint32 time_now = portGetTickCnt();
    int dwt_rx_enable_return = dwt_rxenable(delayed);
	 uint8 debug_msg[100];
	 int n = sprintf((char*)&debug_msg[0], "RX_ENABLE time_now: %lu, rx_enable: %i", time_now, dwt_rx_enable_return);
	 send_usbmessage(&debug_msg[0], n);
	 usb_run();
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
        case TA_TXBLINK_WAIT_SEND : return "TA_TXBLINK_WAIT_SEND";
        case TA_TXRANGINGINIT_WAIT_SEND : return "TA_TXRANGINGINIT_WAIT_SEND";
        case TA_TX_SELECT : return "TA_TX_SELECT";
        default: return "NONE";
    }
}

//debug helper function to print the mode
const char* get_instanceModes_string(enum instanceModes mode)
{
    switch (mode)
    {
        case LISTENER : return "LISTENER";
        case TAG : return "TAG";       
        case ANCHOR : return "ANCHOR";       
        case TAG_TDOA : return "TAG_TDOA";  
        case NUM_MODES : return "NUM_MODES";
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
    else
    {
        return "NONE";
    }
}

void send_statetousb(instance_data_t *inst)
{
    
    int usbdebugdata_size = sprintf((char*)&usbdebugdata[0], "%s , %s , %s , %s", get_inst_states_string(inst->testAppState), get_inst_states_string(inst->previousState), get_inst_states_string(inst->nextState), get_instanceModes_string(inst->mode));
     
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
    int usbrxdebugdata_size = sprintf((char*)&usbrxdebugdata[0], "%s", data);
     
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
    int usbtxdebugdata_size = sprintf((char*)&usbtxdebugdata[0], "TX message: %s", data);
     
    send_usbmessage(&usbtxdebugdata[0], usbtxdebugdata_size);
    usb_run();
}

// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine (all the instance modes Tag, Anchor or Listener use the same statemachine....)
//
// -------------------------------------------------------------------------------------------------------------------
//
int testapprun(instance_data_t *inst, int message)
{
    int done = INST_NOT_DONE_YET;
    
//    uint8 debug_msg[100];
//	int n = sprintf((char *)&debug_msg, "TAR");
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();


    //NOTE: temporary debug code
    if(inst->testAppState != inst->lastState){
    	inst->currentStateStartTime = portGetTickCnt();
    }
    inst->lastState = inst->testAppState;

    send_statetousb(inst);



    switch (inst->testAppState)
    {
        case TA_INIT :
        {
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }

            switch (inst->mode)
            {
                case TAG:
                {
                    int mode = 0;

                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN); //allow data, ACK frames;
                    inst->frameFilteringEnabled = 1 ;
                    dwt_setpanid(inst->panID);
                    dwt_seteui(inst->eui64);

                    inst->uwbShortAdd = inst->eui64[0] + (inst->eui64[1] << 8);
                    
                    
#if (USING_64BIT_ADDR==0)
                    dwt_setaddress16(inst->uwbShortAdd);
#endif

                    //set source address into the message structure
                    for(int i=0; i<UWB_LIST_SIZE; i++)
                    {
                        memcpy(&inst->msg[i].sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
                    }

                    inst->testAppState = TA_TXBLINK_WAIT_SEND;
                    inst->tx_scheduler.last_blink_time = portGetTickCnt();
                    memcpy(&inst->blinkmsg.tagID[0], &inst->eui64[0], ADDR_BYTE_SIZE_L);

                    mode = (DWT_PRESRV_SLEEP|DWT_CONFIG|DWT_TANDV);

                    if(inst->configData.txPreambLength == DWT_PLEN_64)  //if using 64 length preamble then use the corresponding OPSet
                    {
                        mode |= DWT_LOADOPSET;
                    }
#if (DEEP_SLEEP == 1)
                    if (inst->sleepingEabled)
                        dwt_configuresleep(mode, DWT_WAKE_WK|DWT_WAKE_CS|DWT_SLP_EN); //configure the on wake parameters (upload the IC config settings)
#endif

                }
                break;
                case ANCHOR:
                {
                    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN | DWT_FF_RSVD_EN);
                    inst->frameFilteringEnabled = 1 ;
                    dwt_seteui(inst->eui64);
                    dwt_setpanid(inst->panID);

                    inst->uwbShortAdd = inst->eui64[0] + (inst->eui64[1] << 8);

#if (USING_64BIT_ADDR==0)
                    dwt_setaddress16(inst->uwbShortAdd);
#endif

                    //set source address into the message structure
                    for(int i=0; i<UWB_LIST_SIZE; i++)
                    {
                        memcpy(&inst->msg[i].sourceAddr[0], &inst->eui64[0], inst->addrByteSize);
                    }
                    //set source address into the message structure
                    memcpy(&inst->rng_initmsg.sourceAddr[0], &inst->eui64[0], inst->addrByteSize);

                    // First time anchor listens we don't do a delayed RX
                    dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

                    dwt_setrxtimeout(0);
                    inst->canPrintInfo = 1;
                }
                break;
                case LISTENER:
                {
                    dwt_enableframefilter(DWT_FF_NOTYPE_EN); //disable frame filtering
                    inst->frameFilteringEnabled = 0 ;
                    // First time anchor listens we don't do a delayed RX
                    dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

                    dwt_setrxtimeout(0);
                }
                break ; // end case TA_INIT
                default:
                break;
            }
            break; // end case TA_INIT
        }
        case TA_TX_SELECT:
        {
//        	send_statetousb(inst);
            // select whether to blink or send out a range poll message.
            // select a uwb from the list if sending out a range poll message
        	int uwb_index = inst->tx_scheduler.tx_select(&inst->tx_scheduler);

//        	uint8 debug_msg[100];
//			int n = sprintf((char *)&debug_msg, "selected index %i ", uwb_index);
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

        	if(uwb_index < 0){
        		//do nothing
        		//TODO would be best to sleep until next possible poll and then select again...
        		done = INST_DONE_WAIT_FOR_NEXT_EVENT; //TODO make sure that this is the right thing to do.
        	}else if(uwb_index > 244){
        		inst->testAppState = TA_TXBLINK_WAIT_SEND;
        		inst->uwbToRangeWith = (uint8)255;
        	}else{
        		inst->testAppState = TA_TXPOLL_WAIT_SEND;
        		inst->uwbToRangeWith = (uint8)uwb_index;
        	}

            break; // end case TA_TX_SELECT
        }
        case TA_SLEEP_DONE :
        {
            // if (inst->mode == ANCHOR)
            // {
//                 send_statetousb(inst);
            // }
            event_data_t* dw_event = instance_getevent(10); //clear the event from the queue
            // waiting for timout from application to wakup IC
            if (dw_event->type != DWT_SIG_RX_TIMEOUT)
            {
                // if no pause and no wake-up timeout continue waiting for the sleep to be done.
                done = INST_DONE_WAIT_FOR_NEXT_EVENT; //wait here for sleep timeout
                break;
            }

            done = INST_NOT_DONE_YET;
            inst->goToSleep = 0;
            inst->testAppState = inst->nextState;
            inst->nextState = 0; //clear
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
#endif
            instancesetantennadelays(); //this will update the antenna delay if it has changed

            break;
        }
        case TA_TXE_WAIT : //either go to sleep or proceed to TX a message
        {
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }
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

                inst->canPrintInfo = 1;

#if (DEEP_SLEEP == 1)
                if (inst->sleepingEabled)
                {
                    //put device into low power mode
                    dwt_entersleep(); //go to sleep
                }
#endif
                //DW1000 gone to sleep - report the received range
                if(inst->newRangeUWBIndex != 255)
                {
                    if(inst->tof[inst->newRangeUWBIndex] > 0) //if ToF == 0 - then no new range to report
                    {
                        if(reportTOF(inst, inst->newRangeUWBIndex)==0)
                        {
                            inst->newRange = 1;
                        }
                    }
                }
                
                //inst->deviceissleeping = 1; //this is to stop polling device status register (as it will wake it up)
            }
            else //proceed to configuration and transmission of a frame
            {
                inst->testAppState = inst->nextState;
                inst->nextState = 0; //clear
            }
            break ; // end case TA_TXE_WAIT
        }
        case TA_TXBLINK_WAIT_SEND :
            {
                // if (inst->mode == ANCHOR)
                // {
                //     send_statetousb(inst);
                // }
                
                int flength = (BLINK_FRAME_CRTL_AND_ADDRESS + FRAME_CRC);

                //blink frames with IEEE EUI-64 tag ID
                inst->blinkmsg.frameCtrl = 0xC5 ;
                inst->blinkmsg.seqNum = inst->frameSN++;

                dwt_writetxdata(flength, (uint8 *)  (&inst->blinkmsg), 0) ; // write the frame data
                dwt_writetxfctrl(flength, 0, 1);
                
                //using wait for response to do delayed receive
                inst->wait4ack = DWT_RESPONSE_EXPECTED;

                dwt_setrxtimeout((uint16)inst->fwtoTimeB_sy);  //units are symbols
                //set the delayed rx on time (the ranging init will be sent after this delay)
                dwt_setrxaftertxdelay((uint32)inst->rnginitW4Rdelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

                int tx_start = dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack); //always using immediate TX and enable dealyed RX
                
                if(tx_start == 0)
                {
//                    uint32 deltat;
//                    uint8 debug_msg[100];
//                    if(inst->blink0range1 == 1){
//                        deltat = portGetTickCnt() - inst->range_start;
//                        sprintf((char *)&debug_msg, "range time: %u ", (unsigned int)deltat);
//                    }
//                    else{
//                        deltat = portGetTickCnt() - inst->blink_start;
//                        sprintf((char *)&debug_msg, "blink time: %u ", (unsigned int)deltat);
//                    }
//                    send_txmsgtousb((char *)&debug_msg);

                    
					uint8 debug_msg[100];
					int n = sprintf((char *)&debug_msg, "TX_BLINK,%llX,NULL", instance_get_addr());
					send_usbmessage(&debug_msg[0], n);
					usb_run();

					inst->blink0range1 = 0;
                    inst->blink_start = portGetTickCnt();
                    // send_txmsgtousb("BLINK-SUCCESS");
                    inst->timeofTx = portGetTickCnt();
                }
                else
                {
//                    send_txmsgtousb("BLINK-ERROR");
                }


                inst->goToSleep = 1; //go to Sleep after this blink
                inst->testAppState = TA_TX_WAIT_CONF ; // wait confirmation
                inst->previousState = TA_TXBLINK_WAIT_SEND ;
                done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

            }
            break ; // end case TA_TXBLINK_WAIT_SEND

        case TA_TXRANGINGINIT_WAIT_SEND :
        {
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }
//            send_txmsgtousb("RTLS_DEMO_MSG_RNG_INIT");

            uint16 resp_dly_us, resp_dly;

            int psduLength = RANGINGINIT_MSG_LEN;

            inst->rng_initmsg.messageData[FCODE] = RTLS_DEMO_MSG_RNG_INIT;
            //these bytes not used, zero them out.
            inst->rng_initmsg.messageData[RNG_INIT_TAG_SHORT_ADDR_LO] = 0x00;
            inst->rng_initmsg.messageData[RNG_INIT_TAG_SHORT_ADDR_HI] = 0x00;

            // First response delay to send is anchor's response delay.
            resp_dly_us = ANC_TURN_AROUND_TIME_US + inst->frameLengths_us[POLL];
            resp_dly = ((RESP_DLY_UNIT_US << RESP_DLY_UNIT_SHIFT) & RESP_DLY_UNIT_MASK)
                        + ((resp_dly_us << RESP_DLY_VAL_SHIFT) & RESP_DLY_VAL_MASK);
            inst->rng_initmsg.messageData[RNG_INIT_ANC_RESP_DLY_LO] = resp_dly & 0xFF;
            inst->rng_initmsg.messageData[RNG_INIT_ANC_RESP_DLY_HI] = (resp_dly >> 8) & 0xFF;
            // Second response delay to send is tag's response delay.
            resp_dly_us = TAG_TURN_AROUND_TIME_US + inst->frameLengths_us[RESP];
            resp_dly = ((RESP_DLY_UNIT_US << RESP_DLY_UNIT_SHIFT) & RESP_DLY_UNIT_MASK)
                        + ((resp_dly_us << RESP_DLY_VAL_SHIFT) & RESP_DLY_VAL_MASK);
            inst->rng_initmsg.messageData[RNG_INIT_TAG_RESP_DLY_LO] = resp_dly & 0xFF;
            inst->rng_initmsg.messageData[RNG_INIT_TAG_RESP_DLY_HI] = (resp_dly >> 8) & 0xFF;

            inst->rng_initmsg.frameCtrl[0] = 0x41;

#if (USING_64BIT_ADDR == 1)
            inst->rng_initmsg.frameCtrl[1] = 0xCC;
            psduLength += FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
            inst->rng_initmsg.frameCtrl[1] = 0x8C;
            psduLength += FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;
#endif
            inst->rng_initmsg.panID[0] = (inst->panID) & 0xff;
            inst->rng_initmsg.panID[1] = inst->panID >> 8;

            inst->rng_initmsg.seqNum = inst->frameSN++;

            inst->wait4ack = DWT_RESPONSE_EXPECTED;

            inst->testAppState = TA_TX_WAIT_CONF;                           // wait confirmation
            inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;

            dwt_writetxdata(psduLength, (uint8 *)  &inst->rng_initmsg, 0) ; // write the frame data

            //anchor - we don't use timeout, just wait for next frame
            if(instancesendpacket(psduLength, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, inst->delayedReplyTime))
            {
                dwt_setrxaftertxdelay(0);
                inst->testAppState = TA_RXE_WAIT ;  // wait to receive a new blink or poll message
                inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                inst->lateTX++;
            }
            else
            {
                inst->testAppState = TA_TX_WAIT_CONF ;                                               // wait confirmation
                inst->previousState = TA_TXRANGINGINIT_WAIT_SEND ;
                done = INST_DONE_WAIT_FOR_NEXT_EVENT;  //no timeout

                //CONFIGURE FIXED PARTS OF RESPONSE MESSAGE FRAME (these won't change)
                //program option octet and parameters (not used currently)
                inst->msg[inst->uwbToRangeWith].messageData[RES_R1] = 0x2; // "activity"
                inst->msg[inst->uwbToRangeWith].messageData[RES_R2] = 0x0; //
                inst->msg[inst->uwbToRangeWith].messageData[RES_R3] = 0x0;
                inst->msg[inst->uwbToRangeWith].messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

                instanceconfigframeheader(inst);
                inst->timeofTx = portGetTickCnt();
//                ints->
                //inst->monitor = 1;
            }

            break;
        }
        case TA_TXPOLL_WAIT_SEND :
        {
//            uint8 debug_msg[100];
//            sprintf((char *)&debug_msg, "RTLS_DEMO_MSG_TAG_POLL -> uwb %i ", inst->uwbToRangeWith);
//            send_txmsgtousb((char *)&debug_msg);
            int psduLength = 0;
            
            inst->goToSleep = 1; //go to Sleep after this poll

            inst->msg[inst->uwbToRangeWith].seqNum = inst->frameSN++;
            inst->msg[inst->uwbToRangeWith].messageData[FCODE] = RTLS_DEMO_MSG_TAG_POLL; //message function code (specifies if message is a poll, response or other...)

            instanceconfigframeheader(inst);

#if (USING_64BIT_ADDR==1)
            psduLength = TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
            psduLength = TAG_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif
            //set the delayed rx on time (the response message will be sent after this delay)
            dwt_setrxaftertxdelay(inst->txToRxDelayTag_sy*0); //TODO fix this! remove *0
            dwt_setrxtimeout((uint16)inst->fwtoTime_sy*2);    //TODO fix this! remove *2

            dwt_writetxdata(psduLength, (uint8 *)  &inst->msg[inst->uwbToRangeWith], 0) ; // write the frame data

            //response is expected
            inst->wait4ack = DWT_RESPONSE_EXPECTED;

            dwt_writetxfctrl(psduLength, 0, 1);
            if(dwt_starttx(DWT_START_TX_IMMEDIATE | inst->wait4ack) == 0){
//            	uint32 time_now = portGetTickCnt();
            	uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, "TX_POLL,%llX,%llX,%lu", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith),time_now);
				int n = sprintf((char *)&debug_msg, "TX_POLL,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
				send_usbmessage(&debug_msg[0], n);
				usb_run();

				uint8 buffer[SYS_STATUS_LEN];
				uint64 sys_status_reg;
				dwt_readfromdevice(SYS_STATUS_ID,0,SYS_STATUS_LEN,buffer);

				for (int j = SYS_STATUS_LEN-1 ; j >= 0 ; j --)
				{
					sys_status_reg = (sys_status_reg << 8) + buffer[j] ;
				}

				uint64 sys_state_reg;
				dwt_readfromdevice(SYS_STATE_ID,0,SYS_STATE_LEN,buffer);

				for (int j = SYS_STATE_LEN-1 ; j >= 0 ; j --)
				{
					sys_state_reg = (sys_state_reg << 8) + buffer[j] ;
				}

//				debug_msg[100];
				n = sprintf((char *)&debug_msg, "SYS_STATUS: %llu SYS_STATE: %llu", sys_status_reg, sys_state_reg);
				send_usbmessage(&debug_msg[0], n);
				usb_run();

            }

            inst->testAppState = TA_TX_WAIT_CONF ;                                          // wait confirmation
            inst->previousState = TA_TXPOLL_WAIT_SEND ;
            done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set below)

//            uint32 deltat;
////            uint8 debug_msg[100];
//            if(inst->blink0range1 == 1){
//                deltat = portGetTickCnt() - inst->range_start;
//                sprintf((char *)&debug_msg, "range time: %u ", (unsigned int)deltat);
//            }
//            else{
//                deltat = portGetTickCnt() - inst->blink_start;
//                sprintf((char *)&debug_msg, "blink time: %u ", (unsigned int)deltat);
//            }
//            send_txmsgtousb((char *)&debug_msg);
            

//            inst->uwbToRangeWith = 255; //let the rx_scheduler reselect the uwb
            inst->blink0range1 = 1;
            inst->range_start = portGetTickCnt();


            inst->timeofTx = portGetTickCnt();
        
            break;
        }
        case TA_TXRESPONSE_WAIT_SEND : //the frame is loaded and sent from the RX callback
        {
            // if(inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }
            
            inst->testAppState = TA_TX_WAIT_CONF;                                       // wait confirmation
            inst->previousState = TA_TXRESPONSE_WAIT_SEND ;
            
            break;
        }
        case TA_TXFINAL_WAIT_SEND :
        {
            int psduLength = 0;
            // Embbed into Final message:40-bit respRxTime
            // Write Response RX time field of Final message
            memcpy(&(inst->msg[inst->uwbToRangeWith].messageData[RRXT]), (uint8 *)&inst->anchorRespRxTime, 5);

            inst->msg[inst->uwbToRangeWith].messageData[FCODE] = RTLS_DEMO_MSG_TAG_FINAL; //message function code (specifies if message is a poll, response or other...)

            instanceconfigframeheader(inst);
            
#if (USING_64BIT_ADDR==1)
            psduLength = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
            psduLength = TAG_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

            dwt_writetxdata(psduLength, (uint8 *)  &inst->msg[inst->uwbToRangeWith], 0) ; // write the frame data

            uint32 response_time = portGetTickCnt() - inst->range_start;

//            uint16 reg = dwt_read16bitoffsetreg(PMSC_ID, 0) ;
//			reg |= 0x20 ; // set bit 5 high
//			dwt_write16bitoffsetreg(PMSC_ID, 0, reg) ;

            if(instancesendpacket(psduLength, DWT_START_TX_DELAYED, inst->delayedReplyTime))
            {
                // initiate the re-transmission
                inst->testAppState = TA_TXE_WAIT ;
                //inst->nextState = TA_TXPOLL_WAIT_SEND ; //TODO should this go to TX_SELECT instead?
//                inst->testAppState = TA_TX_SELECT; //TODO reset next and previous?
                inst->nextState = TA_TX_SELECT; //TODO reset next and previous?

                inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
                inst->lateTX++;

//
                uint8 debug_msg[100];
				sprintf((char *)&debug_msg, "RTLS_DEMO_MSG_TAG_FINAL -> late tx");
				send_txmsgtousb((char *)&debug_msg);
				usb_run();

            }
            else
            {
                
                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                inst->previousState = TA_TXFINAL_WAIT_SEND;
                done = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out  (set below)
                
                inst->timeofTx = portGetTickCnt();
                inst->monitor = 1;

               uint8 debug_msg[100];
				sprintf((char *)&debug_msg, "RTLS_DEMO_MSG_TAG_FINAL -> success!");
				send_txmsgtousb((char *)&debug_msg);
				usb_run();
            }
//            uint8 debug_msg[100];
//			 int n = sprintf((char*)&debug_msg[0], "time_now: %lu, replytime: %lu", dwt_readsystimestamphi32(), inst->delayedReplyTime);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();




            break;
        }
        case TA_TX_WAIT_CONF :
        {
            // if (inst->mode == ANCHOR)
            // {
//                 send_statetousb(inst);
            // }
            event_data_t* dw_event = instance_getevent(11); //get and clear this event

            //NOTE: Can get the ACK before the TX confirm event for the frame requesting the ACK
            //this happens because if polling the ISR the RX event will be processed 1st and then the TX event
            //thus the reception of the ACK will be processed before the TX confirmation of the frame that requested it.
            if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
            {
                if(dw_event->type == DWT_SIG_RX_TIMEOUT) //got RX timeout - i.e. did not get the response (e.g. ACK)
                {
                    //we need to wait for SIG_TX_DONE and then process the timeout and re-send the frame if needed
                    inst->gotTO = 1;
                }

                done = INST_DONE_WAIT_FOR_NEXT_EVENT;


                //TODO maybe move out of this if statement???
                //sometimes the DW1000 tx callback (TXFRS) fails to trigger and the the SYS_STATE register
                //reads IDLE for for PMSC, RX, and TX so we need another way to timeout since RX FWTO won't be triggered.
                uint32 dt = get_dt32(inst->timeofTx, portGetTickCnt());

                if( inst->previousState == TA_TXFINAL_WAIT_SEND ||
                    inst->previousState == TA_TXPOLL_WAIT_SEND ||
                    inst->previousState == TA_TXRESPONSE_WAIT_SEND)
                {
                    //NOTE timeout duration found experimentally, may need to be changed if the delays in instance.h are modified
                    if(dt > 20)
                    {
                        inst->gotTO = 1;
                    }                
                }
                else if(inst->previousState == TA_TXBLINK_WAIT_SEND){ //TODO put this back above???
                	if(dt > BLINK_SLEEP_DELAY + 20)
					{
						inst->gotTO = 1;
					}
                }
                else if(inst->previousState == TA_TXRANGINGINIT_WAIT_SEND)
                {
                    if(dt > RNG_INIT_REPLY_DLY_MS + 20)
                    {
                    	inst->gotTO = 1;
                    }
                }
                
                break;
            }

            done = INST_NOT_DONE_YET;

            if (inst->gotTO) //timeout
			{
            	uint8 debug_msg[100];
				int n = sprintf((char *)&debug_msg, "inst->gotTO");
				send_usbmessage(&debug_msg[0], n);
				usb_run();
				// send_txmsgtousb("(2nd) got TO in TA_TX_WAIT_CONF");
				inst_processtxrxtimeout(inst);
				inst->gotTO = 0;
				inst->wait4ack = 0 ; //clear this

				break;
			}
            else if(inst->previousState == TA_TXFINAL_WAIT_SEND) //TAG operations
			{

            	//TODO is this in the wrong spot?
            	//add cooldown node
            	uint32 time_now = portGetTickCnt();
				tx_timing_node *node = malloc(sizeof(tx_timing_node));
				node->index = inst->uwbToRangeWith;
				uint32 time_remaining = 0;
				if(time_now < inst->tx_scheduler.expected_tx_end_time){
					time_remaining = inst->tx_scheduler.expected_tx_end_time - time_now;
				}
				node->duration = (inst->uwbNumActive[inst->uwbToRangeWith]-1)*(uint32)RANGE_DURATION_MS + time_remaining+20;

//				node->duration = (inst->uwbNumActive[inst->uwbToRangeWith]-1)*(int)RANGE_DURATION_MS + inst->tx_scheduler.time_reject_select - time_now;
				node->time = time_now;

				inst->tx_scheduler.add_node(&inst->tx_scheduler, node);

//				uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, "tx_scheduler add_node duration %lu", node->duration);
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();

                inst->testAppState = TA_TXE_WAIT ;
                inst->nextState = TA_TX_SELECT;
//                inst->uwbToRangeWith = instfindfirstactiveuwbinlist(inst, inst->uwbToRangeWith + 1);
                
                break;
            }
            else 
            {
                inst->txu.txTimeStamp = dw_event->timeStamp;

                if(inst->previousState == TA_TXPOLL_WAIT_SEND) //TAG operations
                {
                    uint64 tagCalculatedFinalTxTime ;
                    // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime

                    tagCalculatedFinalTxTime = (inst->txu.txTimeStamp + inst->finalReplyDelay) & MASK_TXDTS;  // time we should send the response
                    inst->delayedReplyTime = tagCalculatedFinalTxTime >> 8;

                    // Calculate Time Final message will be sent and write this field of Final message
                    // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
                    // zeroing its low 9 bits, and then having the TX antenna delay added
                    // getting antenna delay from the device and add it to the Calculated TX Time
                    tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txAntennaDelay;
                    tagCalculatedFinalTxTime &= MASK_40BIT;

                    // Write Calculated TX time field of Final message
                    memcpy(&(inst->msg[inst->uwbToRangeWith].messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);
                    // Write Poll TX time field of Final message
                    memcpy(&(inst->msg[inst->uwbToRangeWith].messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);
                }

                inst->testAppState = TA_RXE_WAIT ;       // After sending, tag expects response/report, anchor waits to receive a final/new poll
                
                //fall into the next case (turn on the RX)
                message = 0;
            } 
        }// end case TA_TX_WAIT_CONF
        case TA_RXE_WAIT :
        {
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }
            
            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
                //turn RX on

            	uint8 debug_msg[100];
				 int n = sprintf((char*)&debug_msg[0], "instancerxon called from  case TA_RXE_WAIT :");
				 send_usbmessage(&debug_msg[0], n);
				 usb_run();

                instancerxon(inst, 0, 0) ;   // turn RX on, without delay
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

            if (inst->mode != LISTENER)
            {
                //we are going to use anchor/tag timeout
                done = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
            }

            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it
            inst->rxCheckOnTime = portGetTickCnt() + RX_CHECK_ON_PERIOD;

            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) 
            {
                break;
            }
        }
        case TA_RX_WAIT_DATA :
        {     
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }

            // Wait RX data
            switch (message)
            {
                case DWT_SIG_RX_BLINK :
                {
//                     send_rxmsgtousb("RX process: DWT_SIG_RX_BLINK ");
                    event_data_t* dw_event = instance_getevent(12); //get and clear this event
                    
                    if((inst->mode == LISTENER) || (inst->mode == ANCHOR))
                    {
                        inst->canPrintInfo = 1;

                        //if using longer reply delay time (e.g. if interworking with a PC application)
                        inst->delayedReplyTime = (dw_event->timeStamp + inst->rnginitReplyDelay) >> 8 ;  // time we should send the blink response
                        
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
//                            send_rxmsgtousb("RX process: DWT_SIG_RX_OKAY-RTLS_DEMO_MSG_RNG_INIT");
                            
                        	uint8 debug_msg[100];
							int n = sprintf((char *)&debug_msg, "BLINK_COMPLETE,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
							send_usbmessage(&debug_msg[0], n);
							usb_run();

                            uint32 final_reply_delay_us;
                            uint32 resp_dly[RESP_DLY_NB];
                            int i;

                            inst->testAppState = TA_TXE_WAIT;
                            inst->nextState = TA_TXPOLL_WAIT_SEND ; // send next poll

                            // Get response delays from message and update internal timings accordingly
                            resp_dly[RESP_DLY_ANC] =  messageData[RNG_INIT_ANC_RESP_DLY_LO]
                                                        + (messageData[RNG_INIT_ANC_RESP_DLY_HI] << 8);
                            resp_dly[RESP_DLY_TAG] =  messageData[RNG_INIT_TAG_RESP_DLY_LO]
                                                        + (messageData[RNG_INIT_TAG_RESP_DLY_HI] << 8);
                            for (i = 0; i < RESP_DLY_NB; i++)
                            {
                                if (((resp_dly[i] & RESP_DLY_UNIT_MASK) >> RESP_DLY_UNIT_SHIFT) == RESP_DLY_UNIT_MS)
                                {
                                    // Remove unit bit and convert to microseconds.
                                    resp_dly[i] &= ~RESP_DLY_UNIT_MASK;
                                    resp_dly[i] *= 1000;
                                }
                            }
                            
                            // Update delay between poll transmission and response reception.
                            // Use uint64 for resp_dly here to avoid overflows if it is more than 400 ms.
                            inst->txToRxDelayTag_sy = US_TO_SY_INT((uint64)resp_dly[RESP_DLY_ANC] - inst->frameLengths_us[POLL]) - RX_START_UP_SY;
                           
                            // Update delay between poll transmission and final transmission.
                            final_reply_delay_us = resp_dly[RESP_DLY_ANC] + resp_dly[RESP_DLY_TAG];
                            inst->finalReplyDelay = convertmicrosectodevicetimeu(final_reply_delay_us);
                            inst->finalReplyDelay_ms = CEIL_DIV(final_reply_delay_us, 1000);
                           
                            // If we are using long response delays, deactivate sleep.
                            if (resp_dly[RESP_DLY_ANC] >= LONG_RESP_DLY_LIMIT_US
                                || resp_dly[RESP_DLY_TAG] >= LONG_RESP_DLY_LIMIT_US)
                            {
                                inst->sleepingEabled = 0;
                            }

                            memcpy(&inst->msg[inst->uwbToRangeWith].destAddr[0], &srcAddr[0], inst->addrByteSize); //set the anchor address for the reply (set destination address)

//                            inst->mode = TAG ;
                            //inst->responseTimeouts = 0; //reset timeout count
                            inst->goToSleep = 0; //don't go to sleep - start ranging instead and then sleep after 1 range is done or poll times out
                            inst->instanceTimerTimeSaved = inst->instanceTimerTime = portGetTickCnt(); //set timer base
                        
                            break; 
                        } //RTLS_DEMO_MSG_RNG_INIT
                        case RTLS_DEMO_MSG_TAG_POLL:
                        {
                            if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                            {
                                //only enable receiver when not using double buffering
                                inst->testAppState = TA_RXE_WAIT ;  // wait for next frame
                                break;
                            }

                            if(dw_event->typePend == DWT_SIG_TX_PENDING)
                            {
                                inst->canPrintInfo = 0;
                                inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
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
//                             send_rxmsgtousb("RX process: DWT_SIG_RX_OKAY-RTLS_DEMO_MSG_ANCH_RESP ");
                            if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                            {
                                inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                break;
                            }

                            inst->anchorRespRxTime = dw_event->timeStamp ; //Response's Rx time

                            uint32 dt = portGetTickCnt() - inst->timeofTx;
							char debug_msg[100];
							int n = sprintf((char*)&debug_msg[0], "time till ANCH_RESP %lu", dt);
							send_usbmessage(&debug_msg[0], n);
							usb_run();

                            inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final

                            inst->canPrintInfo = 2;

                            inst->tof[inst->uwbToRangeWith] = 0;
                            //copy previously calculated ToF
                            memcpy(&inst->tof[inst->uwbToRangeWith], &messageData[TOFR], 6);
                            memcpy(&inst->uwbNumActive[inst->uwbToRangeWith], &messageData[NTAG], 1);

                            uint32 time_till = 0;
                            memcpy(&time_till, &messageData[TIME_TILL], sizeof(uint32));
                            if(time_till != 0){

                            	tx_timing_node *node = malloc(sizeof(tx_timing_node));
								node->index = inst->uwbToRangeWith;
								node->duration = 200; //TODO cleanup, set to blink window duration
//								node->time = portGetTickCnt() - inst->tof[inst->uwbToRangeWith] + time_till; //TODO correctly implement (convert tof timescale?)
								node->time = portGetTickCnt() + time_till;

								inst->tx_scheduler.add_node(&inst->tx_scheduler, node);



                            }


                            inst->newRangeUWBIndex = inst->uwbToRangeWith;
                            inst->newRangeAncAddress = instance_get_uwbaddr(inst->uwbToRangeWith);
                            inst->newRangeTagAddress = instance_get_addr();

                            break; 
                        } //RTLS_DEMO_MSG_ANCH_RESP
                        case RTLS_DEMO_MSG_TAG_FINAL:
                        {
//                            send_rxmsgtousb("RX process: DWT_SIG_RX_OKAY-RTLS_DEMO_MSG_TAG_FINAL ");
                            int64 Rb, Da, Ra, Db ;
                            uint64 tagFinalTxTime  = 0;
                            uint64 tagFinalRxTime  = 0;
                            uint64 tagPollTxTime  = 0;
                            uint64 anchorRespRxTime  = 0;

                            double RaRbxDaDb = 0;
                            double RbyDb = 0;
                            double RayDa = 0;

                            if(inst->mode == LISTENER) //don't process any ranging messages when in Listener mode
                            {
                                inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                                break;
                            }

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
                            Db = (int64)((inst->txu.anchorRespTxTime - inst->tagPollRxTime) & MASK_40BIT);

                            // response final round trip delay time is calculated as
                            // (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
                            Rb = (int64)((tagFinalRxTime - inst->txu.anchorRespTxTime) & MASK_40BIT);
                            Da = (int64)((tagFinalTxTime - anchorRespRxTime) & MASK_40BIT);

                            RaRbxDaDb = (((double)Ra))*(((double)Rb)) - (((double)Da))*(((double)Db));
                            RbyDb = ((double)Rb + (double)Db);
                            RayDa = ((double)Ra + (double)Da);

                            //time-of-flight
                            inst->tof[inst->uwbToRangeWith] = (int64) ( RaRbxDaDb/(RbyDb + RayDa) );
                            inst->newRangeUWBIndex = inst->uwbToRangeWith;

//							char debug_msg[100];
//							int n = sprintf((char*)&debug_msg[0], "(int64) ( RaRbxDaDb/(RbyDb + RayDa) ); %lld", inst->tof[inst->uwbToRangeWith]);
//							send_usbmessage(&debug_msg[0], n);
//							usb_run();

                            if(reportTOF(inst, inst->newRangeUWBIndex) == 0)
                            {
                                inst->newRange = 1;
                            }

                            //TODO update or remove?
                            // inst->newRangeTagAddress = (uint16) srcAddr[0] + ((uint16) srcAddr[1] << 8);
                            // inst->newRangeAncAddress = (uint16) inst->eui64[0] + ((uint16) inst->eui64[1] << 8);
                            inst->newRangeTagAddress = instance_get_uwbaddr(inst->uwbToRangeWith);
                            inst->newRangeAncAddress = instance_get_addr();
                            
                            uint8 debug_msg[100];
							int n = sprintf((char *)&debug_msg, "POLL_COMPLETE,%llX,%llX", inst->newRangeTagAddress, inst->newRangeAncAddress);
							send_usbmessage(&debug_msg[0], n);
							usb_run();

                            //get num active
                            rx_timing_node *node = malloc(sizeof(rx_timing_node));
							node->index = inst->uwbToRangeWith;
							uint32 num_active = 0;
							for(int i = 0; i < inst->uwbListLen; i++)
							{
								if(inst->uwbTimeout[i] == 0)
								{
									num_active++;
								}
							}
							uint32 time_now = portGetTickCnt();
							uint32 time_remaining = 0;
//							if(time_now < inst->rx_scheduler.time_reject_all){
//								time_remaining = inst->rx_scheduler.time_reject_all - time_now;
//							}
							if(time_now < inst->rx_scheduler.accepted_node_end_time){
								time_remaining = inst->rx_scheduler.accepted_node_end_time - time_now;
							}
							node->duration = (int)RANGE_DURATION_MS*(num_active-1) + time_remaining;
							node->time = time_now;
							node->type = RX_REJECT;

							inst->rx_scheduler.add_node(&inst->rx_scheduler, node);

//							char debug_msg[100];
//							n = sprintf((char*)&debug_msg[0], "TAG_FINAL %i", *node->index);
//							send_usbmessage(&debug_msg[0], n);
//							usb_run();


                            inst->testAppState = TA_RXE_WAIT ;
                            inst->previousState = TA_INIT;
                            inst->nextState = TA_INIT;
                            inst->uwbToRangeWith = 255;

                            dwt_setrxaftertxdelay(0);
							instancesetantennadelays(); //this will update the antenna delay if it has changed


                            //char debug_msg[100];
//							n = sprintf((char*)&debug_msg[0], "TAG_FINAL %d", inst->uwbToRangeWith);
//							send_usbmessage(&debug_msg[0], n);
//							usb_run();


                            break; 
                        } //RTLS_DEMO_MSG_TAG_FINAL
                        default:
                        {
//                            if(inst->mode == ANCHOR)
//                            {
//                                send_rxmsgtousb("RX process: DWT_SIG_RX_OKAY-default ");
//                            }
                            inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
                            dwt_setrxaftertxdelay(0);
                            
                            break;    
                        }
                    } //end switch (fcode)

                    if((inst->goToSleep == 0) && (inst->mode == LISTENER) /*|| (inst->mode == ANCHOR)*/)//update received data, and go back to receiving frames
                    {
                        inst->testAppState = TA_RXE_WAIT ;              // wait for next frame

                        dwt_setrxaftertxdelay(0);
                    }
                
                    break ; 
                } //end of DWT_SIG_RX_OKAY
                case DWT_SIG_RX_TIMEOUT :
                {
//                    send_txmsgtousb("RX process: DWT_SIG_RX_TIMEOUT ");
                    int n;
                    if(inst->previousState == TA_TXBLINK_WAIT_SEND)
					{
						uint8 debug_msg[100];
						n = sprintf((char *)&debug_msg, "TX_BLINK_TIMEOUT,%llX,NULL", instance_get_addr());
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}
					else if(inst->previousState == TA_TXFINAL_WAIT_SEND)
					{
						uint8 debug_msg[100];
						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}
					else if(inst->previousState == TA_TXPOLL_WAIT_SEND)
					{
						uint8 debug_msg[100];
						n = sprintf((char *)&debug_msg, "TX_POLL_TIMEOUT,%llX,%llX", instance_get_addr(), instance_get_uwbaddr(inst->uwbToRangeWith));
						send_usbmessage(&debug_msg[0], n);
						usb_run();
					}
                    
                    uint8 buffer[SYS_STATUS_LEN];
					uint64 sys_status_reg;
					dwt_readfromdevice(SYS_STATUS_ID,0,SYS_STATUS_LEN,buffer);

					for (int j = SYS_STATUS_LEN-1 ; j >= 0 ; j --)
					{
						sys_status_reg = (sys_status_reg << 8) + buffer[j] ;
					}

					uint64 sys_state_reg;
					dwt_readfromdevice(SYS_STATE_ID,0,SYS_STATE_LEN,buffer);

					for (int j = SYS_STATE_LEN-1 ; j >= 0 ; j --)
					{
						sys_state_reg = (sys_state_reg << 8) + buffer[j] ;
					}

					uint8 debug_msg[100];
					n = sprintf((char *)&debug_msg, "SYS_STATUS: %llu SYS_STATE: %llu", sys_status_reg, sys_state_reg);
					send_usbmessage(&debug_msg[0], n);
					usb_run();



                    instance_getevent(17); //get and clear this event
                    inst_processtxrxtimeout(inst);
                    message = 0; //clear the message as we have processed the event
                    
                    break;
                }
                case DWT_SIG_TX_DONE: //shouldn't happen under normal conditions, but this signal seems to be somewhat buggy
                {
                	//clear the event?
                	//event_data_t* dw_event = instance_getevent(11); //get and clear this event
                	instance_getevent(11); //get and clear this event

//                	uint8 debug_msg[100];
//					int n = sprintf((char *)&debug_msg, "DWT_SIG_TX_DONE in RX_WAIT_DATA");
//					send_usbmessage(&debug_msg[0], n);
//					usb_run();
                	break;
                }
                case DWT_SIG_TX_AA_DONE: //ignore this event - just process the rx frame that was received before the ACK response
                case 0:
                default :
                {
                	//TODO maybe also check the events to see if nothing is upcoming that will handle this for us!
                	//check if RX is on every so often. Turn it on if it isn't.
                	uint32 time_now = portGetTickCnt();
					if(time_now >= inst->rxCheckOnTime)
					{
						inst->rxCheckOnTime = portGetTickCnt() + RX_CHECK_ON_PERIOD;

						//read SYS_STATE, getting second byte
						uint8 regval = dwt_read8bitoffsetreg(SYS_STATE_ID,1);
						//get the first 5 bytes
						regval &= 0x1F;
						if(regval == 0){//RX IDLE
							dwt_forcetrxoff();
							instancerxon(inst, 0, 0);
							uint8 debug_msg[200];
							int n = sprintf((char *)&debug_msg, "RX IDLE, RESET: regval %u", regval);
							send_usbmessage(&debug_msg[0], n);
							usb_run();
						}


						uint8 debug_msg[200];
						int n = sprintf((char *)&debug_msg, "RX CHECKED: regval %u", regval);
						send_usbmessage(&debug_msg[0], n);
						usb_run();

					}


                    if(inst->mode == ANCHOR)
                    {   
//                        send_rxmsgtousb("RX process: default ");

                    	uint32 delta_t = portGetTickCnt() - inst->currentStateStartTime;
                    	if(delta_t > 20000){


							 uint8 buffer[SYS_STATUS_LEN];
							uint64 sys_status_reg;
							dwt_readfromdevice(SYS_STATUS_ID,0,SYS_STATUS_LEN,buffer);

							for (int j = SYS_STATUS_LEN-1 ; j >= 0 ; j --)
							{
								sys_status_reg = (sys_status_reg << 8) + buffer[j] ;
							}

							uint64 sys_state_reg;
							dwt_readfromdevice(SYS_STATE_ID,0,SYS_STATE_LEN,buffer);

							for (int j = SYS_STATE_LEN-1 ; j >= 0 ; j --)
							{
								sys_state_reg = (sys_state_reg << 8) + buffer[j] ;
							}

							uint8 debug_msg[200];
							int n = sprintf((char *)&debug_msg, "SYS_STATUS: %llu SYS_STATE: %llu", sys_status_reg, sys_state_reg);
							send_usbmessage(&debug_msg[0], n);
							usb_run();
                    	}
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
            // if (inst->mode == ANCHOR)
            // {
            //     send_statetousb(inst);
            // }
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
int instance_init_s(int mode)
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);


    inst->mode =  mode;                                // assume anchor,
    inst->testAppState = TA_INIT;

//    inst->blink_duration = BLINK_DURATION_MS;
//    inst->range_duration = RANGE_DURATION_MS;
//
//	//TODO move and tie to other #define variables
//	float_t blink_frequency = 0.0001;
//
//	if(inst->mode == ANCHOR){
//		inst->rx_scheduler = RXScheduler.new(blink_frequency, inst->range_duration, &inst->uwbList, &inst->uwbListLen, UWB_LIST_SIZE, inst->uwbTimeout);
//	}
//	else if(inst->mode == TAG){
//		inst->tx_scheduler = TXScheduler.new(blink_frequency, inst->blink_duration, inst->range_duration, &inst->uwbList, &inst->uwbListLen, UWB_LIST_SIZE, inst->uwbTimeout);
//	}

//    inst->blink_duration = BLINK_DURATION_MS;
//	inst->range_duration = RANGE_DURATION_MS;

	//TODO move and tie to other #define variables
	//float_t blink_frequency = 0.0001;

	if(inst->mode == ANCHOR){
		inst->rx_scheduler = RXScheduler.new(BLINK_FREQUENCY, BLINK_DURATION_MS, RANGE_DURATION_MS, &inst->uwbList, &inst->uwbListLen, UWB_LIST_SIZE, inst->uwbTimeout, inst->time_till_next_reported);//TODO do i need & ???
	}
	else if(inst->mode == TAG){
		inst->tx_scheduler = TXScheduler.new(BLINK_FREQUENCY, BLINK_DURATION_MS, RANGE_DURATION_MS, &inst->uwbList, &inst->uwbListLen, UWB_LIST_SIZE, inst->uwbTimeout);
	}


    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    // dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
//    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_RXOVRR | DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(SYS_MASK_VAL, 1);

//    uint32 mask = dwt_read32bitreg(SYS_MASK_ID);


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
        RESP_FRAME_LEN_BYTES, FINAL_FRAME_LEN_BYTES};
    int i;
    // Margin used for timeouts computation.
    const int margin_sy = 500; //50;

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
    // Second step is data length for all frame types.
    for (i = 0; i < FRAME_TYPE_NB; i++)
    {
        // Compute the number of symbols for the given length.
        inst->frameLengths_us[i] = data_len_bytes[i] * 8
                         + CEIL_DIV(data_len_bytes[i] * 8, 330) * 48;
        // Convert from symbols to time and add PHY header length.
        if(inst->configData.dataRate == DWT_BR_110K)
        {
            inst->frameLengths_us[i] *= 820513;
            inst->frameLengths_us[i] += 17230800;
        }
        else if (inst->configData.dataRate == DWT_BR_850K)
        {
            inst->frameLengths_us[i] *= 102564;
            inst->frameLengths_us[i] += 2153900;
        }
        else
        {
            inst->frameLengths_us[i] *= 12821;
            inst->frameLengths_us[i] += 2153900;
        }
        // Last step: add preamble length and convert to microseconds.
        inst->frameLengths_us[i] += pre_len;
        inst->frameLengths_us[i] = CEIL_DIV(inst->frameLengths_us[i], 100000);
    }
    // Final frame wait timeout time.
    inst->fwtoTime_sy = US_TO_SY_INT(inst->frameLengths_us[FINAL])
                        + RX_START_UP_SY + margin_sy;
    // Ranging init frame wait timeout time.
    inst->fwtoTimeB_sy = US_TO_SY_INT(inst->frameLengths_us[RNG_INIT])
                         + RX_START_UP_SY + margin_sy;
    // Delay between blink transmission and ranging init reception.
    inst->rnginitW4Rdelay_sy =
        US_TO_SY_INT((RNG_INIT_REPLY_DLY_MS * 1000) - inst->frameLengths_us[BLINK])
        - RX_START_UP_SY;
    // Delay between anchor's response transmission and final reception.
    inst->txToRxDelayAnc_sy = US_TO_SY_INT(TAG_TURN_AROUND_TIME_US) - RX_START_UP_SY;

    // No need to init txToRxDelayTag_sy here as it will be set upon reception
    // of ranging init message.

    // Delay between blink reception and ranging init message transmission.
    inst->rnginitReplyDelay = convertmicrosectodevicetimeu(RNG_INIT_REPLY_DLY_MS * 1000);
    // Delay between poll reception and response transmission. Computed from
    // poll reception timestamp to response transmission timestamp so you have
    // to add poll frame length to delay that must be respected between frames.
#if (IMMEDIATE_RESPONSE == 0)
    inst->responseReplyDelay = convertmicrosectodevicetimeu(ANC_TURN_AROUND_TIME_US + inst->frameLengths_us[POLL]);
#else
    inst->responseReplyDelay = 0 ;
#endif


    // Smart Power is automatically applied by DW chip for frame of which length
    // is < 1 ms. Let the application know if it will be used depending on the
    // length of the longest frame.
    if (inst->frameLengths_us[FINAL] <= 1000)
        inst->smartPowerEn = 1;
    else
        inst->smartPowerEn = 0;
}

uint64 instance_get_addr(void) //get own address
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint64 x = (uint64) inst->eui64[0];
    x |= (uint64) inst->eui64[1] << 8;
    x |= (uint64) inst->eui64[2] << 16;
    x |= (uint64) inst->eui64[3] << 24;
    x |= (uint64) inst->eui64[4] << 32;
    x |= (uint64) inst->eui64[5] << 40;
    x |= (uint64) inst->eui64[6] << 48;
    x |= (uint64) inst->eui64[7] << 56;


    return (x);
}

uint64 instance_get_uwbaddr(uint8 uwb_index) //get uwb address by index
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint64 x = (uint64) inst->uwbList[uwb_index][0];
    x |= (uint64) inst->uwbList[uwb_index][1] << 8;
    x |= (uint64) inst->uwbList[uwb_index][2] << 16;
    x |= (uint64) inst->uwbList[uwb_index][3] << 24;
    x |= (uint64) inst->uwbList[uwb_index][4] << 32;
    x |= (uint64) inst->uwbList[uwb_index][5] << 40;
    x |= (uint64) inst->uwbList[uwb_index][6] << 48;
    x |= (uint64) inst->uwbList[uwb_index][7] << 56;

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


#endif



/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
