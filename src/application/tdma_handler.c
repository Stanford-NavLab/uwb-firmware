#include "tdma_handler.h"
#include "port.h"
#include "instance.h"


extern void usb_run(void);
extern void send_usbmessage(uint8*, int);

//class methods
static bool slot_transition(struct TDMAHandler *this)
{
	bool transition = FALSE;
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	if(inst->mode == TAG ||
	   inst->mode == ANCHOR ||
	   (inst->mode == DISCOVERY && (this->discovery_mode == WAIT_SEND_SUG)))// || this->discovery_mode == COLLECT_INF_REG)))//TODO this may be out of date
	{
		uint64 time_now_us = portGetTickCntMicro();
		uint64 timeSinceSlotStart64 = get_dt64(this->lastSlotStartTime64, time_now_us);

		if(timeSinceSlotStart64 >= this->slotDuration_us) //TODO should i check frame separate from slot?
		{
			transition = TRUE;
			this->firstPollSentThisSlot = FALSE;
			this->firstPollComplete = FALSE;
			this->secondPollSentThisSlot = FALSE;
			this->infSentThisSlot = FALSE;

			//we have transitioned into the next slot.
			//get the slot number and set the start time appropriately

//			uint64 timeSinceFrameStart64 = get_dt64(this->uwbFrameStartTimes64[0], time_now_us);
			uint64 timeSinceFrameStart64 = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
			//TODO this->uwbListTDMAInfo[0].frameStartTime is sometimes getting set to a time in the future... figure out!

			uint64 frameDuration64 = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;
			if(timeSinceFrameStart64 >= frameDuration64)
			{
//				uint8 debug_msg[100];
//////				 int n = sprintf((char*)&debug_msg[0], "NEW FRAME, %llX,  this->frameStartTime: %lu, this->slotDuration*this->framelength: %lu", instance_get_addr(), this->frameStartTime, (this->slotDuration*this->framelength));
//				int n = sprintf((char*)&debug_msg[0], "%llX, %llu, %llu", instance_get_addr(), this->lastFST, time_now_us);
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();

				//TODO think about adopting rebase AFTER all neighbors have had a chance to TX the rebase in their INF message as well.
//				this->rebase_pending = FALSE;
//				this->rebase_tx = FALSE;
//				this->uwbFrameStartTimes64[0] = this->rebase_frameStartTime64;
//				timeSinceFrameStart64 = get_dt64(this->uwbFrameStartTimes64[0], time_now_us);
			}

			while(timeSinceFrameStart64 >= frameDuration64)
			{
//				this->uwbFrameStartTimes64[0] = timestamp_add64(this->uwbFrameStartTimes64[0], frameDuration64);
				this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, frameDuration64);
				timeSinceFrameStart64 -= frameDuration64;
			}

//			this->lastFST = this->uwbFrameStartTimes64[0];
			this->lastFST = this->uwbListTDMAInfo[0].frameStartTime;

			uint8 slot = timeSinceFrameStart64/(this->slotDuration_us); //integer division rounded down
//			this->lastSlotStartTime64 = this->uwbFrameStartTimes64[0] + (uint64)(this->slotDuration_us*slot);
			this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)(this->slotDuration_us*slot);


//				uint8 debug_msg[100];
//				int n = sprintf((char*)&debug_msg[0], "NEW SLOT %u, time_now %llu", slot, time_now_us);
//				 send_usbmessage(&debug_msg[0], n);
//				 usb_run();


//			n = sprintf((char*)&debug_msg[0], "&this->slotAssignments[slot]: %u, &inst->uwbShortAdd %u :", this->slotAssignments[slot], inst->uwbShortAdd);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();
//
//			if(this->slot_assigned(this, slot) == TRUE)
//			{
//
//			}

//			struct TDMAInfo *info = &this->uwbListTDMAInfo[0];
//			bool assigned0 = this->slot_assigned(info, 0);
//			bool assigned1 = this->slot_assigned(info, 1);
//			bool assigned2 = this->slot_assigned(info, 2);
//			bool assigned3 = this->slot_assigned(info, 3);
//
//			uint8 debug_msg[100];
//			int n = sprintf((char*)&debug_msg[0], "NEW SLOT %u, ass0: %u, ass1: %u, ass2: %u, ass3: %u,", slot, assigned0, assigned1, assigned2, assigned3);
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();


			if(inst->mode != DISCOVERY)
			{
				if(this->slot_assigned(&this->uwbListTDMAInfo[0], slot) == TRUE) //TODO left off here, for some reason it fails when I try this...
				{

					inst->mode = TAG;
					inst->testAppState = TA_TX_SELECT;
					//go to TX select, select the oldest uwb, send INF, then send POLL
				}
				else
				{
					//go to RX
					inst->mode = ANCHOR;
					inst->testAppState = TA_RXE_WAIT;
				}
			}

			instance_getevent(17); //get and clear this event
			inst_processtxrxtimeout(inst);
		}
	}
	else if(inst->mode == DISCOVERY)
	{
		this->infSentThisSlot = FALSE;
		this->firstPollSentThisSlot = FALSE;
		this->firstPollComplete = FALSE;
		this->secondPollSentThisSlot = FALSE;
	}

	return transition;
}

//static void frame_sync(struct TDMAHandler *this, uint8 *messageData, uint16 rxLength, uint8 srcIndex, FRAME_SYNC_MODE mode)
//static void frame_sync(struct TDMAHandler *this, event_data_t *dw_event, uint8 *messageData, uint8 srcIndex, FRAME_SYNC_MODE mode)
static void frame_sync(struct TDMAHandler *this, event_data_t *dw_event, uint8 framelength, uint64 timeSinceFrameStart_us, uint8 srcIndex, FRAME_SYNC_MODE mode)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

//	uint8 debug_msg[100];
//	int n = sprintf((char*)&debug_msg[0], "frame sync");
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

	//do not process erroneous tsfs
	//can happen if frame start time is shifted ahead of time_now_us in transmitting UWB
	if(timeSinceFrameStart_us > 1000000000 || timeSinceFrameStart_us == 0)
	{
		return;
	}


	uint8 sys_time_arr[5] = {0, 0, 0, 0, 0};
	dwt_readsystime(sys_time_arr);
	uint64 dwt_time_now = 0;
	dwt_time_now = (uint64)sys_time_arr[0] + ((uint64)sys_time_arr[1] << 8) + ((uint64)sys_time_arr[2] << 16) + ((uint64)sys_time_arr[3] << 24) + ((uint64)sys_time_arr[4] << 32);
	uint64 time_now_us = portGetTickCntMicro();

//	uint8 framelength;
//	uint64 timeSinceFrameStart_us = 0;

//	memcpy(&framelength, &messageData[TDMA_FRAMELENGTH], sizeof(uint8));
//	//timeSinceFrameStart in message
//	memcpy(&timeSinceFrameStart_us, &messageData[TDMA_TSFS], 6);

	//time from message to tx
	//assuming zero since we use DWT_START_TX_IMMEDIATE

	//tx antenna delay
	uint64 tx_antenna_delay = (uint64)inst->txAntennaDelay;

	//time to propagate
	//NOTE: assuming zero since difference for speed of light travel time over 10cm and 100m is negligible for frame sync purposes

	//rx antenna delay
	//NOTE: we won't use this, because the antenna delay is captured by the RX timestamp

	//time from rx timestamp to now
	uint64 rxfs_process_delay = dwt_getdt(dw_event->timeStamp, dwt_time_now);

	uint64 txrx_delay =  (uint64)(convertdevicetimetosec(tx_antenna_delay + rxfs_process_delay)*1000000.0) + inst->storePreLen_us;

	uint64 hisTimeSinceFrameStart_us = timeSinceFrameStart_us + txrx_delay;
//	this->uwbFrameStartTimes64[srcIndex] = timestamp_subtract64(time_now_us, hisTimeSinceFrameStart_us); //TODO consider applying the diff!
	this->uwbListTDMAInfo[srcIndex].frameStartTime = timestamp_subtract64(time_now_us, hisTimeSinceFrameStart_us); //TODO consider applying the diff!

	if(mode == FS_COLLECT)
	{
		return;
	}
	else if(mode == FS_ADOPT) //TODO this might not be right! incoming framelength not always the same as ours!
	{
//		this->uwbFrameStartTimes64[0] = this->uwbFrameStartTimes64[srcIndex];
//		this->uwbListTDMAInfo[0].frameStartTime = this->uwbListTDMAInfo[srcIndex].frameStartTime;
//		uint8 slot = hisTimeSinceFrameStart_us/this->slotDuration_us; //integer division rounded down
//		this->lastSlotStartTime64 = this->uwbFrameStartTimes64[0] + (uint64)((this->slotDuration_us)*slot);
//		this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)((this->slotDuration_us)*slot);

		this->uwbListTDMAInfo[0].frameStartTime = this->uwbListTDMAInfo[srcIndex].frameStartTime;//NOTE gets processed further at end of function
	}
	else// if(mode == FS_AVERAGE || mode == FS_EVAL) //TODO consider using a weighted average. perhaps use threshholds to determine if instead we should just use the ADOPT logic
	{
//		uint64 myFramelengthDuration_us = this->uwbListTDMAInfo[0].framelength*this->slotDuration_us;
//		uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbFrameStartTimes64[0], time_now_us);
		uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);

		//SELF VS INCOMING

		uint8 min_fl = this->uwbListTDMAInfo[0].framelength;
		if(framelength < min_fl)
		{
			min_fl = framelength;
		}

		uint64 min_framelengthDuration = min_fl*this->slotDuration_us;
		uint64 diff_tsfs = 0;
		uint64 diff_tsfs_mod = 0;
		uint64 diff_us = 0;

		bool diff_add = FALSE;
		if(hisTimeSinceFrameStart_us <= myTimeSinceFrameStart_us)
		{
			diff_tsfs = myTimeSinceFrameStart_us - hisTimeSinceFrameStart_us;
			diff_tsfs_mod = diff_tsfs%min_framelengthDuration;

			if(diff_tsfs_mod <= 0.5*min_framelengthDuration)
			{
				diff_us = diff_tsfs_mod;
				diff_add = TRUE;
			}
			else
			{
				diff_us = min_framelengthDuration - diff_tsfs_mod;
				//subtract
			}
		}
		else
		{
			diff_tsfs = hisTimeSinceFrameStart_us - myTimeSinceFrameStart_us;
			diff_tsfs_mod = diff_tsfs%min_framelengthDuration;

			if(diff_tsfs_mod <= 0.5*min_framelengthDuration)
			{
				diff_us = diff_tsfs_mod;
				//subtract
			}
			else
			{
				diff_us = min_framelengthDuration - diff_tsfs_mod;
				diff_add = TRUE;
			}
		}

		if(diff_us > this->frameSyncThreshold_us)
		{
			this->tx_sync_msg(this);
		}
		else if(mode == FS_EVAL)
		{
			return;
		}

		uint8 div = 2;
		if(mode == FS_EVAL)
		{
			div = 1;
		}

		if(diff_add == TRUE)
		{
//			uint8 debug_msg[100];
//			 int n = sprintf((char*)&debug_msg[0], "add %llu", diff_us);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();

//			this->uwbFrameStartTimes64[0] = timestamp_add64(this->uwbFrameStartTimes64[0], diff_us/div);
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, diff_us/div);
//			this->lastSlotStartTime64 = timestamp_add64(this->lastSlotStartTime64, diff_us/div);
		}
		else
		{
//			uint8 debug_msg[100];
//			 int n = sprintf((char*)&debug_msg[0], "subtract %llu", diff_us);
//			 send_usbmessage(&debug_msg[0], n);
//			 usb_run();
//			this->uwbFrameStartTimes64[0] = timestamp_subtract64(this->uwbFrameStartTimes64[0], diff_us/div);
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_subtract64(this->uwbListTDMAInfo[0].frameStartTime, diff_us/div);
//			this->lastSlotStartTime64 = timestamp_subtract64(this->lastSlotStartTime64, diff_us/div);
		}
	}

	uint64 myFrameDuration = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;
	uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us); //TODO what if added to be past time_now???
	if(myTimeSinceFrameStart_us > 100000000)
	{
		//if this is a very large number, then the frame start time was likely moved head of time_now_us.
		this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime - (uint64)(this->slotDuration_us);
	}
	else
	{
		while(myTimeSinceFrameStart_us >= myFrameDuration)
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, myFrameDuration);
			myTimeSinceFrameStart_us -= myFrameDuration;
		}

		uint8 slot = hisTimeSinceFrameStart_us/this->slotDuration_us; //integer division rounded down
		this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)(this->slotDuration_us*slot);
	}
}


static bool tx_sync_msg(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint64 time_now_us = portGetTickCntMicro();
//	uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbFrameStartTimes64[0], time_now_us);
	uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
	memcpy(&inst->sync_msg.messageData[SYNC_FRAMELENGTH], &this->uwbListTDMAInfo[0].framelength, sizeof(uint8));
	memcpy(&inst->sync_msg.messageData[SYNC_TSFS], &myTimeSinceFrameStart_us, 6);
	int psduLength = 0;

	inst->sync_msg.seqNum = inst->frameSN++;


#if (USING_64BIT_ADDR==1)
	psduLength = SYNC_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;
#else
	psduLength = SYNC_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

	dwt_writetxdata(psduLength, (uint8 *)&inst->sync_msg, 0) ; // write the frame data

	inst->wait4ack = 0;

	if(instancesendpacket(psduLength, DWT_START_RX_IMMEDIATE, 0))
	{
		inst->testAppState = TA_RXE_WAIT;
		return FALSE;
	}
	else
	{

		inst->previousState = inst->testAppState;
		inst->testAppState = TA_TX_WAIT_CONF;	// wait confirmation
		return TRUE;
	}
}



//
////static void frame_sync(struct TDMAHandler *this, uint8 *messageData, uint16 rxLength, uint8 srcIndex, FRAME_SYNC_MODE mode)
//static void frame_sync(struct TDMAHandler *this, event_data_t *dw_event, uint8 *messageData, uint8 srcIndex, FRAME_SYNC_MODE mode)
//{
//	//ignore frame sync if we have a rebase pending
////	if(this->rebase_pending == TRUE)
////	{
////		return;
////	}
//
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//
//	uint8 sys_time_arr[5] = {0, 0, 0, 0, 0};
//	dwt_readsystime(sys_time_arr);
//	uint64 dwt_time_now = (uint64)sys_time_arr[0] + ((uint64)sys_time_arr[1] << 8) + ((uint64)sys_time_arr[2] << 16) + ((uint64)sys_time_arr[3] << 24) + ((uint64)sys_time_arr[4] << 32);
//	uint64 time_now_us = portGetTickCntMicro();
//
//	uint8 framelength;
//	uint32 timeSinceFrameStart64 = 0;
////	bool rebase = FALSE;
//
//	memcpy(&framelength, &messageData[TDMA_FRAMELENGTH], sizeof(uint8));
//	//timeSinceFrameStart in message
//	memcpy(&timeSinceFrameStart64, &messageData[TDMA_TSFS], sizeof(uint64));
////	memcpy(&rebase, &messageData[TDMA_TSFS_REBASE], sizeof(bool));
//
//	//time from message to tx
//	//assuming zero since we use DWT_START_TX_IMMEDIATE
//
//	//tx antenna delay
//	uint64 tx_antenna_delay = (uint64)inst->txAntennaDelay;
//
//	//time for xmission (only count once, happens on both sides near simultaneously)
//	//easiest way to check would be to see if it is the same as the defines for other standard messages...
//	inst->frameLengths_us[INF] = instance_getmessageduration_us(dw_event->rxLength); //TODO should maybe make sure extended framelength cannot overflow a uint32
//
//	//time to propagate
//	//NOTE: assuming zero since difference for speed of light travel time over 10cm and 100m is negligible for frame sync purposes
//
//	//rx antenna delay
//	//NOTE: we won't use this, because the antenna delay is captured by the RX timestamp
//
//	//time from rx timestamp to now
//	uint64 rxfs_process_delay = dwt_getdt(dw_event->timeStamp, dwt_time_now);
//
//	uint64 txrx_delay =  (uint64)(convertdevicetimetosec(tx_antenna_delay + rxfs_process_delay)*1000000.0) + inst->storePreLen_us;
//
//	uint64 hisTimeSinceFrameStart_us = timeSinceFrameStart64 + txrx_delay;
//	this->uwbFrameStartTimes64[srcIndex] = timestamp_subtract64(time_now_us, hisTimeSinceFrameStart_us); //TODO consider applying the diff!
//
//	if(mode == FS_ADOPT)
//	{
//		this->uwbFrameStartTimes64[0] = this->uwbFrameStartTimes64[srcIndex];
//		uint8 slot = hisTimeSinceFrameStart_us/this->slotDuration_us; //integer division rounded down
//		this->lastSlotStartTime64 = this->uwbFrameStartTimes64[0] + (uint64)((this->slotDuration_us)*slot);
//	}
//	else if(mode == FS_COLLECT)
//	{
//		return;
//	}
//	else if(mode == FS_AVERAGE) //TODO consider using a weighted average. perhaps use threshholds to determine if instead we should just use the ADOPT logic
//	{
////		if(this->discovery_mode == COLLECT_INF_REG)
////		{
////			//TODO handle the case where we are collecting from two different UWB networks that are not synchronized
////			//in case there are multiple subnetworks... only collect average frame start times for one of them...
////			//send out INF_SUG in that 0th frame with rebase to the other subnetwork's frame. but what if there are two other networks?
////			//instead send out INF fug in that 0th frame with rebase to that frame. Hopefully the other networks will get the message.
////			//if not, the discrepency will eventually be taken care of the ANCHOR and TAG logic, regarless of how many subnetworks there are.
////			//sync to the largest subnetwork...
////		}
//
//
////		if(rebase == TRUE)
////		{
////			this->rebase_pending = TRUE;
////			this->rebase_tx = FALSE;
////			this->rebase_frameStartTime64 = this->uwbFrameStartTimes64[srcIndex];
////			return;
////		}
//
//		uint64 myFramelengthDuration_us = this->uwbListTDMAInfo[0].framelength*this->slotDuration_us;
//		uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbFrameStartTimes64[0], time_now_us);
//
//		uint64 diff_us = 0;
//		bool diff_add = FALSE;
//		uint64 threshold = this->slotStartDelay_us; //TODO make a permanent variable for this threshold
//
//		//TODO make sure this can't/doesn't happen
//		//myTimeSinceFrameStart is longer than the FramelengthDuration!!!
////		if(myTimeSinceFrameStart > myFramelengthDuration)
////		{
////			uint8 debug_msg[100];
////			int n = sprintf((char *)&debug_msg, "myTimeSinceFrameStart > myTimeSinceFrameStart!");//,%llX, psdu: %d ", instance_get_addr(), psduLength);
////			send_usbmessage(&debug_msg[0], n);
////			usb_run();
////		}
//
//		if(this->uwbListTDMAInfo[0].framelength <= framelength)
//		{
//			uint64 hisTimeSinceFrameStartMod_us = hisTimeSinceFrameStart_us%myFramelengthDuration_us;
//
//			if(myTimeSinceFrameStart_us > hisTimeSinceFrameStartMod_us)
//			{
//				diff_us = myTimeSinceFrameStart_us - hisTimeSinceFrameStartMod_us;
//				diff_add = TRUE;
//
////				uint64 diff_us = myTimeSinceFrameStart_us - hisTimeSinceFrameStartMod_us;
////				this->uwbFrameStartTimes64[0] = timestamp_add64(this->uwbFrameStartTimes64[0], diff_us/2);
////				this->lastSlotStartTime64 = timestamp_add64(this->lastSlotStartTime64, diff_us/2);
//			}
//			else
//			{
//				diff_us = hisTimeSinceFrameStartMod_us - myTimeSinceFrameStart_us;
//				diff_add = FALSE;
////				uint64 diff_us = hisTimeSinceFrameStartMod_us - myTimeSinceFrameStart_us;
////				this->uwbFrameStartTimes64[0] = timestamp_subtract64(this->uwbFrameStartTimes64[0], diff_us/2);
////				this->lastSlotStartTime64 = timestamp_subtract64(this->lastSlotStartTime64, diff_us/2);
//			}
//		}
//		else
//		{
//			uint64 hisFramelengthDuration_us = framelength*this->slotDuration_us;
//			uint64 myTimeSinceFrameStartMod_us = myTimeSinceFrameStart_us%hisFramelengthDuration_us;
//
//
////			if(timeSinceFrameStart > hisFramelengthDuration)
////			{
////				uint8 debug_msg[100];
////				int n = sprintf((char *)&debug_msg, "timeSinceFrameStart > hisFramelengthDuration!");
////				send_usbmessage(&debug_msg[0], n);
////				usb_run();
////			}
//
//			if(hisTimeSinceFrameStart_us > myTimeSinceFrameStartMod_us)
//			{
//				diff_us = hisTimeSinceFrameStart_us - myTimeSinceFrameStartMod_us;
//				diff_add = FALSE;
//
////				uint64 diff_us = hisTimeSinceFrameStart_us - myTimeSinceFrameStartMod_us;
////				this->uwbFrameStartTimes64[0] = timestamp_subtract64(this->uwbFrameStartTimes64[0], diff_us/2);
////				this->lastSlotStartTime64 = timestamp_subtract64(this->lastSlotStartTime64, diff_us/2);
//			}
//			else
//			{
//				diff_us = myTimeSinceFrameStartMod_us - hisTimeSinceFrameStart_us;
//				diff_add = TRUE;
//
////				uint64 diff_us = myTimeSinceFrameStartMod_us - hisTimeSinceFrameStart_us;
////				this->uwbFrameStartTimes64[0] = timestamp_add64(this->uwbFrameStartTimes64[0], diff_us/2);
////				this->lastSlotStartTime64 = timestamp_add64(this->lastSlotStartTime64, diff_us/2);
//			}
//		}
//
//
//		//if the difference is too large, we need to rebase our frame start time
////		if(diff_us > threshold)
////		{
////			this->rebase_pending = TRUE;
////			this->rebase_tx = FALSE;
////			this->rebase_frameStartTime64 = this->uwbFrameStartTimes64[srcIndex];
////		}
////		else
////		{
//			if(diff_add == TRUE)
//			{
//				this->uwbFrameStartTimes64[0] = timestamp_add64(this->uwbFrameStartTimes64[0], diff_us/2);
//				this->lastSlotStartTime64 = timestamp_add64(this->lastSlotStartTime64, diff_us/2);
//			}
//			else
//			{
//				this->uwbFrameStartTimes64[0] = timestamp_subtract64(this->uwbFrameStartTimes64[0], diff_us/2);
//				this->lastSlotStartTime64 = timestamp_subtract64(this->lastSlotStartTime64, diff_us/2);
//			}
////		}
//	}
//
//}


static bool tx_select(struct TDMAHandler *this)
{
//	uint8 debug_msg[100];
//	int n = sprintf((char *)&debug_msg, "tx_select,");
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

	instance_data_t *inst = instance_get_local_structure_ptr(0);

	uint32 time_now = portGetTickCnt(); //TODO come back and perhaps use only one or the other
	uint32 time_now_us = portGetTickCntMicro(); //TODO see time_now

	//DISCOVERY pauses for BLINK_DELAY <-added
//	if(this->waitForInf == TRUE || this->waitForRngInit == TRUE)
//	{
//		return FALSE;
//	}

	int uwb_index = 255;

	if(inst->mode == DISCOVERY)
	{
		if(this->discovery_mode == WAIT_INF_REG)
		{
			//start blinking if enough time has passed since entering DISCOVERY mode
			uint32 timeSinceDiscoverStart = get_dt32(this->discoveryStartTime, time_now);

//			uint8 debug_msg[100];
//			int n = sprintf((char *)&debug_msg, "discovery wait inf reg");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

	//		uint8 debug_msg[100];
	//		int n = sprintf((char *)&debug_msg, "timeSinceDiscoverStart %lu, maxFrameDuration %lu ", timeSinceDiscoverStart, maxFrameDuration);
	//		send_usbmessage(&debug_msg[0], n);
	//		usb_run();

			if(timeSinceDiscoverStart > this->waitInfDuration)
			{
				//enforce blink period
				uint32 timeSinceLastBlink = get_dt32(this->last_blink_time, time_now);

	//			uint8 debug_msg[100];
	//			int n = sprintf((char *)&debug_msg, "timeSinceLastBlink %lu, BLINK_PERIOD_MS %lu ", timeSinceLastBlink, (uint32)BLINK_PERIOD_MS);
	//			send_usbmessage(&debug_msg[0], n);
	//			usb_run();

				if(timeSinceLastBlink  > (uint32)BLINK_PERIOD_MS + (uint32)(rand() % 100))
				{
					//time to blink
					uwb_index = 255;
					this->set_discovery_mode(this, WAIT_RNG_INIT, time_now);
				}
				else
				{
					//not time to blink yet, keep waiting for RNG_INIT
					inst->wait4ack = 0;
					inst->testAppState = TA_RXE_WAIT;
					return TRUE;
				}
			}
			else
			{
				//shouldn't be in this mode, should be listening for INF messages
				inst->wait4ack = 0;
				inst->testAppState = TA_RXE_WAIT;
				return TRUE;
			}
		}
		else if(this->discovery_mode == SEND_SUG)
		{
//			uint8 debug_msg[100];
//			int n = sprintf((char *)&debug_msg, "discovery send sug");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			//get time since slot start and make sure that's greater than delay
			uint64 timeSinceSlotStart = get_dt64(this->lastSlotStartTime64, time_now_us);

			//make sure that we are in slot 0

			if(timeSinceSlotStart <= this->slotStartDelay_us)
			{
				uwb_index = -1;
			}
			else
			{
				//TODO figure out how to make sure we send our SUG packet at the right time...
				inst->wait4ack = 0;
				inst->testAppState = TA_TXINF_WAIT_SEND;
				inst->uwbToRangeWith = (uint8)255;
				return TRUE;
			}
		}
		else
		{
			return FALSE;
		}
	}
	else if(inst->mode == TAG)
	{
		//get time since slot start and make sure that's greater than delay
		uint64 timeSinceSlotStart = get_dt64(this->lastSlotStartTime64, time_now_us);

		//TAG pauses for INF_POLL_DELAY <-added at beginning of slot
		if(timeSinceSlotStart <= this->slotStartDelay_us)
		{
			uwb_index = -1;
		}
		else
		{
			if(this->firstPollSentThisSlot == TRUE && this->secondPollSentThisSlot == FALSE && this->firstPollComplete == FALSE)
			{
				int test = 1;
				test++;
			}

			//check which neighbor UWB hasn't been ranged with for the longest amount of time
			uint32 timeSinceOldestRange = 0;
			uint32 timeSinceSecondOldestRange = 0;
			uint8 second_oldest_uwb_index = 255;
			for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
			{
				if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR)
				{
					uint32 timeSinceRange = get_dt32(this->uwbListTDMAInfo[i].lastRange, time_now);

					if(timeSinceOldestRange == 0)
					{
						timeSinceOldestRange = timeSinceRange;
						timeSinceSecondOldestRange = timeSinceRange;

						uwb_index = i;
						second_oldest_uwb_index = i;
					}
					else if(timeSinceRange > timeSinceOldestRange)
					{
						timeSinceSecondOldestRange = timeSinceOldestRange;
						second_oldest_uwb_index = uwb_index;
						timeSinceOldestRange = timeSinceRange;
						uwb_index = i;
					}
					else if(timeSinceRange > timeSinceSecondOldestRange)
					{
						timeSinceSecondOldestRange = timeSinceRange;
						second_oldest_uwb_index = i;
					}
					else if(timeSinceOldestRange == timeSinceSecondOldestRange)
					{
						timeSinceSecondOldestRange = timeSinceRange;
						second_oldest_uwb_index = i;
					}
				}
			}

			if(this->firstPollSentThisSlot == TRUE && this->secondPollSentThisSlot == FALSE && this->firstPollComplete == FALSE)
			{
				uwb_index = second_oldest_uwb_index;
			}

			if(uwb_index == 255 && inst->uwbListLen > 1)
			{
				uwb_index = 1;
			}
		}
	}
	else
	{
		//ANCHOR shouldn't be in this mode, should be listening for INF and POLL
		inst->testAppState = TA_RXE_WAIT;
		return TRUE;
	}

//	debug_msg[100];
//	n = sprintf((char *)&debug_msg, "selected index %i ", uwb_index);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();


	if(uwb_index < 1) //set to -1 when waiting for the slotStartDelay to pass
	{
		//do nothing
		return FALSE;
	}
	else if(uwb_index > 254)
	{
		inst->testAppState = TA_TXBLINK_WAIT_SEND;
		inst->uwbToRangeWith = (uint8)255;
	}
	else
	{
		uint8 fcode = RTLS_DEMO_MSG_INF_INIT;
		if(memcmp(&inst->inf_msg.messageData[FCODE], &fcode, sizeof(uint8)) == 0)
		{
			this->infSentThisSlot = TRUE;
			inst->testAppState = TA_TXINF_WAIT_SEND;
		}
		else if(this->firstPollSentThisSlot == FALSE)
		{
			this->firstPollSentThisSlot = TRUE;
			inst->testAppState = TA_TXPOLL_WAIT_SEND;
		}
		else if(this->secondPollSentThisSlot == FALSE && this->firstPollComplete == FALSE)
		{
			this->secondPollSentThisSlot = TRUE;
			inst->testAppState = TA_TXPOLL_WAIT_SEND; //select new uwb index!
		}
		else if(this->infSentThisSlot == FALSE)
		{
			this->infSentThisSlot = TRUE;
			inst->testAppState = TA_TXINF_WAIT_SEND;
		}
		else
		{
			return TRUE;
		}

		inst->uwbToRangeWith = (uint8)uwb_index;
	}

	return TRUE;
}


static bool check_blink(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	bool retval = FALSE;

	if(inst->mode == DISCOVERY && this->discovery_mode == WAIT_INF_REG)
	{
		uint32 time_now = portGetTickCnt();
		uint32 timeSinceDiscoveryStart = get_dt32(this->discoveryStartTime, time_now);
		if(timeSinceDiscoveryStart > this->maxFramelength*this->slotDuration_ms )
		{
//			uint8 debug_msg[100];
//			int n = sprintf((char *)&debug_msg, "in RX_WAIT_DATA, portGetTickCnt(): %lu, inst->last_blink_time: %lu, BLINK_PERIOD_MS: %lu", portGetTickCnt(), inst->last_blink_time, (uint32)BLINK_PERIOD_MS);
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			uint32 timeSinceBlink = get_dt32(this->last_blink_time, time_now);
			if(timeSinceBlink > (uint32)BLINK_PERIOD_MS + (uint32)(rand()%100)){
//				uint8 debug_msg[100];
//				int n = sprintf((char *)&debug_msg, "in RX_WAIT_DATA, siwtch to TA_TX_SELECT");
//				send_usbmessage(&debug_msg[0], n);
//				usb_run();

//				this->discovery_mode = WAIT_RNG_INIT;
				retval = TRUE;
			}
		}
	}

	return retval;
}

static void populate_inf_msg(struct TDMAHandler *this, uint8 inf_msg_type)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	int num_neighbors = instfindnumneighbors(inst);
	int num_hidden = instfindnumhidden(inst);
//	uint32 time_now = portGetTickCnt();

	//fcode
	int msgDataIndex = FCODE;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &inf_msg_type, sizeof(uint8));


	//time since frame start
	//populated directly before being sent
//	msgDataIndex = TDMA_TSFS;
//	uint32 timeSinceFrameStart = get_dt32(this->uwbFrameStartTimes[0], time_now); //TODO handle number wrapping
//	memcpy(&inst->inf_msg.messageData[msgDataIndex], &timeSinceFrameStart, sizeof(uint32));


//	uint8 debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "TX MSG_INF timeSinceFrameStart: %lu", timeSinceFrameStart);
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();

	//number of neighbors
	msgDataIndex = TDMA_NUMN;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &num_neighbors, sizeof(uint8));

	//number of hidden neighbors
	msgDataIndex = TDMA_NUMH;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &num_hidden, sizeof(uint8));


	//self framelength
	msgDataIndex = TDMA_FRAMELENGTH;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->uwbListTDMAInfo[0].framelength, sizeof(uint8));

	//self number of slots
	msgDataIndex = TDMA_NUMS;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->uwbListTDMAInfo[0].slotsLength, sizeof(uint8));
	msgDataIndex++;

	//self slot assignments
	for(int s = 0; s < this->uwbListTDMAInfo[0].slotsLength; s++)
	{
		memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->uwbListTDMAInfo[0].slots[s], sizeof(uint8));
		msgDataIndex++;
	}

	//neighbor address, framelength, number of slots, and slot assignments
	for(int i = 1; i < inst->uwbListLen; i++) //slot 0 reserved for self
	{
//		if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR)
		if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR)
		{
			struct TDMAInfo *info = &this->uwbListTDMAInfo[i];

			//address
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &inst->uwbList[i][0], inst->addrByteSize);
			msgDataIndex += inst->addrByteSize;

			//framelength
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->framelength, sizeof(uint8));
			msgDataIndex++;

			//number of slots
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->slotsLength, sizeof(uint8));
			msgDataIndex++;

			//slot assignments
			for(int s = 0; s < info->slotsLength; s++)
			{
				memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->slots[s], sizeof(uint8));
				msgDataIndex++;
			}
		}
	}

	//hidden address, framelength, number of slots, and slot assignments
	for(int i = 1; i < inst->uwbListLen; i++) //slot 0 reserved for self
	{
//		if(inst->uwbListType[i] == UWB_LIST_HIDDEN)
		if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_HIDDEN)
		{
			struct TDMAInfo *info = &this->uwbListTDMAInfo[i];

			//address
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &inst->uwbList[i][0], inst->addrByteSize);
			msgDataIndex += inst->addrByteSize;

			//framelength
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->framelength, sizeof(uint8));
			msgDataIndex++;

			//number of slots
			memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->slotsLength, sizeof(uint8));
			msgDataIndex++;

			//slot assignments
			for(int s = 0; s < info->slotsLength; s++)
			{
				memcpy(&inst->inf_msg.messageData[msgDataIndex], &info->slots[s], sizeof(uint8));
				msgDataIndex++;
			}
		}
	}


#if (USING_64BIT_ADDR==1)
			this->infMessageLength = msgDataIndex + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
#else
			this->infMessageLength = msgDataIndex + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
#endif

}


static void update_inf_tsfs(struct TDMAHandler *this)
{
	//TODO check if tsfs is greater than frameDuration, update if so

	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint64 time_now_us = portGetTickCntMicro();
	int msgDataIndex = TDMA_TSFS;
	uint64 timeSinceFrameStart64 = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
	uint64 frameDuration = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;

	//if frameStartTime ahead of time_now_us, just send the erroneously large number over.
	//the receiver will detect and disregard
	if(timeSinceFrameStart64 < 10000000)
	{
		while(timeSinceFrameStart64 > frameDuration)
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, frameDuration);
			timeSinceFrameStart64 -= frameDuration;
		}
	}
	else
	{
		timeSinceFrameStart64 = 0;
	}

	memcpy(&inst->inf_msg.messageData[msgDataIndex], &timeSinceFrameStart64, 6);

}


//Procedure for processing INF SUG, INF REG, and INF UPDATE
//1. Check for differences with locally stored TDMA assignment information
//		(a) exit if none exist
//2. Drop all slot assignments for self, neighbor, hidden, and twice hidden nodes that appear
//	 in the INF message
//3. Copy all assignments for self, neighbors, hidden, and twice hidden nodes that appear
//	 in the INF message
//4. Check for conflicts with slot assignments for nodes not contained in the INF message.
//		(a) if conflicts exist and self node is one of the conflicts, release all slot assignments
//			from self and follow the procedure for a new node (Section 1.1), skipping the
//			collect INF REG step.
//		(b) if conflicts exist and node is not one of the conflicts, deconflict according to 1.2
//5. Send INF UPDATE message at beginning of allocated slot (handled elsewhere)

//process types... 1.) clear all and copy 2.) clear mentioned, copy 3.) copy
//returns TRUE if a change was made to the TDMA assingments, FALSE if invalid message FCODE or process mode or if no TDMA changes made
static bool process_inf_msg(struct TDMAHandler *this, uint8 *messageData, uint8 srcIndex, INF_PROCESS_MODE mode)
{
	//NOTE: this function does not handle TDMA deconflict

//	uint8 debug_msg[100];
//	int n = sprintf((char*)&debug_msg[0], "process INF");
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();


	bool tdma_modified = FALSE;

	uint32 time_now = portGetTickCnt();

	if((mode != CLEAR_ALL_COPY)     && //happens when we creat a new network
	   (mode != CLEAR_LISTED_COPY)	&& //happens most of the time while processing
	   (mode != COPY))				   //happens when collecting inf messages
	{
		//only process if valid mode supplied
		return tdma_modified;
	}

	bool safeAssign = FALSE;
	if(mode == COPY)
	{
		safeAssign = TRUE;
	}


	uint8 inf_msg_type;
	memcpy(&inf_msg_type, &messageData[FCODE], sizeof(uint8));

	if((inf_msg_type != RTLS_DEMO_MSG_INF_REG)	  &&
	   (inf_msg_type != RTLS_DEMO_MSG_INF_UPDATE) &&
	   (inf_msg_type != RTLS_DEMO_MSG_INF_INIT)   &&
	   (inf_msg_type != RTLS_DEMO_MSG_INF_SUG))
	{
		//only process INF messages
		return FALSE;
	}

	instance_data_t *inst = instance_get_local_structure_ptr(0);
	this->uwbListTDMAInfo[srcIndex].connectionType = UWB_LIST_NEIGHBOR;

	uint8 numNeighbors;
	uint8 numHidden;
	uint8 framelength;
	uint8 numSlots;
	uint8 slot;
	struct TDMAInfo *info;

	memcpy(&numNeighbors, &messageData[TDMA_NUMN], sizeof(uint8));
	memcpy(&numHidden, &messageData[TDMA_NUMH], sizeof(uint8));
	memcpy(&framelength, &messageData[TDMA_FRAMELENGTH], sizeof(uint8));
	memcpy(&numSlots, &messageData[TDMA_NUMS], sizeof(uint8));

	int msgDataIndex = TDMA_NUMS + 1;

	//TODO have deconflict mode???
	bool uwbListInMsg[UWB_LIST_SIZE];
	for(int i = 0; i < inst->uwbListLen; i++)
	{
		uwbListInMsg[i] = FALSE;
	}
	uwbListInMsg[srcIndex] = TRUE;

	if(mode == CLEAR_ALL_COPY)
	{
		//clear all TDMA assignments and reset framelength to MIN
		this->tdma_free_all_slots(this);
		tdma_modified = TRUE;
	}

	//copy slot assignments for source UWB
	info = &this->uwbListTDMAInfo[srcIndex];
	if(framelength != info->framelength)
	{
		tdma_modified = TRUE;
	}

	//check if the tdma has been modified
	if(tdma_modified == FALSE) //dont look for any more differences if we already know one exists
	{
		//frist check if same number of slots
		if(numSlots == info->slotsLength)
		{
			//then check if each incoming slot is already assigned
			for(int i = 0; i < numSlots; i++)
			{
				memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
				msgDataIndex++;

				if(this->slot_assigned(info, slot) == FALSE)
				{
					tdma_modified = TRUE;
					break;
				}
			}
		}
		else
		{
			tdma_modified = TRUE;
		}
	}

	if(mode == CLEAR_LISTED_COPY)
	{
		this->free_slots(info); //do after cheking framelength because framelength will be reset
	}

	info->framelength = MAX(framelength, info->framelength);

	msgDataIndex = TDMA_NUMS + 1;
	for(int s = 0; s < numSlots; s++)
	{
		memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;

		this->assign_slot(info, slot, safeAssign);

//		if(this->assign_slot(info, slot) == TRUE)
//		{
//			tdma_modified = TRUE;  //TODO this is not right, it should somehow compare before and after...
//		}
	}


	for(int i = 0; i < numNeighbors; i++)
	{
		uint8 address[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		memcpy(&address[0], &messageData[msgDataIndex], inst->addrByteSize);
		msgDataIndex += inst->addrByteSize;

		uint8 uwb_index = instgetuwblistindex(inst, &address[0], inst->addrByteSize);
		if(uwb_index != 0)
		{
//			if(inst->uwbListType[uwb_index] == UWB_LIST_INACTIVE || inst->uwbListType[uwb_index] == UWB_LIST_TWICE_HIDDEN)
			if(this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_INACTIVE || this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_TWICE_HIDDEN)
			{
//				inst->uwbListType[uwb_index] = UWB_LIST_HIDDEN;
				this->uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_HIDDEN;
			}

//			inst->lastHiddenTimeStamp[uwb_index] = time_now;
			this->uwbListTDMAInfo[uwb_index].lastCommHidden = time_now;
		}

		info = &this->uwbListTDMAInfo[uwb_index];
		uwbListInMsg[uwb_index] = TRUE;


		memcpy(&framelength, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;
		memcpy(&numSlots, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;
		int msgDataIndexSave = msgDataIndex;

		//check if the tdma has been modified
		if(tdma_modified == FALSE) //dont look for any more differences if we already know one exists
		{
			//frist check if same framelength and number of slots
			if(framelength == info->framelength && numSlots == info->slotsLength)
			{
				//then check if each incoming slot is already assigned
				for(int s = 0; s < numSlots; s++)
				{
					memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
					msgDataIndex++;

					if(this->slot_assigned(info, slot) == FALSE)
					{
						tdma_modified = TRUE;
						break;
					}
				}
			}
			else
			{
				tdma_modified = TRUE;
			}
		}

		if(mode == CLEAR_LISTED_COPY)
		{
			this->free_slots(info); //do after checking framelength because framelength reset
		}
		info->framelength = MAX(framelength, info->framelength);


		msgDataIndex = msgDataIndexSave;
		for(int s = 0; s < numSlots; s++)
		{
			memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
			msgDataIndex++;

			this->assign_slot(info, slot, safeAssign);
//			if(this->assign_slot(info, slot) == TRUE)
//			{
//				tdma_modified = TRUE; //TODO also not right! need to check modification
//			}
		}
	}

	for(int i = 0; i < numHidden; i++)
	{
		uint8 address[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		memcpy(&address[0], &messageData[msgDataIndex], inst->addrByteSize);
		msgDataIndex += inst->addrByteSize;

		uint8 uwb_index = instgetuwblistindex(inst, &address[0], inst->addrByteSize);
		if(uwb_index != 0)//0 reserved for self
		{
//			if(inst->uwbListType[uwb_index] == UWB_LIST_INACTIVE)
			if(this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_INACTIVE)
			{
//				inst->uwbListType[uwb_index] = UWB_LIST_TWICE_HIDDEN;
				this->uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_TWICE_HIDDEN;
			}

//			inst->lastTwiceHiddenTimeStamp[uwb_index] = time_now;
			this->uwbListTDMAInfo[uwb_index].lastCommTwiceHidden = time_now;
		}

		uwbListInMsg[uwb_index] = TRUE;
		info = &this->uwbListTDMAInfo[uwb_index];

		memcpy(&framelength, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;
		memcpy(&numSlots, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;
		int msgDataIndexSave = msgDataIndex;

		//check if the tdma has been modified
		if(tdma_modified == FALSE) //dont look for any more differences if we already know one exists
		{
			//frist check if same framelength and number of slots
			if(framelength == info->framelength && numSlots == info->slotsLength)
			{
				//then check if each incoming slot is already assigned
				for(int s = 0; s < numSlots; s++)
				{
					memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
					msgDataIndex++;

					if(this->slot_assigned(info, slot) == FALSE)
					{
						tdma_modified = TRUE;
						break;
					}
				}
			}
			else
			{
				tdma_modified = TRUE;
			}
		}

		if(mode == CLEAR_LISTED_COPY)
		{
			this->free_slots(info); //do after checking for difference because will reset framelength as well
		}
		info->framelength = MAX(framelength, info->framelength);

		msgDataIndex = msgDataIndexSave;
//		int msgDataIndex = msgDataIndexSave;
		for(int s = 0; s < numSlots; s++)
		{
			memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
			msgDataIndex++;

			this->assign_slot(info, slot, safeAssign);
//			if(this->assign_slot(info, slot) == TRUE) //the only problem i see with this is that it does not give me a good way to isolate which ones were or weren't modified, this is a problem for deconflict logic...
//			{
//				tdma_modified = TRUE;
//			}
		}
	}


	//handle deconflict???
	if(mode == CLEAR_LISTED_COPY)
	{
		//deconflict uncopied against copied. (excluding self)
		for(int i = 1; i < inst->uwbListLen; i++)
		{
			for(int j = i + 1; j < inst->uwbListLen; j++)
			{
				if((uwbListInMsg[i] == FALSE && uwbListInMsg[j] == TRUE) || (uwbListInMsg[i] == TRUE && uwbListInMsg[j] == FALSE))
				{

//					if((inst->uwbListType[i] == UWB_LIST_NEIGHBOR && inst->uwbListType[j] == UWB_LIST_NEIGHBOR) ||
//					   (inst->uwbListType[i] == UWB_LIST_NEIGHBOR && inst->uwbListType[j] == UWB_LIST_HIDDEN)   ||
//					   (inst->uwbListType[j] == UWB_LIST_NEIGHBOR && inst->uwbListType[i] == UWB_LIST_NEIGHBOR) ||
//					   (inst->uwbListType[j] == UWB_LIST_NEIGHBOR && inst->uwbListType[i] == UWB_LIST_HIDDEN))
					if((this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR) ||
					   (this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_HIDDEN)   ||
					   (this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR) ||
					   (this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[i].connectionType == UWB_LIST_HIDDEN))
					{
						//TODO make sure this is okay. will i need to ensure the assignments from the message are maintained?
						if(this->deconflict_uwb_pair(this, &this->uwbListTDMAInfo[i], &this->uwbListTDMAInfo[j]) == TRUE)
						{
							tdma_modified = TRUE;
						}
					}
				}
			}
		}

		//check if self has any conflicts
		if(this->self_conflict(this))
		{
			//if so, release all assignments from self
			this->free_slots(&this->uwbListTDMAInfo[0]);

			//find self a new slot assignment
			this->find_assign_slot(this);

			tdma_modified = TRUE;
		}
	}

//	uint32 deltat = get_dt32(time_now, portGetTickCnt());

//	if(deltat > 1)
//	{
//		uint8 debug_msg[100];
//		int n = sprintf((char *)&debug_msg, "process_inf_time:%lu,%04X,xxxx", deltat, inst->uwbShortAdd);
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();
//	}

	return tdma_modified;
}


static bool check_tdma_diff(struct TDMAHandler *this, uint8 *messageData, uint8 *srcIndex)
{
	return TRUE;

//	struct TDMAInfo info = &this->uwbListTDMAInfo[srcIndex];
//
//	if(info->framelength != framelength || info->slotsLength != numSlots)
//	{
//		return TRUE;
//	}
//
//	for(int s = 0; s < numSlots; s++)
//	{
//		memcpy(&slot, &messageData[msgDataIndex], 1);
//		msgDataIndex++;
//
//		if(this->slot_assigned(info, slot) == FALSE)
//		{
//			return TRUE;
//		}
//	}
//
//	for(int i = 0; i < numNeighbors; i++)
//	{
//		memcpy(&address, &messageData[msgDataIndex], inst->addrByteSize);
//		msgDataIndex += inst->addrByteSize;
//
//		uint8 uwb_index = instgetuwblistindex(inst, &address[0], inst->addrByteSize);
//		info = &this->uwbListTDMAInfo[uwb_index];
//
//		memcpy(&framelength, &messageData[msgDataIndex], 1);
//		msgDataIndex++;
//
//		memcpy(&numSlots, &messageData[msgDataIndex], 1);
//		msgDataIndex++;
//
//		if(info->framelength != framelength || info->slotsLength != numSlots)
//		{
//			return TRUE;
//		}
//
//		for(int s = 0; s < numSlots; s++)
//		{
//			memcpy(&slot, &messageData[msgDataIndex], 1);
//			msgDataIndex++;
//
//			if(this->slot_assigned(info, slot) == FALSE)
//			{
//				return TRUE;
//			}
//		}
//	}
//
//	for(int i = 0; i < numHidden; i++)
//	{
//		memcpy(&address, &messageData[msgDataIndex], inst->addrByteSize);
//		msgDataIndex += inst->addrByteSize;
//
//		uint8 uwb_index = instgetuwblistindex(inst, &address[0], inst->addrByteSize);
//
//		info = &this->uwbListTDMAInfo[uwb_index];
//
//		memcpy(&framelength, &messageData[msgDataIndex], 1);
//		msgDataIndex++;
//
//		memcpy(&numSlots, &messageData[msgDataIndex], 1);
//		msgDataIndex++;
//
//		if(info->framelength != framelength || info->slotsLength != numSlots)
//		{
//			return TRUE;
//		}
//
//		for(int s = 0; s < numSlots; s++)
//		{
//			memcpy(&slot, &messageData[msgDataIndex], 1);
//			msgDataIndex++;
//
//			if(this->slot_assigned(info, slot) == FALSE)
//			{
//				return TRUE;
//			}
//		}
//	}
//
//
//	return FALSE;
}



//TODO add conflict detection/resolution!!!
//static void populate_sug_msg(struct TDMAHandler *this)
//{
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//
//	//if I'm here, i should clear my own assignment information
//	if(this->mySlots != NULL && this->mySlotsLength != 0)
//	{
//		this->mySlotsLength = 0;
//		free(this->mySlots);
//		this->mySlots = NULL;
//	}
//	uint8 unassigned = 255;
//	for(int i = 0; i < this->maxFramelength; i++)
//	{
//		memcpy(&this->slotAssignments[i], &unassigned, sizeof(uint8));
//	}
//	this->framelength = MIN_FRAMELENGTH;
//
//
//
//
//	inst->inf_msg.messageData[FCODE] = RTLS_DEMO_MSG_INF_SUG;
//	int msgDataIndex = FCODE + 1;
//
//	int num_neighbors = instfindnumactiveneighbors(inst);
//	uint32 time_now = portGetTickCnt();
//
//
//	//TODO build framelength after creating slot assignments...
////	//framelength
////	memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->framelength, 1);
//	msgDataIndex++;
//
//	uint32 timeSinceFrameStart = get_dt32(this->frameStartTime, time_now); //TODO handle number wrapping
//	memcpy(&inst->inf_msg.messageData[msgDataIndex], &timeSinceFrameStart, sizeof(uint32));
//	msgDataIndex += sizeof(timeSinceFrameStart);
//
////	uint8 debug_msg[100];
////	 int n = sprintf((char*)&debug_msg[0], "TX MSG_INF timeSinceFrameStart: %lu", timeSinceFrameStart);
////	 send_usbmessage(&debug_msg[0], n);
////	 usb_run();
//
//
//	//number of neighbors
//	memcpy(&inst->inf_msg.messageData[msgDataIndex], &num_neighbors, 1);
//	msgDataIndex++;
//
//	//neighbor addresses
//	for(int i = 0; i < inst->uwbListLen; i++)
//	{
//		if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR)
//		{
//			memcpy(&inst->inf_msg.messageData[msgDataIndex], &inst->uwbList[i][0], inst->addrByteSize);
//			msgDataIndex += inst->addrByteSize;
//		}
//	}
//
//	//neighbor framelength
//	for(int i = 0; i < inst->uwbListLen; i++)
//	{
//		if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR)
//		{
//			memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->uwbFramelengths[i], 1);
//			msgDataIndex++;
//		}
//	}
//
//	//addresses in each TDMA slot
////	for(int i = 0; i < this->framelength; i++)
////	{
////		memcpy(&inst->inf_msg.messageData[msgDataIndex], &this->slotAssignments[i], 2);
////
////		msgDataIndex += 2;
////	}
//
//	//TODO check for conflicts at some point
//
//	//use the slotAssignements array to combine the collected information
//	//note: try to keep framelength as small as possible
//	//note: if we have to increase the framelength, start over!
//
////	uint8 mfl = this->framelength;
////
////	uint8 debug_msg[100];
////	 int n = sprintf((char*)&debug_msg[0], "this->framelength: %u", this->framelength);
////	 send_usbmessage(&debug_msg[0], n);
////	 usb_run();
//
//	//getting stuck in here... somehow this->framelength is getting set to 0
//	for(int i = 0; i < inst->uwbListLen; i++)
//	{
//		if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
//		{
//			uint8 framelength_i = this->uwbFramelengths[i];
//			for(int j = 0; j < this->uwbListSlotsLengths[i]; j++)
//			{
//				//get slot
//				uint8 slot_j;
//				bool double_frame = FALSE;
//				memcpy(&slot_j, &this->uwbListSlots[i][j], 1);
//
//
//				//check if slot is taken
//				if(slot_j >= this->framelength)
//				{
//					uint8 mod_slot = slot_j%this->framelength;
//					if(mod_slot == 0)
//					{
//						//slots not allowed to be assigned to the zeroth slot
//						double_frame = TRUE;
//					}
//					else
//					{
////						if(memcmp(&this->slotAssignments[mod_slot], &zero, sizeof(uint8)) == 0)
//						if(memcmp(&this->slotAssignments[mod_slot], &unassigned, sizeof(uint8)) == 0)
//						{
//							//slot not assigned
//							memcpy(&this->slotAssignments[mod_slot], &i, sizeof(uint8));
//						}
//						else if(memcmp(&this->slotAssignments[mod_slot], &i, sizeof(uint8)) != 0)
//						{
//							//already assigned to another UWB,
//							//double the frame and start over!(?) should I instead consider framelengths and such?
//							double_frame = TRUE;
//						}
//					}
//
//				}
//				else if(slot_j < this->framelength)
//				{
//					while(slot_j < this->framelength)
//					{
//						if(memcmp(&this->slotAssignments[slot_j], &unassigned, sizeof(uint8)) == 0)
//						{
//							//slot not assigned
//							memcpy(&this->slotAssignments[slot_j], &i, sizeof(uint8));
//						}
//						else if(memcmp(&this->slotAssignments[slot_j], &i, sizeof(uint8)) != 0)
//						{
//							//already assigned to another UWB,
//							//double the frame and start over!(?) should I instead consider framelengths and such?
//							double_frame = TRUE;
//							break;
//						}
//						slot_j += framelength_i;
//					}
//				}
//
//				if(double_frame == TRUE)
//				{
//					this->framelength *= 2;
//					i = 0;
//					j = 0;
//				}
//
//	//			uint16 uwbShortAdd = address64to16(&inst->uwbList[i][0]);
//	//			uint8 index = msgDataIndex + 2*this->uwbListSlots[i][j];
//	//			memcpy(&inst->inf_msg.messageData[msgDataIndex + (int)(inst->addrByteSize*this->uwbListSlots[i][j])], &uwbShortAdd, 2);
//			}
//		}
//
//	}
//
//	//if we are here, then we should've successfully combined the INF message information
//
//
//
//	bool assigned = FALSE;
//	while(assigned == FALSE)
//	{
//		//first try GU (Get Unassigned) slot
//		for(int i = 1; i < this->framelength; i++)//start at i = 1 because 0 is a reserved slot
//		{
////			if(memcmp(&this->slotAssignments[i], &zero, sizeof(uint8)) == 0)
//			if(memcmp(&this->slotAssignments[i], &unassigned, sizeof(uint8)) == 0)
//			{
//				//empty slot, assign
//				this->assign_slot(&this->myTDMAInfo, i);
//				assigned = TRUE;
//			}
//		}
//
//		//no open slots for GU
//		//next try RMA (Release Multiple Assigned) slot
//		if(assigned == FALSE)
//		{
//			uint8 most_slots = 1;
//			uint8 most_slots_index = 255;
//			uint8 release_slot = 255;
//			uint8 assign_slot = 255;
//			for(int i = 0; i < inst->uwbListLen; i++)
//			{
//				//TODO do I only need to consider neighbors or also hidden?
//				if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
//				{
//					uint8 uwbSlotsLen;
//					memcpy(&uwbSlotsLen, &this->uwbListSlotsLengths[i], sizeof(uint8));
//					if(uwbSlotsLen > most_slots)
//					{
//						bool slot_found = FALSE;
//						//check if the slot to be released can be assigned within our framelength
//						for(int j = 0; j < uwbSlotsLen; j++)
//						{
//							uint8 slot;
//							memcpy(&slot, &this->uwbListSlots[i][j], sizeof(uint8));
//							uint8 mod_slot = slot%this->framelength;
//
//							if(mod_slot != 0 && memcmp(&this->slotAssignments[mod_slot], &i, sizeof(uint8)) == 0)
//							{
//								release_slot = slot;
//								assign_slot = mod_slot;
//								slot_found = TRUE;
//								break;
//							}
//						}
//
//						if(slot_found == TRUE)
//						{
//							most_slots =  uwbSlotsLen;
//							most_slots_index = i;
//						}
//					}
//				}
//			}
//
//			if(most_slots_index != 255) //what about self? 254?
//			{
//				this->assign_slot(&this->myTDMAInfo, assign_slot);
//				this->uwblist_free_slot(this, most_slots_index, release_slot);
//				memcpy(&this->slotAssignments[assign_slot], &this->slotAssingmentSelfIndex, sizeof(uint8));
//
//				assigned = TRUE;
//			}
//		}
//
//		//no slots released via RMA
//		//DF (Double Frame), and assign via GU
//		if(assigned == FALSE)
//		{
//			memcpy(&this->slotAssignments[(int)this->framelength], &this->slotAssingmentSelfIndex, sizeof(uint8));
//
//			for(int i = 1; i < this->framelength; i++)
//			{
//				memcpy(&this->slotAssignments[i + (int)this->framelength], &this->slotAssignments[i], sizeof(uint8));
//			}
//			this->framelength *= 2;
//
//			assigned = TRUE;
//		}
//	}
//
//	for(int i = 0; i < this->framelength; i++)
//	{
//		//TODO handle the case where we are assigned to the slot
//		uint8 slot_index;
//		memcpy(&slot_index, &this->slotAssignments[i], sizeof(uint8));
//		memcpy(&inst->inf_msg.messageData[msgDataIndex], &inst->uwbList[slot_index], sizeof(inst->addrByteSize));
//		msgDataIndex += sizeof(inst->addrByteSize);
//	}
//
//	memcpy(&inst->inf_msg.messageData[FCODE + 1], &this->framelength, 1);
//
//	dwt_setrxtimeout((uint16)0);
////	this->usb_dump_tdma(this);
//
//
//}


//void process_sug_msg(struct TDMAHandler *this, uint8 *messageData, uint8 *srcAddr)
//{
//	//how to compare and integrate changes???
//	//first add any addresses to my list
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//
//	uint8 srcIndex = instgetuwblistindex(inst, &srcAddr[0], inst->addrByteSize);
//	inst->uwbListType[srcIndex] = UWB_LIST_NEIGHBOR;
//	uint8 neighborAddressN[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//	uint8 neighborFramelengthN;
//	uint8 slotAddress[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//	uint8 blankAddress[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//	uint32 timeSinceFrameStart;
//
////	if(tdma_handler->discovery_mode == WAIT_INF_REG)
////	{
////		tdma_handler->tdma_free_all_slots(tdma_handler);
////	}
//
//	int msgDataIndex = FCODE + 1;
//	memcpy(&tdma_handler->uwbFramelengths[srcIndex], &messageData[msgDataIndex], 1);
//	msgDataIndex++;
//
//	//TODO use range to help sync frame start time
//	memcpy(&timeSinceFrameStart, &messageData[msgDataIndex], sizeof(timeSinceFrameStart));
//	msgDataIndex += sizeof(timeSinceFrameStart);
//
//
//	memcpy(&srcNumNeighbors, &messageData[msgDataIndex], 1);
//	msgDataIndex++;
//
//	int fl_index_offset = (int)(inst->addrByteSize*srcNumNeighbors);
//	for(int n = 0; n < srcNumNeighbors; n++)
//	{
//		int framelength_index = msgDataIndex + fl_index_offset;
//		memcpy(&neighborAddressN, &messageData[msgDataIndex], inst->addrByteSize);
//		memcpy(&neighborFramelengthN, &messageData[framelength_index], sizeof(neighborFramelengthN));
//
//		msgDataIndex += (int)inst->addrByteSize;
//
//		uint8 uwb_index = instgetuwblistindex(inst, &neighborAddressN[0], inst->addrByteSize);
//		if(inst->uwbListType[uwb_index] != UWB_LIST_NEIGHBOR)
//		{
//			inst->uwbListType[uwb_index] = UWB_LIST_HIDDEN;
//		}
//
//		tdma_handler->uwbFramelengths[uwb_index] = neighborFramelengthN;
//	}
//	msgDataIndex += sizeof(uint8)*srcNumNeighbors;
//
//
//	//now process the slot assignments
//	for(int i = 0; i < tdma_handler->uwbFramelengths[srcIndex]-1; i++)//zeroth slot not included in INF
//	{
//		uint8 slot = i + 1;
//		memcpy(&slotAddress, &messageData[msgDataIndex], inst->addrByteSize);
//		//see who the address belongs to
//#if (USING_64BIT_ADDR==0)
//		if(memcmp(&inst->uwbShortAdd, &slotAddress, inst->addrByteSize) == 0)
//#else
//		if(memcmp(&inst->eui64[0], &slotAddress, inst->addrByteSize) == 0)
//#endif
//		{
//			//assignment is self
//		}
//		else
//		{
//			//assignment in UWB list if not unassigned
//			if(memcmp(&inst->eui64[0], &blankAddress, inst->addrByteSize) != 0)
//			{
//				//find in our list (or add as hidden)
//				uint8 uwb_index = instgetuwblistindex(inst, &slotAddress[0], inst->addrByteSize);
//				if(inst->uwbListType[uwb_index] != UWB_LIST_NEIGHBOR)
//				{
//					inst->uwbListType[uwb_index] = UWB_LIST_HIDDEN;
//				}
//
//				//what to do with the information???
//				//TODO left off here!!!
////				tdma_handler->uwblist_assign_slot(tdma_handler, uwb_index, slot);
//			}
//
//
//
//
//		}
//
//		msgDataIndex += (int)inst->addrByteSize;
//	}
//
//
//	//account for the delay between transmission and reception of the INF message
//	//TODO make this a function of the data rate as well!
//	//TODO experimentally found this number to be +- 1 millisecond for some reason. figure out why
//	//experimentally found formula is 0.079667x + 0.85611
//	uint32 txrx_delay = (uint32)(0.079667*(float)(dw_event->rxLength) + 0.85611); //TODO make this function of message length!
//
//
//	tdma_handler->uwbFrameStartTimes[srcIndex] = time_now - (timeSinceFrameStart + txrx_delay);
//
//
//	uint32 myFramelengthDuration = tdma_handler->framelength*tdma_handler->slotDuration;
//	uint32 myTimeSinceFrameStart = get_dt32(tdma_handler->frameStartTime, time_now);
//
//	//NEW
//	if(tdma_handler->framelength <= tdma_handler->uwbFramelengths[srcIndex])
//	{
//		uint32 timeSinceFrameStartMod = (timeSinceFrameStart + txrx_delay)%myFramelengthDuration;
//
//		if(myTimeSinceFrameStart > timeSinceFrameStartMod)
//		{
//			uint32 diff = myTimeSinceFrameStart - timeSinceFrameStartMod;
//			tdma_handler->frameStartTime += diff/2;
//			tdma_handler->lastSlotStartTime += diff/2;
//		}
//		else
//		{
//			uint32 diff = timeSinceFrameStartMod - myTimeSinceFrameStart;
//			tdma_handler->frameStartTime -= diff/2;
//			tdma_handler->lastSlotStartTime -= diff/2;
//
//		}
//	}
//	else
//	{
//		uint32 hisFramelengthDuration = tdma_handler->uwbFramelengths[srcIndex]*tdma_handler->slotDuration;
//		uint32 myTimeSinceFrameStartMod = (myTimeSinceFrameStart - txrx_delay)%hisFramelengthDuration;
//
//		if(timeSinceFrameStart > myTimeSinceFrameStartMod)
//		{
//			uint32 diff = timeSinceFrameStart - myTimeSinceFrameStartMod;
//			tdma_handler->frameStartTime -= diff/2;
//			tdma_handler->lastSlotStartTime -= diff/2;
//		}
//		else
//		{
//			uint32 diff = myTimeSinceFrameStartMod - timeSinceFrameStart;
//			tdma_handler->frameStartTime += diff/2;
//			tdma_handler->lastSlotStartTime += diff/2;
//		}
//	}
//
//
//
//
//}



static bool poll_delay(struct TDMAHandler *this, uint32 time_now_offset, uint32 offset)
{
	bool delay = FALSE;

	uint64 time_now_us = portGetTickCntMicro();
	uint64 timeSinceSlotStart = get_dt64(this->lastSlotStartTime64, time_now_us);
	if(timeSinceSlotStart >= this->slotStartDelay_us)
	{
		delay = FALSE;
	}

	return delay;
}

static bool slot_assigned(struct TDMAInfo *info, uint8 slot)
{
	bool assigned = FALSE;
	if(info->slots != NULL && info->slotsLength != 0)
	{
		for(int i = 0; i < info->slotsLength; i++)
		{
			if(memcmp(&info->slots[i], &slot, 1) == 0)
			{
				assigned = TRUE;
				break;
			}
		}
	}

	return assigned;
}

static bool assign_slot(struct TDMAInfo *info, uint8 slot, bool safeAssign)
{
	//NOTE: deconflicting happens elsewhere
	bool retval = FALSE;

	if(safeAssign == TRUE)//when using safeAssign, first check if the slot is assigned
	{
		//if not assigned, increase slots size and add slot index to end of array (array is unsorted)
		if(slot_assigned(info, slot) == FALSE)
		{
			uint8 *newSlots = malloc(sizeof(uint8)*(info->slotsLength + 1));
			memcpy(&newSlots[0], &info->slots[0], sizeof(uint8)*info->slotsLength);
			memcpy(&newSlots[info->slotsLength], &slot, 1);

			free(info->slots);
			info->slots = NULL;
			info->slots = newSlots;
			info->slotsLength += 1;

			retval = TRUE;
		}
	}
	else
	{
		uint8 *newSlots = malloc(sizeof(uint8)*(info->slotsLength + 1));
		memcpy(&newSlots[0], &info->slots[0], sizeof(uint8)*info->slotsLength);
		memcpy(&newSlots[info->slotsLength], &slot, 1);

		free(info->slots);
		info->slots = NULL;
		info->slots = newSlots;
		info->slotsLength += 1;

		retval = TRUE;
	}


	return retval;
}

//finding and assigning a slot works according to the following:
//1.) Set framelength to 4
//2.) Get Unassigned Slots (GU)
//		applicable if one or more open slots exist
//		assign self all unassigned slots (except for 0th slot); exit
//3.) Release Multiple Assigned Slots (RMA)
//		applicable if 2.) not applicable
//		applicable if one or more nodes has multiple slot assignments
//		release one slot from node with greatest number of slot assignments and assign to self; exit
//4.) Double the Frame (DF)
//		applicable if 2.) and 3.) not applicable
//		double own framelength and go back to 2.)

//procedure above doesn't seem to alwasy work, even for 3 UWBs... each assigned a slot out of four
//1,2,3 find no slots to assign and 4 wont change that... will just double the frame forever...
//actually it should work, because when we have double the framelength, the other uwbs virtually have two slots...
static void find_assign_slot(struct TDMAHandler *this)
{

	//NOTE: this assumes that all other TDMAInfo stored in the TDMAHandler are not in conflict with each other
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	struct TDMAInfo *info = &this->uwbListTDMAInfo[0];

	//if this UWB was somehow reset, recover its prior slot assignment info from the network traffic
	if(info->slotsLength > 0)
	{
		return;
	}

	bool assignment_made = FALSE;

	//set framelength
	info->framelength = 4;

	//come here after DF
	while(TRUE)
	{
		//GU
		for(uint8 i = 1; i < info->framelength; i++) //TODO make sure we don't accidentally assign to slot 0
		{
			bool assigned = FALSE;

			for(uint8 u = 1; u < inst->uwbListLen; u++)//0 reserved for self
			{
				for(uint8 su = 0; su < this->uwbListTDMAInfo[u].slotsLength; su++)
				{
					uint8 slot_su;
					memcpy(&slot_su, &this->uwbListTDMAInfo[u].slots[su], sizeof(uint8));

					if(info->framelength > this->uwbListTDMAInfo[u].framelength)
					{
						uint8 mod_i = i%this->uwbListTDMAInfo[u].framelength;

						if(slot_su == mod_i)
						{
							//slot assigned to this uwb
							assigned = TRUE;
						}
					}
					else if(info->framelength < this->uwbListTDMAInfo[u].framelength)
					{
						uint8 mod_slot_su = slot_su % info->framelength;

						if(mod_slot_su == i)
						{
							//slot assigned to this uwb
							assigned = TRUE;
						}
					}
					else //same framelength
					{
						if(slot_su == i)
						{
							//slot assigned to this uwb
							assigned = TRUE;
						}
					}


					if(assigned == TRUE)
					{
						break;
					}
				}

				if(assigned == TRUE)
				{
					break;
				}
			}

			//slot not assigned, assign to self
			if(assigned == FALSE)
			{
				this->assign_slot(info, i, FALSE);
				assignment_made = TRUE; //TODO just stop here? or get all unassigned slots?
			}
		}

		if(assignment_made == TRUE)
		{
			break;
		}

		//RMA
		//find UWB with greatest number of slot assignments
		uint8 max_assignments = 0; //TODO may need uint16?
		uint8 max_uwb_index = 255;
		for(uint8 u = 1; u < inst->uwbListLen; u++)//0 reserved for self
		{
			uint8 slotsLength = this->uwbListTDMAInfo[u].slotsLength; //TODO may need uint16?
			if(info->framelength > this->uwbListTDMAInfo[u].framelength && this->uwbListTDMAInfo[u].slotsLength != 0)
			{
				slotsLength *= info->framelength/this->uwbListTDMAInfo[u].framelength;
			}

			if(slotsLength > max_assignments)
			{
				max_assignments = slotsLength;
				max_uwb_index = u;
			}
		}

		if(max_uwb_index != 255 && max_assignments > 1)
		{
			uint8 slot;
			memcpy(&slot, &this->uwbListTDMAInfo[max_uwb_index].slots[0], sizeof(uint8));
			uint8 mod_slot = slot % info->framelength;

			this->assign_slot(info, mod_slot, TRUE);
			this->deconflict_uwb_pair(this, info, &this->uwbListTDMAInfo[max_uwb_index]); //TODO consider handling differently (could maybe be more efficient?)
			assignment_made = TRUE;
		}

		if(assignment_made == TRUE)
		{
			break;
		}

		//DF
		info->framelength *= 2;
	}
}

static void build_new_network(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
//	uint32 time_now = portGetTickCnt();
	uint32 time_now_us = portGetTickCntMicro();
	bool safeAssign = FALSE;

	//clear all tdma information
	this->tdma_free_all_slots(this);

	//build the initial TDMA
//	this->uwbFrameStartTimes_stm32[0] = time_now - this->slotDuration_ms;//TODO handle timer wrapping...
//	this->lastSlotStartTime_stm32 = time_now;

	//build the initial TDMA
//	this->uwbFrameStartTimes64[0] = timestamp_subtract64(time_now_us, this->slotDuration_us);
	this->uwbListTDMAInfo[0].frameStartTime = timestamp_subtract64(time_now_us, this->slotDuration_us);
	this->lastSlotStartTime64 = time_now_us;


	this->uwbListTDMAInfo[0].framelength = (uint8)MIN_FRAMELENGTH;
	this->uwbListTDMAInfo[inst->uwbToRangeWith].framelength = (uint8)MIN_FRAMELENGTH;

	this->assign_slot(&this->uwbListTDMAInfo[0], 1, safeAssign);
	this->assign_slot(&this->uwbListTDMAInfo[inst->uwbToRangeWith],  2, safeAssign);
	this->assign_slot(&this->uwbListTDMAInfo[0], 3, safeAssign);
}


static bool deconflict_slot_assignments(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	bool conflict = FALSE;

	while(TRUE)
	{
		bool conflict_this_iter = FALSE;
		//first deconflict slots in neighbor, hidden, and twice hidden
		for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
		{
//			if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
			if(this->uwbListTDMAInfo[i].connectionType != UWB_LIST_INACTIVE)
			{
				for(int j = i+1; j < inst->uwbListLen; j++)
				{
//					if(inst->uwbListType[j] != UWB_LIST_INACTIVE && j != i)
					if(this->uwbListTDMAInfo[j].connectionType != UWB_LIST_INACTIVE && j != i)
					{
						//first check if their list type requires deconflicting
//						if((inst->uwbListType[i] == UWB_LIST_NEIGHBOR && inst->uwbListType[j] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[j] == UWB_LIST_NEIGHBOR && inst->uwbListType[i] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[i] == UWB_LIST_TWICE_HIDDEN && inst->uwbListType[j] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[i] == UWB_LIST_HIDDEN && inst->uwbListType[j] == UWB_LIST_HIDDEN))
						if((this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_TWICE_HIDDEN) ||
						   (this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[i].connectionType == UWB_LIST_TWICE_HIDDEN) ||
						   (this->uwbListTDMAInfo[i].connectionType == UWB_LIST_TWICE_HIDDEN && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_TWICE_HIDDEN) ||
						   (this->uwbListTDMAInfo[i].connectionType == UWB_LIST_HIDDEN && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_HIDDEN))
						{
							continue;
						}

						if(this->deconflict_uwb_pair(this, &this->uwbListTDMAInfo[i],&this->uwbListTDMAInfo[j]))
						{
							conflict = TRUE;
							conflict_this_iter = TRUE;
							break;
						}
					}
				}
			}

			if(conflict_this_iter)
			{
				break;
			}
		}

		if(conflict_this_iter)
		{
			continue;
		}


		//next deconflict slots between self and neighbor, hidden, and twice hidden
		for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
		{
//			if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
			if(this->uwbListTDMAInfo[i].connectionType != UWB_LIST_INACTIVE)
			{
				if(this->deconflict_uwb_pair(this, &this->uwbListTDMAInfo[0], &this->uwbListTDMAInfo[i]))
				{
					conflict = TRUE;
					conflict_this_iter = TRUE;
				}
			}
			if(conflict_this_iter)
			{
				break;
			}
		}

		if(conflict_this_iter)
		{
			continue;
		}

		break; //no conflicts found this iteration, break out of while loop
	}

	return conflict;
}


//bool indexed_deconflict_slot_assignments(struct TDMAHandler *this, bool selfDeconflict, bool *uwbListDeconflict)
//{
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//	bool conflict = FALSE;
//
//	while(TRUE)
//	{
//		bool conflict_this_iter = FALSE;
//		//first deconflict slots in neighbor, hidden, and twice hidden
//		for(int i = 0; i < inst->uwbListLen; i++)
//		{
//			if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
//			{
//				for(int j = i+1; j < inst->uwbListLen; j++)
//				{
//					if(inst->uwbListType[j] != UWB_LIST_INACTIVE && j != i)
//					{
//						//first check if their list type requires deconflicting
//						if((inst->uwbListType[i] == UWB_LIST_NEIGHBOR && inst->uwbListType[j] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[j] == UWB_LIST_NEIGHBOR && inst->uwbListType[i] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[i] == UWB_LIST_TWICE_HIDDEN && inst->uwbListType[j] == UWB_LIST_TWICE_HIDDEN) ||
//						   (inst->uwbListType[i] == UWB_LIST_HIDDEN && inst->uwbListType[j] == UWB_LIST_HIDDEN))
//						{
//							continue;
//						}
//
//						if(this->deconflict_uwb_pair(this, &this->uwbListTDMAInfo[i], &this->uwbListTDMAInfo[j]))
//						{
//							conflict = TRUE;
//							conflict_this_iter = TRUE;
//							break;
//						}
//					}
//				}
//			}
//
//			if(conflict_this_iter)
//			{
//				break;
//			}
//		}
//
//		if(conflict_this_iter)
//		{
//			continue;
//		}
//
//
//		//next deconflict slots between self and neighbor, hidden, and twice hidden
//		for(int i = 0; i < inst->uwbListLen; i++)
//		{
//			if(inst->uwbListType[i] != UWB_LIST_INACTIVE)
//			{
//				if(this->deconflict_uwb_pair(this, &this->myTDMAInfo, &this->uwbListTDMAInfo[i]))
//				{
//					conflict = TRUE;
//					conflict_this_iter = TRUE;
//				}
//			}
//			if(conflict_this_iter)
//			{
//				break;
//			}
//		}
//
//		if(conflict_this_iter)
//		{
//			continue;
//		}
//
//		break; //no conflicts found this iteration, break out of while loop
//	}
//
//	return conflict;
//}


//return true if a conflict was found
static bool deconflict_uwb_pair(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b)
{
	bool conflict = FALSE;

	while(TRUE)
	{
		bool conflict_this_iter = false;

		for(int sa = 0; sa < info_a->slotsLength; sa++)
		{
			uint8 slot_sa;
			memcpy(&slot_sa, &info_a->slots[sa], 1);

			for(int sb = 0; sb < info_b->slotsLength; sb++)
			{
				uint8 slot_sb;
				memcpy(&slot_sb, &info_b->slots[sb], 1);


				//check if slot is taken
				if(slot_sa >= info_b->framelength)
				{
					uint8 mod_slot_sa = slot_sa%info_b->framelength;

					if(mod_slot_sa == slot_sb)
					{
						//slot already assigned, deconflict!
						this->deconflict_slot_pair(this, info_a, info_b, sa, sb);
						conflict = TRUE;
						conflict_this_iter = TRUE;
						break;
					}
				}
				else if(slot_sb >= info_a->framelength)
				{
					uint8 mod_slot_sb = slot_sb%info_a->framelength;
					if(mod_slot_sb == slot_sa)
					{
						//slot already assigned, deconflict!
						this->deconflict_slot_pair(this, info_a, info_b, sa, sb);
						conflict = TRUE;
						conflict_this_iter = TRUE;
						break;
					}
				}
				else
				{
					if(slot_sa == slot_sb)
					{
						//slot already assigned, deconflict!
						this->deconflict_slot_pair(this, info_a, info_b, sa, sb);
						conflict = TRUE;
						conflict_this_iter = TRUE;
						break;
					}
				}
			}

			if(conflict_this_iter)
			{
				break;
			}
		}

		if(conflict_this_iter)
		{
			continue;
		}

		break; //no conflicts found this iterations, break while loop
	}

	return conflict;
}


//TODO assign reference in class struct!!! same with others written today
static void deconflict_slot_pair(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b, uint8 slot_idx_a, uint8 slot_idx_b)
{
	//procedure for deconflicting slots (PDS)
	//1.) delete a conflicting slot
	//		applicable if all but node with fewest slots has more than one slot assignment
	//		release conflicting slot from all but node with fewest slots
	//2.) divide the assignment
	//		applicable if multiple conflicting slots between to nodes
	//		release lowest slot from first, greatest from second
	//3.) double the frame and divide the assignment
	//		applicable if single conflict between two nodes and neither has another slot assignment
	//		double the framelength of one or both and assign one slot assignment to the first and other to the second
	//		make the change
	//4.) if any of 1-3.) is applied, check for conflicts again, and start again at 1.)

	//logic for 1.) and 2.) not explicitly programmed.
	//Should be taken care of by repeatedly checking for conflicts and executing the code block below
	if(info_a->slotsLength > 1 || info_b->slotsLength > 1)
	{
		if(info_a->slotsLength >= info_b->slotsLength)
		{
			//release slot from uwb_a
			uint8 slot_a;
			memcpy(&slot_a, &info_a->slots[slot_idx_a], sizeof(uint8));
			this->free_slot(info_a, slot_a);
			return;
		}
		else
		{
			//release slot from uwb_b
			uint8 slot_b;
			memcpy(&slot_b, &info_b->slots[slot_idx_b], sizeof(uint8));
			this->free_slot(info_b, slot_b);
			return;
		}
	}

	//double the frame and divide the assignment
	if(info_a->framelength == info_b->framelength)
	{
		uint8 slot_b;
		memcpy(&slot_b, &info_b->slots[slot_idx_b], sizeof(uint8));
		slot_b += info_b->framelength;
		memcpy(&info_b->slots[slot_idx_b], &slot_b, sizeof(uint8));
		info_a->framelength *= 2;
		info_b->framelength *= 2;
	}
	else if(info_a->framelength > info_b->framelength)
	{
		uint8 slot_a;
		memcpy(&slot_a, &info_a->slots[slot_idx_a], sizeof(uint8));
		uint8 mod_a = slot_a % (2*info_b->framelength);

		uint8 slot_b;
		memcpy(&slot_b, &info_b->slots[slot_idx_b], sizeof(uint8));

		if(mod_a == slot_b)
		{
			slot_b += info_b->framelength;
			memcpy(&info_b->slots[slot_idx_b], &slot_b, sizeof(uint8));
		}

		info_b->framelength *= 2;
	}
	else if(info_a->framelength < info_b->framelength)
	{
		uint8 slot_b;
		memcpy(&slot_b, &info_b->slots[slot_idx_b], sizeof(uint8));
		uint8 mod_b = slot_b % (2*info_a->framelength);

		uint8 slot_a;
		memcpy(&slot_a, &info_a->slots[slot_idx_a], sizeof(uint8));

		if(mod_b == slot_a)
		{
			slot_a += info_a->framelength;
			memcpy(&info_a->slots[slot_idx_a], &slot_a, sizeof(uint8));
		}

		info_a->framelength *= 2;
	}

	//re-checking for conflicts handled in calling function
}

//check if this uwb has any TDMA conflicts with others in the uwbList
static bool self_conflict(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	struct TDMAInfo *info_a = &this->uwbListTDMAInfo[0];

	for(int b = 1; b < inst->uwbListLen; b++)
	{
//		if(inst->uwbListType[b] == UWB_LIST_NEIGHBOR ||
//		   inst->uwbListType[b] == UWB_LIST_HIDDEN   ||
//		   inst->uwbListType[b] == UWB_LIST_TWICE_HIDDEN)
		if(this->uwbListTDMAInfo[b].connectionType == UWB_LIST_NEIGHBOR ||
			this->uwbListTDMAInfo[b].connectionType == UWB_LIST_HIDDEN   ||
			this->uwbListTDMAInfo[b].connectionType == UWB_LIST_TWICE_HIDDEN)
		{
			struct TDMAInfo *info_b = &this->uwbListTDMAInfo[b];

			for(int sa = 0; sa < info_a->slotsLength; sa++)
			{
				uint8 slot_sa;
				memcpy(&slot_sa, &info_a->slots[sa], sizeof(uint8));

				for(int sb = 0; sb < info_b->slotsLength; sb++)
				{
					uint8 slot_sb;
					memcpy(&slot_sb, &info_b->slots[sb], sizeof(uint8));


					//check if slot is taken
					if(slot_sa >= info_b->framelength)
					{
						uint8 mod_slot_sa = slot_sa%info_b->framelength;

						if(mod_slot_sa == slot_sb)
						{
							return TRUE;
						}
					}
					else if(slot_sb >= info_a->framelength)
					{
						uint8 mod_slot_sb = slot_sb%info_a->framelength;
						if(mod_slot_sb == slot_sa)
						{
							return TRUE;
						}
					}
					else
					{
						if(slot_sa == slot_sb)
						{
							return TRUE;
						}
					}
				}
			}
		}
	}

	return FALSE;
}

static void free_slot(struct TDMAInfo *info, uint8 slot)
{
	bool assigned = TRUE;

	while(assigned == TRUE) //duplicate assignments shouldn't exist, but will make sure to remove any just in case
	{
		uint8 slot_index = 255;
		assigned = FALSE;
		if(info->slots != NULL && info->slotsLength != 0)
		{
			for(int i = 0; i < info->slotsLength; i++)
			{
				if(memcmp(&info->slots[i], &slot, sizeof(uint8)) == 0)
				{
					assigned = TRUE;
					slot_index = i;
					break;
				}
			}
		}

		//if assigned, remove from array
		if(assigned == TRUE)
		{
			memcpy(&info->slots[slot_index], &info->slots[slot_index + 1], sizeof(uint8)*(info->slotsLength - slot_index - 1));
			info->slotsLength -= 1;
			if(info->slotsLength <= 0)
			{
				info->slotsLength = 0;
				free(info->slots);
				info->slots = NULL;
			}
		}
	}


	return;
}

static void free_slots(struct TDMAInfo *info)
{
	if(info->slots != NULL)
	{
		free(info->slots);
		info->slots = NULL;
	}

	info->slotsLength = 0;
	info->framelength = MIN_FRAMELENGTH;
}

static void uwblist_free_slots(struct TDMAHandler *this, uint8 uwb_index)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	if(uwb_index >= inst->uwbListLen)
	{
		//out of bounds!
//		uint8 debug_msg[100];
//		 int n = sprintf((char*)&debug_msg[0], "uwblist_free_slots: uwb_index %u out of bounds", uwb_index);
//		 send_usbmessage(&debug_msg[0], n);
//		 usb_run();
		return;
	}

	this->free_slots(&this->uwbListTDMAInfo[uwb_index]);

	return;
}

static void tdma_free_all_slots(struct TDMAHandler *this)
{
	for(int i = 0; i < (int)UWB_LIST_SIZE; i++)
	{
		if(this->uwbListTDMAInfo[i].slots != NULL)
		{
			free(this->uwbListTDMAInfo[i].slots);
			this->uwbListTDMAInfo[i].slots = NULL;
		}

		this->uwbListTDMAInfo[i].slotsLength = 0;
		this->uwbListTDMAInfo[i].framelength = MIN_FRAMELENGTH;
	}

	return;
}


//TODO
static void enter_discovery_mode(struct TDMAHandler *this)
{
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//	inst->inf_msg.messageData[FCODE] = RTLS_DEMO_MSG_INF_INIT;

	uint32 time_now = portGetTickCnt();
	this->discoveryStartTime = time_now;
	this->last_blink_time = time_now;
	this->set_discovery_mode(this, WAIT_INF_REG, time_now);
	this->collectInfStartTime = time_now;

	this->tdma_free_all_slots(this);
}

static void set_discovery_mode(struct TDMAHandler *this, DISCOVERY_MODE discovery_mode, uint32 time_now)
{
	this->discovery_mode_start_time = time_now;
	this->discovery_mode = discovery_mode;


//	uint8 debug_msg[100];
//	 int n = sprintf((char*)&debug_msg[0], "set_discovery_mode: %s", get_discovery_modes_string(discovery_mode));
//	 send_usbmessage(&debug_msg[0], n);
//	 usb_run();

	switch (discovery_mode)//TODO make sure all modes are captured here...
	{
		case WAIT_INF_REG:
		{
			this->discovery_mode_duration = 0;
			this->discovery_mode_expires = FALSE;
			break;
		}
		case COLLECT_INF_REG:
		{
			this->collectInfStartTime = time_now;
			this->discovery_mode_duration = this->collectInfDuration;
			this->discovery_mode_expires = TRUE;
			break;
		}
		case WAIT_INF_INIT:
		{
			this->discovery_mode_duration = 400;//TODO use a smart timeout value
			this->discovery_mode_expires = TRUE;
			break;
		}
		case WAIT_RNG_INIT:
		{
			this->discovery_mode_duration = 400;//TODO use a smart timeout value
			this->discovery_mode_expires = TRUE;
			break;
		}
		case WAIT_SEND_SUG:
		{
//			uint8 debug_msg[200];
//			int n = sprintf((char*)&debug_msg[0], "SET WAIT_SEND_SUG start");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			//find common frame start time among neighbors
			instance_data_t *inst = instance_get_local_structure_ptr(0);

			//TODO need to modify this.
			//need to discriminate between different subnetworks.
			//count the number of UWBs that belong to each subnetwork
			//select the subnetwork with the largest number of UWBs
			//find the common frame start time among the UWBs in that network
			//if subnetworks do indeed exist, set the appropriate rebase variables here and in our inf_message

			//keep track of which number each UWB belongs to
			//then keep track of which was selected so we can iterate over it later in this function
			uint8 num_sub_networks = 0;
			uint8 sub_network_membership[UWB_LIST_SIZE] = {};
			uint8 sub_network_selected = 0;
			uint8 sub_network_members[UWB_LIST_SIZE-1] = {0}; //cannot be more subnetworks than other UWBs
			uint64 sub_network_tsfs[UWB_LIST_SIZE-1] = {0};
			uint8 sub_network_base_framelength[UWB_LIST_SIZE-1] = {0};
			//TODO handle number wrapping
			uint64 time_now_us = portGetTickCntMicro();
			uint64 tcommon;
			uint64 shortestFrameDuration = this->maxFramelength*this->slotDuration_us;



			for(int i=1; i < inst->uwbListLen; i++) //zero reserved for self
			{
//				if(inst->uwbListType[i] != UWB_LIST_NEIGHBOR)
				if(this->uwbListTDMAInfo[i].connectionType != UWB_LIST_NEIGHBOR)
				{
					continue; //TODO consider a way to account for hidden and twice hidden to determine which network is larger
				}

				struct TDMAInfo *info_i = &this->uwbListTDMAInfo[i];
//				uint64 framelengthDuration_us = info_i->framelength*this->slotDuration_us;
//				uint64 timeSinceFrameStart_us = get_dt64(this->uwbFrameStartTimes64[i], time_now_us);
				uint64 timeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[i].frameStartTime, time_now_us);

				if(timeSinceFrameStart_us > 100000000)
				{
					sub_network_membership[i] = 255; //disregard
					continue;
				}

				//if tsfs > framelengthDuration_us, reduce it by framelengthDuration_us

				//test the ith UWB against the jth subnetwork
				for(int j=0; j < num_sub_networks; j++)
				{
					uint64 diff_us = 0;

					uint8 min_fl = info_i->framelength;
					if(sub_network_base_framelength[j] < min_fl)
					{
						min_fl = sub_network_base_framelength[j];
					}

					uint64 min_framelengthDuration = min_fl*this->slotDuration_us;
					uint64 diff_tsfs = 0;

					if(timeSinceFrameStart_us <= sub_network_tsfs[j])
					{
						diff_tsfs = sub_network_tsfs[j] - timeSinceFrameStart_us;
					}
					else
					{
						diff_tsfs = timeSinceFrameStart_us - sub_network_tsfs[j];
					}

					uint64 diff_tsfs_mod = diff_tsfs%min_framelengthDuration;

					if(diff_tsfs_mod <= 0.5*min_framelengthDuration)
					{
						diff_us = diff_tsfs_mod;
					}
					else
					{
						diff_us = min_framelengthDuration - diff_tsfs_mod;
					}


					//if difference is below the threshold, it belongs to this subnetwork
					//if not, it may belong to another one already listed,
					//if not, create a new one...
					if(diff_us < this->frameSyncThreshold_us)
					{
						sub_network_members[j]++;
						sub_network_membership[i] = j;
						break;
					}
					else if(j == num_sub_networks - 1)
					{
						//reached the last listed sub_netowrk, list a new subnetwork.
						sub_network_members[num_sub_networks] = 1;
						sub_network_base_framelength[num_sub_networks] = this->uwbListTDMAInfo[i].framelength;
						sub_network_tsfs[num_sub_networks] = get_dt64(this->uwbListTDMAInfo[i].frameStartTime, time_now_us);
						sub_network_membership[num_sub_networks] = num_sub_networks;
						num_sub_networks++;
						break;
					}
				}

				//no subnetworks listed yet, set the first one.
				if(num_sub_networks == 0)
				{
					sub_network_members[num_sub_networks] = 1;
					sub_network_base_framelength[num_sub_networks] = this->uwbListTDMAInfo[i].framelength;
					sub_network_tsfs[num_sub_networks] = timeSinceFrameStart_us;
					sub_network_membership[num_sub_networks] = num_sub_networks;
					num_sub_networks++;
				}
			}

//			debug_msg[200];
//			n = sprintf((char*)&debug_msg[0], "checkpoint 1");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			//now select the subnetwork with the greatest number of uwbs
			uint8 max_num = 0;
			for(int i=0; i < num_sub_networks; i++)
			{
				if(sub_network_members[i] > max_num)
				{
					max_num = sub_network_members[i];
					sub_network_selected = i;
				}
			}

			uint64 tnext[max_num]; //TODO zero?
			uint8 neighborIndices[max_num]; //TODO zero?
			uint64 latest_tnext;
			uint8 nidx = 0;
			uint64 slotDuration_us = this->slotDuration_us;

			for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
			{
//				if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR && sub_network_membership[i] == sub_network_selected)
				if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && sub_network_membership[i] == sub_network_selected)
				{
					neighborIndices[nidx] = i;
//					tnext[nidx] = this->uwbFrameStartTimes64[i];
					tnext[nidx] = this->uwbListTDMAInfo[i].frameStartTime;
					while(time_now_us > tnext[nidx]) //TODO handle number wrapping...
					{
						tnext[nidx] += this->uwbListTDMAInfo[i].framelength*slotDuration_us;
					}
					nidx++;

					if(this->uwbListTDMAInfo[i].framelength*slotDuration_us < shortestFrameDuration)
					{
						shortestFrameDuration = this->uwbListTDMAInfo[i].framelength*slotDuration_us;
					}
				}
			}

//			debug_msg[200];
//			n = sprintf((char*)&debug_msg[0], "checkpoint 2");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			tcommon = tnext[0];
			latest_tnext = tcommon;
			bool converged = FALSE;
			while(converged == FALSE)
			{
				converged = TRUE;
				for(int i = 0; i < nidx; i++)
				{
					uint64 frameduration = this->uwbListTDMAInfo[neighborIndices[i]].framelength*slotDuration_us;
					while(tnext[i] < tcommon && tcommon - tnext[i] >= frameduration) //include small buffer
					{
						tnext[i] += frameduration;
					}


					if(tnext[i] > tcommon && tnext[i] - tcommon >= shortestFrameDuration) //TODO maybe include a small buffer to account for small timing errors?
					{
						tcommon = tnext[i];
						converged = FALSE;
					}

					if(tnext[i] > latest_tnext)
					{
						latest_tnext = tnext[i];
					}
				}
			}


//			if(num_sub_networks > 1)
//			{
//				//need indicate rebase in inf message so other networks will have a chance to rebase to the network selected by this UWB
//				this->rebase_pending = TRUE;
//				this->rebase_tx = FALSE;
//				this->rebase_frameStartTime64 = latest_tnext;
//			}


			//expire as the beginning of the common frame start time
			this->discovery_mode_duration = (uint32)(get_dt64(time_now_us, latest_tnext)/1000); //TODO make duration in us?
			this->discovery_mode_expires = TRUE;

			this->deconflict_slot_assignments(this);
			//assign self slot
			this->find_assign_slot(this);
			//construct SUG packet
			this->populate_inf_msg(this, RTLS_DEMO_MSG_INF_SUG);


			//back-track the frame start time so we can inform the need to rebase
			//and keep in sync with the subnetwork we initially chose to sync with
//			this->uwbFrameStartTimes64[0] = latest_tnext;
			this->uwbListTDMAInfo[0].frameStartTime = latest_tnext;
			//TODO handle number wrapping
			uint64 myFrameDuration = this->uwbListTDMAInfo[0].framelength*this->slotDuration_us;
//			while(this->uwbFrameStartTimes64[0] > time_now_us)
			while(this->uwbListTDMAInfo[0].frameStartTime > time_now_us)
			{
//				this->uwbFrameStartTimes64[0] -= myFrameDuration;
				this->uwbListTDMAInfo[0].frameStartTime -= myFrameDuration;
			}

			uint64 myTimeSinceFrameStart = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now);
			uint8 slot = myTimeSinceFrameStart/this->slotDuration_us; //integer division rounded down
			this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)(this->slotDuration_us*slot);

//			this->lastSlotStartTime64 = this->uwbFrameStartTimes64[0];

//			debug_msg[200];
//			n = sprintf((char*)&debug_msg[0], "SET WAIT_SEND_SUG end");
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();

			break;
		}
		case EXIT:
		{
			this->discovery_mode_duration = 0;
			this->discovery_mode_expires = FALSE;
			break;
		}
		default:
		{
			break;
		}

	}


}

static void check_discovery_mode_expiration(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	if(inst->mode == DISCOVERY)
	{
		if(this->discovery_mode_expires == TRUE)
		{
			uint32 time_now = portGetTickCnt();
			uint32 timeSinceModeStart = get_dt32(this->discovery_mode_start_time, time_now);
			if(timeSinceModeStart > this->discovery_mode_duration)
			{
				//discovery mode expired
				DISCOVERY_MODE new_mode = WAIT_INF_REG;
				if(this->discovery_mode == COLLECT_INF_REG)
				{
					new_mode = WAIT_SEND_SUG;
//					inst->testAppState = TA_TX_SELECT;
					inst->testAppState = TA_RXE_WAIT; //still collect RNG_REPORT messages while we wait to send our SUG message

					//NOTE moved to set_discovery_mode for WAIT_SEND_SUG
					//TODO remove below
					//deconflict gathered tdma info
//					this->deconflict_slot_assignments(this);
//					//assign self slot
//					this->find_assign_slot(this);
//					//construct SUG packet
//					this->populate_inf_msg(this, RTLS_DEMO_MSG_INF_SUG);

					//TODO how to actually send SUG message? TA_TX_SELECT?
					//could go directly to TA_TXSUG_WAIT_SEND and delay until start of next frame...
					//that could remove the need for extra logic in TA_TX_SELECT...
				}
				else if(this->discovery_mode == WAIT_SEND_SUG)
				{
//					this->uwbFrameStartTimes64[0] = portGetTickCntMicro();
//					this->lastSlotStartTime64 = this->uwbFrameStartTimes64[0];
					inst->testAppState = TA_TX_SELECT;
					new_mode = SEND_SUG;
				}

				this->set_discovery_mode(this, new_mode, time_now);
			}
		}
	}
}


static bool check_timeouts(struct TDMAHandler *this, uint32 time_now)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint32 delta_t = 0;
	bool rangingUWBTimeout = FALSE;
	bool setInactive = FALSE;
	bool updateINF = FALSE;
	bool noNeighbors = FALSE;

	for(int i=1; i < inst->uwbListLen; i++)//0 reserved for self, timeout not applicable
	{
		struct TDMAInfo *info = &this->uwbListTDMAInfo[i];

		switch (info->connectionType)
		{
			case UWB_LIST_NEIGHBOR:
			{
				delta_t = get_dt32(info->lastCommNeighbor, time_now);

				if(delta_t > UWB_COMM_TIMEOUT)
				{
					if(info->lastCommHidden != 0)
					{
						info->connectionType = UWB_LIST_HIDDEN;
						updateINF = TRUE;
					}
					else if(info->lastCommTwiceHidden != 0)
					{
						info->connectionType = UWB_LIST_TWICE_HIDDEN;
						updateINF = TRUE;
					}
					else
					{
						info->connectionType = UWB_LIST_INACTIVE;
						updateINF = TRUE;
						setInactive = TRUE;
						this->free_slots(info);
					}

					if(instfindnumneighbors(inst) <= 0)
					{
						noNeighbors = TRUE;
					}

					if(inst->uwbToRangeWith == i)
					{
						rangingUWBTimeout = TRUE;
					}
				}

				break;
			}
			case UWB_LIST_HIDDEN:
			{
				delta_t = get_dt32(info->lastCommHidden, time_now);

				if(delta_t > UWB_COMM_TIMEOUT)
				{
					if(info->lastCommTwiceHidden != 0)
					{
						info->connectionType = UWB_LIST_TWICE_HIDDEN;
						updateINF = TRUE;
					}
					else
					{
						info->connectionType = UWB_LIST_INACTIVE;
						this->free_slots(info);
						setInactive = TRUE;
						updateINF = TRUE;
					}
				}

				break;
			}
			case UWB_LIST_TWICE_HIDDEN:
			{
				delta_t = get_dt32(info->lastCommTwiceHidden, time_now);

				if(delta_t > UWB_COMM_TIMEOUT)
				{
					info->connectionType = UWB_LIST_INACTIVE;
					this->free_slots(info);
					setInactive = TRUE;
					updateINF = TRUE;
				}

				break;
			}
			case UWB_LIST_INACTIVE:
			{
				//TODO figure out the why this doesn't work.
				//specifically when instremoveuwb(inst, i); and i--; are used


//				//find the most recent comm...
//				uint32 newest = 0;
//				if(info->lastCommNeighbor != 0)
//				{
//					newest = info->lastCommNeighbor;
//				}
//
//				if(info->lastCommHidden != 0)
//				{
//					if(info->lastCommHidden > newest)//TODO number wrapping
//					{
//						newest = info->lastCommHidden;
//					}
//				}
//
//				if(info->lastCommTwiceHidden != 0)
//				{
//					if(info->lastCommTwiceHidden > newest)//TODO number wrapping
//					{
//						newest = info->lastCommTwiceHidden;
//					}
//				}
//
//				if(newest != 0)
//				{
//					delta_t = get_dt32(newest, time_now);
//
//					if(delta_t > UWB_COMM_TIMEOUT*2)
//					{
//						uint64 uwb_addr = 0;
//						uwb_addr = instance_get_uwbaddr(i);
//						uint8 debug_msg[100];
//						int n = sprintf((char*)&debug_msg[0], "REMOVE UWB: %d, %llX", i, uwb_addr);
//						 send_usbmessage(&debug_msg[0], n);
//						 usb_run();
//						instremoveuwb(inst, i); //TODO finish testing to make sure it works like it should.
//						i--; //TODO is this a probelm if i=1 then i=0?
//
//					}
//				}


				break;
			}
			default:
			{
				//invalid list type
				break;
			}
		}
	}

	//one of the UWBs we were tracking became inactive,
	//reoptimize our TDMA assignments and repopulate the inf message
	if(setInactive == TRUE)
	{
		this->free_slots(&this->uwbListTDMAInfo[0]);
		this->find_assign_slot(this);
	}

	//adjust the INF message to reflect any changes
	if(updateINF == TRUE)
	{
		this->populate_inf_msg(this, RTLS_DEMO_MSG_INF_UPDATE);
	}


	if(rangingUWBTimeout == TRUE)
	{
		inst->uwbToRangeWith = 255;
		if(inst->mode == ANCHOR || inst->mode == DISCOVERY)
		{
			inst->testAppState = TA_RXE_WAIT;
		}
		else
		{
			inst->testAppState = TA_TX_SELECT;
		}
	}

	return noNeighbors;
}


static void remove_uwbinfo(struct TDMAHandler *this, uint8 uwb_index)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	if(uwb_index > 0 && uwb_index < inst->uwbListLen)
	{
		this->free_slots(&this->uwbListTDMAInfo[uwb_index]);

		for(int i=uwb_index; i<inst->uwbListLen-1; i++)
		{
			struct TDMAInfo *info_a = &this->uwbListTDMAInfo[i];
			struct TDMAInfo *info_b = &this->uwbListTDMAInfo[i+1];

			info_a->connectionType = info_b->connectionType;
			info_a->lastCommNeighbor = info_b->lastCommNeighbor;
			info_a->lastCommHidden = info_b->lastCommHidden;
			info_a->lastCommTwiceHidden = info_b->lastCommTwiceHidden;
			info_a->lastRange = info_b->lastRange;
			info_a->frameStartTime = info_b->frameStartTime;
			info_a->framelength = info_b->framelength;
			info_a->slotsLength = info_b->slotsLength;
			info_a->slots = info_b->slots;
		}


		struct TDMAInfo *info_end = &this->uwbListTDMAInfo[inst->uwbListLen-1]; //maybe this!
		info_end->connectionType = UWB_LIST_INACTIVE;
		info_end->lastCommNeighbor = 0;
		info_end->lastCommHidden = 0;
		info_end->lastCommTwiceHidden = 0;
		info_end->lastRange = 0;
		info_end->frameStartTime = 0;
		info_end->framelength = MIN_FRAMELENGTH;
		this->free_slots(info_end);
	}
}





static void usb_dump_tdma(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	uint8 debug_msg[20000];
	int n = sprintf((char*)&debug_msg[0], "TDMA Handler Dump \n");
	int n_char = n;

//	n = sprintf((char*)&debug_msg[n_char], "framelength: %d, maxFramelength: %d \n", this->myTDMAInfo.framelength, this->maxFramelength);
//	n_char += n;

//	n = sprintf((char*)&debug_msg[n_char], "mySlots[%u]: ", this->myTDMAInfo.slotsLength);
//	n_char += n;
//	for(int i = 0; i < this->myTDMAInfo.slotsLength; i++)
//	{
//		uint8 slot;
//		memcpy(&slot, &this->myTDMAInfo.slots[i], sizeof(uint8));
//		n = sprintf((char*)&debug_msg[n_char], " %u ", slot);
//		n_char += n;
//	}
//	n = sprintf((char*)&debug_msg[n_char], "\n");
//	n_char += n;

	for(int u = 0; u < inst->uwbListLen; u++)
	{
		n = sprintf((char*)&debug_msg[n_char], "UWB %i framelength: %u \n", u, this->uwbListTDMAInfo[u].framelength);
		n_char += n;

		n = sprintf((char*)&debug_msg[n_char], "UWB %i Slots[%u]: \n", u, this->uwbListTDMAInfo[u].slotsLength);
		n_char += n;
		for(int i = 0; i < this->uwbListTDMAInfo[u].slotsLength; i++)
		{
			uint8 slot;
			memcpy(&slot, &this->uwbListTDMAInfo[u].slots[i], sizeof(uint8));
			n = sprintf((char*)&debug_msg[n_char], " %u ", slot);
			n_char += n;
		}
		n = sprintf((char*)&debug_msg[n_char], "\n");
		n_char += n;
	}



	int div = 100;
	int idx = 0;

	for(int i = 0; i + div < n_char; i += div)
	{
		idx = i;
		send_usbmessage(&debug_msg[i], div);
		usb_run();
		Sleep(10);
	}
	if(idx + 1 < n_char)
	{
		send_usbmessage(&debug_msg[idx], n_char - idx);
		usb_run();
	}

}

//uint8 get_largest_framelength(struct TDMAHandler *this)
//{
//	instance_data_t *inst = instance_get_local_structure_ptr(0);
//
//	uint8 max_framelength = MIN_FRAMELENGTH;
//	if(this->myTDMAInfo.framelength > max_framelength)
//	{
//		max_framelength = this->myTDMAInfo.framelength;
//	}
//
//	for(int i = 0; i < inst->uwbListLen; i++)
//	{
//		if(inst->uwbListType[i] == UWB_LIST_NEIGHBOR)
//		{
//			if(this->uwbFramelengths[i] > max_framelength)
//			{
//				max_framelength = this->uwbFramelengths[i];
//			}
//		}
//
//	}
//
//	return max_framelength;
//}


static struct TDMAHandler new(){
	struct TDMAHandler ret = {};

	ret.slotDuration_ms = 50; //TODO should be a function of the data rate and the max number of UWBs
	ret.slotDuration_us = ret.slotDuration_ms*1000;

	ret.slot_transition = &slot_transition;
	ret.frame_sync = &frame_sync;
	ret.tx_sync_msg = &tx_sync_msg;
	ret.tx_select  = &tx_select;
	ret.check_blink  = &check_blink;

	ret.populate_inf_msg = &populate_inf_msg;
	ret.update_inf_tsfs = &update_inf_tsfs;
	ret.process_inf_msg = &process_inf_msg;
	ret.check_tdma_diff = &check_tdma_diff;

	ret.poll_delay = &poll_delay;
	ret.slot_assigned = &slot_assigned;
	ret.assign_slot = &assign_slot;
	ret.find_assign_slot = &find_assign_slot;
	ret.build_new_network = &build_new_network;

	ret.free_slot = &free_slot;
	ret.free_slots = &free_slots;
	ret.tdma_free_all_slots = &tdma_free_all_slots;
	ret.uwblist_free_slots = &uwblist_free_slots;
	ret.enter_discovery_mode = &enter_discovery_mode;
	ret.set_discovery_mode = &set_discovery_mode;
	ret.check_discovery_mode_expiration = &check_discovery_mode_expiration;
	ret.check_timeouts = &check_timeouts;
	ret.remove_uwbinfo = &remove_uwbinfo;
	ret.usb_dump_tdma = &usb_dump_tdma;


	ret.deconflict_slot_assignments = &deconflict_slot_assignments;
	ret.deconflict_uwb_pair = &deconflict_uwb_pair;
	ret.deconflict_slot_pair = &deconflict_slot_pair;
	ret.self_conflict = &self_conflict;

	uint32 time_now_us = portGetTickCntMicro();

	//TODO create a function to clear the frame information!
	//and have it called in enter_discovery function!


	ret.maxFramelength = (uint8)MIN_FRAMELENGTH;
	while(ret.maxFramelength < (int)UWB_LIST_SIZE + 1)
	{
		ret.maxFramelength *= 2;
	}

	for(int i = 0; i < UWB_LIST_SIZE; i++)
	{
//		ret.uwbListTDMAInfo[i].uwbIndex = i;
		ret.uwbListTDMAInfo[i].framelength = (uint8)MIN_FRAMELENGTH;
		ret.uwbListTDMAInfo[i].slots = NULL;
		ret.uwbListTDMAInfo[i].slotsLength = 0;
		ret.uwbListTDMAInfo[i].frameStartTime = time_now_us;
		ret.uwbListTDMAInfo[i].connectionType = UWB_LIST_INACTIVE;
		ret.uwbListTDMAInfo[i].lastCommNeighbor = 0;
		ret.uwbListTDMAInfo[i].lastCommHidden = 0;
		ret.uwbListTDMAInfo[i].lastCommTwiceHidden = 0;
		ret.uwbListTDMAInfo[i].lastRange = 0;
	}
	ret.uwbListTDMAInfo[0].connectionType = UWB_LIST_SELF;

    ret.lastFST = time_now_us;
    ret.lastSlotStartTime64 = time_now_us;
    ret.infSentThisSlot = FALSE;
    ret.firstPollSentThisSlot = FALSE;
    ret.firstPollComplete = FALSE;
    ret.secondPollSentThisSlot = FALSE;
    ret.slotStartDelay_us = 4000;
    ret.frameSyncThreshold_us = 2*ret.slotStartDelay_us;
    ret.infMessageLength = 0;
    ret.rebase_pending = FALSE;
	ret.rebase_tx = FALSE;
	ret.rebase_frameStartTime64 = time_now_us;

    ret.enter_discovery_mode(&ret);
//    ret.collectInfDuration = ret.maxFramelength*ret.slotDuration;
    ret.collectInfDuration = 10000; //TODO change this back
	ret.waitInfDuration = ret.collectInfDuration;

	return ret;
}

const struct TDMAHandlerClass TDMAHandler={.new=&new};


