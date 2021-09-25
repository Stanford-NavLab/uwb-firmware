#include "tdma_handler.h"
#include "port.h"
#include "instance.h"
#include "lib.h"


extern void usb_run(void);
extern void send_usbmessage(uint8*, int);

//class methods
static bool slot_transition(struct TDMAHandler *this)
{
	bool transition = FALSE;
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	if(inst->mode == TAG ||
	   inst->mode == ANCHOR ||
	   (inst->mode == DISCOVERY && (this->discovery_mode == WAIT_SEND_SUG || this->discovery_mode == COLLECT_INF_REG)))
	{
		uint64 time_now_us = portGetTickCntMicro();
		uint64 timeSinceSlotStart64 = get_dt64(this->lastSlotStartTime64, time_now_us);

		if(timeSinceSlotStart64 >= this->slotDuration_us)
		{
			transition = TRUE;
			this->firstPollSentThisSlot = FALSE;
			this->firstPollResponse = FALSE;
			this->firstPollComplete = FALSE;
			this->secondPollSentThisSlot = FALSE;
			this->infSentThisSlot = FALSE;
			inst->canPrintUSB = TRUE;
			inst->canPrintLCD = TRUE;

			//we have transitioned into the next slot.
			//get the slot number and set the start time appropriately

			uint64 timeSinceFrameStart64 = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
			uint64 frameDuration64 = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;
			if(timeSinceFrameStart64 >= frameDuration64)
			{
				int div = timeSinceFrameStart64/frameDuration64;
				this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, frameDuration64*div);
				timeSinceFrameStart64 -= frameDuration64*div;
			}

			uint8 slot = timeSinceFrameStart64/(this->slotDuration_us); //integer division rounded down
			this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)(this->slotDuration_us*slot);

			if(inst->mode != DISCOVERY)
			{
				if(this->slot_assigned(&this->uwbListTDMAInfo[0], slot) == TRUE)
				{
					inst->mode = TAG;
					inst->testAppState = TA_TX_SELECT;
					//go to TX select, select the oldest uwb, send INF, then send POLL
				}
				else
				{
					//go to RX
					inst->mode = ANCHOR;
					inst->wait4ack = 0;
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
		this->firstPollResponse = FALSE;
		this->firstPollComplete = FALSE;
		this->secondPollSentThisSlot = FALSE;
	}

	return transition;
}

static uint64 update_frame_start(struct TDMAHandler *this){

	uint64 time_now_us = portGetTickCntMicro();
	uint64 frameDuration_us = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;
	uint64 timeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);

	if(timeSinceFrameStart_us > 1000000000) //if very large number, assume frame start time accidentally moved ahead of time now
	{
		uint64 diff_us = get_dt64(time_now_us, this->uwbListTDMAInfo[0].frameStartTime);
		int div = diff_us/frameDuration_us;
		if(diff_us%frameDuration_us != 0)
		{
			div += 1;
		}
		this->uwbListTDMAInfo[0].frameStartTime = timestamp_subtract64(this->uwbListTDMAInfo[0].frameStartTime, frameDuration_us*div);
		timeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
	}
	else if(timeSinceFrameStart_us >= frameDuration_us)
	{
		int div = timeSinceFrameStart_us/frameDuration_us;
		this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, frameDuration_us*div);
		timeSinceFrameStart_us -= timeSinceFrameStart_us*div;
	}

	return timeSinceFrameStart_us;
}

static void frame_sync(struct TDMAHandler *this, event_data_t *dw_event, uint8 framelength, uint64 timeSinceFrameStart_us, uint8 srcIndex, FRAME_SYNC_MODE mode)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

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

	//time from command tx to tx timestamp
	uint64 infCmdToTsDelay_us = TX_CMD_TO_TX_CB_DLY_US + inst->storedPreLen_us;

	//tx antenna delay
	uint64 tx_antenna_delay = (uint64)inst->defaultAntennaDelay;

	//time to propagate
	//NOTE: assuming zero since difference for speed of light travel time over 10cm and 100m is negligible for frame sync purposes

	//rx antenna delay
	//NOTE: antenna delay is captured by the RX timestamp

	//time from rx timestamp to now
	uint64 rxfs_process_delay = dwt_getdt(dw_event->timeStamp, dwt_time_now);

	uint64 txrx_delay =  (uint64)(convertdevicetimetosec(tx_antenna_delay + rxfs_process_delay)*1000000.0) + infCmdToTsDelay_us;

	uint64 hisTimeSinceFrameStart_us = timeSinceFrameStart_us + txrx_delay;
	this->uwbListTDMAInfo[srcIndex].frameStartTime = timestamp_subtract64(time_now_us, hisTimeSinceFrameStart_us);
	uint64 myFrameDuration = this->slotDuration_us*this->uwbListTDMAInfo[0].framelength;

	if(mode == FS_COLLECT)
	{
		return;
	}
	else if(mode == FS_ADOPT)
	{
		this->uwbListTDMAInfo[0].frameStartTime = this->uwbListTDMAInfo[srcIndex].frameStartTime;//NOTE gets processed further at end of function
	}
	else// if(mode == FS_AVERAGE || mode == FS_EVAL)
	{
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
			}
		}
		else
		{
			diff_tsfs = hisTimeSinceFrameStart_us - myTimeSinceFrameStart_us;
			diff_tsfs_mod = diff_tsfs%min_framelengthDuration;

			if(diff_tsfs_mod <= 0.5*min_framelengthDuration)
			{
				diff_us = diff_tsfs_mod;
			}
			else
			{
				diff_us = min_framelengthDuration - diff_tsfs_mod;
				diff_add = TRUE;
			}
		}


		//check if frame sync out of tolerance (don't xmit sync message in case of 0th slot misalignment)
		if(diff_us%(MIN_FRAMELENGTH*this->slotDuration_us) > this->frameSyncThreshold_us){
			if(MIN_FRAMELENGTH*this->slotDuration_us - diff_us%(MIN_FRAMELENGTH*this->slotDuration_us) > this->frameSyncThreshold_us){
				this->tx_sync_msg(this);
			}
		}
		else if(mode == FS_EVAL)
		{
			return;
		}

		uint8 div = 2;
		if(mode == FS_EVAL || diff_us > this->frameSyncThreshold_us) //if diff_us > threshold, there is 0th slot misalignment
		{
			div = 1;
		}

		if(diff_add == TRUE)
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, diff_us/div);
		}
		else
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_subtract64(this->uwbListTDMAInfo[0].frameStartTime, diff_us/div);
		}
	}

	uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
	if(myTimeSinceFrameStart_us > 100000000)
	{
		//if this is a very large number, then the frame start time was likely moved ahead of time_now_us.
		while(this->uwbListTDMAInfo[0].frameStartTime > time_now_us)
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_subtract64(this->uwbListTDMAInfo[0].frameStartTime, myFrameDuration);
		}

		myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);
	}
	else
	{
		while(myTimeSinceFrameStart_us >= myFrameDuration)
		{
			this->uwbListTDMAInfo[0].frameStartTime = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, myFrameDuration);
			myTimeSinceFrameStart_us -= myFrameDuration;
		}
	}

	uint8 slot = myTimeSinceFrameStart_us/this->slotDuration_us; //integer division rounded down
	this->lastSlotStartTime64 = timestamp_add64(this->uwbListTDMAInfo[0].frameStartTime, (uint64)(this->slotDuration_us*slot));
}


static bool tx_sync_msg(struct TDMAHandler *this)
{
	int psduLength = SYNC_FRAME_LEN_BYTES;

	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint64 time_now_us = portGetTickCntMicro();

	uint64 myTimeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);

	memcpy(&inst->sync_msg.messageData[SYNC_FRAMELENGTH], &this->uwbListTDMAInfo[0].framelength, sizeof(uint8));
	memcpy(&inst->sync_msg.messageData[SYNC_TSFS], &myTimeSinceFrameStart_us, 6);
	inst->sync_msg.seqNum = inst->frameSN++;

	inst->wait4ack = 0;

	dwt_writetxdata(psduLength, (uint8 *)&inst->sync_msg, 0) ; // write the frame data
	if(instancesendpacket(psduLength, DWT_START_RX_IMMEDIATE | inst->wait4ack, 0))
	{
		inst->previousState = TA_INIT;
		inst->nextState = TA_INIT;
		inst->testAppState = TA_RXE_WAIT;
		inst->wait4ack = 0;
		return FALSE;
	}
	else
	{
		inst->previousState = inst->testAppState;
		inst->testAppState = TA_TX_WAIT_CONF;	// wait confirmation

		inst->timeofTx = portGetTickCnt();
		inst->txDoneTimeoutDuration = inst->durationSyncTxDoneTimeout_ms; //NOTE timeout duration found experimentally
		return TRUE;
	}
}

static bool tx_select(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);

	uint64 time_now_us = portGetTickCntMicro();

	int uwb_index = 255;

	if(inst->mode == DISCOVERY)
	{
		if(this->discovery_mode == WAIT_INF_REG)
		{
			if(this->check_blink(this))
			{
				//time to blink
				uwb_index = 255;
				this->set_discovery_mode(this, WAIT_RNG_INIT, portGetTickCnt());
			}
			else
			{
				//not time to blink yet, keep waiting for RNG_INIT
				inst->wait4ack = 0;
				inst->testAppState = TA_RXE_WAIT;
				return TRUE;
			}
		}
		else if(this->discovery_mode == SEND_SUG)
		{
			//get time since slot start and make sure that's greater than delay
			uint64 timeSinceSlotStart = get_dt64(this->lastSlotStartTime64, time_now_us);

			if(timeSinceSlotStart <= this->slotStartDelay_us)
			{
				uwb_index = -1;
			}
			else
			{
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

			uint32 timeSinceRange[UWB_LIST_SIZE] = {0}; //0th entry unused
			uint32 numNeighbors = 0;
			uint32 time_now = portGetTickCnt();

			//get time since range for each neighbor
			for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
			{
				if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR)
				{
					numNeighbors++;
					timeSinceRange[i] = get_dt32(this->uwbListTDMAInfo[i].lastRange, time_now);
				}
			}

			if(this->nthOldest > numNeighbors)
			{
				this->nthOldest = 1;
			}

			//get the nth oldest
			for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
			{
				if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR)
				{
					uint8 numOlder = 0;
					for(int j = 1; j < inst->uwbListLen; j++)//0 reserved for self
					{
						if(i != j)
						{
							if(this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR)
							{
								if(timeSinceRange[i] < timeSinceRange[j])
								{
									numOlder++;
								}
							}
						}
					}

					if(numOlder + 1 == this->nthOldest)
					{
						uwb_index = i;
						break;
					}
				}
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
		else if(this->secondPollSentThisSlot == FALSE && this->firstPollComplete == FALSE && this->firstPollResponse == FALSE)
		{
			this->secondPollSentThisSlot = TRUE;
			inst->testAppState = TA_TXPOLL_WAIT_SEND;
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
		if(timeSinceDiscoveryStart > this->waitInfDuration )
		{
			uint32 timeSinceBlink = get_dt32(this->last_blink_time, time_now);
			if(timeSinceBlink > (uint32)BLINK_PERIOD_MS + this->blinkPeriodRand)
			{
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

	//fcode
	int msgDataIndex = FCODE;
	memcpy(&inst->inf_msg.messageData[msgDataIndex], &inf_msg_type, sizeof(uint8));


	//time since frame start
	//populated immediately before being sent

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
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint64 time_now_us = portGetTickCntMicro();
	uint64 timeSinceFrameStart64 = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now_us);

	if(timeSinceFrameStart64 > 10000000)
	{
		timeSinceFrameStart64 = 0;
	}

	memcpy(&inst->inf_msg.messageData[TDMA_TSFS], &timeSinceFrameStart64, 6);
}


//General procedure for processing INF SUG, INF REG, and INF UPDATE (NOTE: slightly different for each INF_PROCESS_MODE)
//1. Check for differences with stored assignments
//		(a) exit if none exist
//2. Drop stored assignments for self, neighbor, hidden, and twice hidden nodes that appear
//	 in the INF message
//3. Copy all assignments for self, neighbors, hidden, and twice hidden nodes that appear
//	 in the INF message
//4. Check for conflicts between nodes in the INF message and nodes not in the INF message (excluding self) and deconflict using PDS
//5. Release self slot assignments and follow PSA
//6. Send INF message at beginning of allocated slot (handled elsewhere)
//returns TRUE if a change was made to the TDMA assingments, FALSE if invalid message FCODE or process mode or if no TDMA changes made
static bool process_inf_msg(struct TDMAHandler *this, uint8 *messageData, uint8 srcIndex, INF_PROCESS_MODE mode)
{
	//NOTE: this function does not handle TDMA deconflict

	bool tdma_modified = FALSE;

	uint32 time_now = portGetTickCnt();

	if((mode != CLEAR_ALL_COPY)     && //happens when we creat a new network
	   (mode != CLEAR_LISTED_COPY)	&& //happens most of the time while processing
	   (mode != COPY))				   //happens when collecting inf messages
	{
		//only process if valid mode supplied
		return FALSE;
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
		//do after cheking framelength because framelength will be reset
		this->free_slots(info);
	}

	info->framelength = MAX(framelength, info->framelength);

	msgDataIndex = TDMA_NUMS + 1;
	for(int s = 0; s < numSlots; s++)
	{
		memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
		msgDataIndex++;

		this->assign_slot(info, slot, safeAssign);
	}

	for(int i = 0; i < numNeighbors; i++)
	{
		uint8 address[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		memcpy(&address[0], &messageData[msgDataIndex], inst->addrByteSize);
		msgDataIndex += inst->addrByteSize;

		uint8 uwb_index = instgetuwblistindex(inst, &address[0], inst->addrByteSize);
		if(uwb_index != 0)
		{
			if(this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_INACTIVE || this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_TWICE_HIDDEN)
			{
				this->uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_HIDDEN;
			}

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
			//do after checking framelength because framelength reset
			this->free_slots(info);
		}
		info->framelength = MAX(framelength, info->framelength);


		msgDataIndex = msgDataIndexSave;
		for(int s = 0; s < numSlots; s++)
		{
			memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
			msgDataIndex++;

			this->assign_slot(info, slot, safeAssign);
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
			if(this->uwbListTDMAInfo[uwb_index].connectionType == UWB_LIST_INACTIVE)
			{
				this->uwbListTDMAInfo[uwb_index].connectionType = UWB_LIST_TWICE_HIDDEN;
			}

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
		if(tdma_modified == FALSE) //don't look for any more differences if we already know one exists
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
			//do after checking for difference because will reset framelength as well
			this->free_slots(info);
		}
		info->framelength = MAX(framelength, info->framelength);

		msgDataIndex = msgDataIndexSave;
		for(int s = 0; s < numSlots; s++)
		{
			memcpy(&slot, &messageData[msgDataIndex], sizeof(uint8));
			msgDataIndex++;

			this->assign_slot(info, slot, safeAssign);
		}
	}

	if(mode == CLEAR_LISTED_COPY)
	{
		//deconflict uncopied against copied. (excluding self)
		for(int i = 1; i < inst->uwbListLen; i++)
		{
			for(int j = i + 1; j < inst->uwbListLen; j++)
			{
				if((uwbListInMsg[i] == FALSE && uwbListInMsg[j] == TRUE) || (uwbListInMsg[i] == TRUE && uwbListInMsg[j] == FALSE))
				{
					if((this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR) ||
					   (this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[j].connectionType == UWB_LIST_HIDDEN)   ||
					   (this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR) ||
					   (this->uwbListTDMAInfo[j].connectionType == UWB_LIST_NEIGHBOR && this->uwbListTDMAInfo[i].connectionType == UWB_LIST_HIDDEN))
					{
						if(this->deconflict_uwb_pair(this, &this->uwbListTDMAInfo[i], &this->uwbListTDMAInfo[j]) == TRUE)
						{
							tdma_modified = TRUE;
						}
					}
				}
			}
		}

		if(tdma_modified == TRUE)
		{

			//if so, release all assignments from self
			this->free_slots(&this->uwbListTDMAInfo[0]);

			//find self a new slot assignment
			this->find_assign_slot(this);
		}
	}

	return tdma_modified;
}

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
//		assign self up to two unassigned slots (except for 0th slot); exit
//3.) Release Multiple Assigned Slots (RMA)
//		applicable if 2.) not applicable
//		applicable if one or more nodes has multiple slot assignments
//		release one slot from node with greatest number of slot assignments and assign to self; exit
//4.) Double the Frame (DF)
//		applicable if 2.) and 3.) not applicable
//		double own framelength and go back to 2.)
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
		uint8 slotsAssigned = 0;
		for(uint8 i = 1; i < info->framelength; i++)//do not assign to 0th slot
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
				assignment_made = TRUE;

				slotsAssigned++;
				if(slotsAssigned > 1) //assign self up to two empty slots
				{
					break;
				}
			}
		}

		if(assignment_made == TRUE)
		{
			break;
		}

		//RMA
		//find UWB with greatest number of slot assignments
		uint8 max_assignments = 0;
		uint8 max_uwb_index = 255;
		for(uint8 u = 1; u < inst->uwbListLen; u++)//0 reserved for self
		{
			uint8 slotsLength = this->uwbListTDMAInfo[u].slotsLength;
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
			this->deconflict_uwb_pair(this, info, &this->uwbListTDMAInfo[max_uwb_index]);
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
	uint64 time_now_us = portGetTickCntMicro();
	bool safeAssign = FALSE;

	//clear all tdma information
	this->tdma_free_all_slots(this);

	//build the initial TDMA
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
			if(this->uwbListTDMAInfo[i].connectionType != UWB_LIST_INACTIVE)
			{
				for(int j = i+1; j < inst->uwbListLen; j++)
				{
					if(this->uwbListTDMAInfo[j].connectionType != UWB_LIST_INACTIVE && j != i)
					{
						//first check if their list type requires deconflicting
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


static void enter_discovery_mode(struct TDMAHandler *this)
{
	uint32 time_now = portGetTickCnt();
	this->discoveryStartTime = time_now;
	this->last_blink_time = time_now;
	this->set_discovery_mode(this, WAIT_INF_REG, time_now);
	this->collectInfStartTime = time_now;

	instance_data_t *inst = instance_get_local_structure_ptr(0);
	inst->canPrintUSB = TRUE;
	inst->canPrintLCD = TRUE;

	this->tdma_free_all_slots(this);
}

static void set_discovery_mode(struct TDMAHandler *this, DISCOVERY_MODE discovery_mode, uint32 time_now)
{
	this->discovery_mode_start_time = time_now;
	this->discovery_mode = discovery_mode;

	switch (discovery_mode)
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
			this->discovery_mode_duration = this->slotDuration_ms;
			this->discovery_mode_expires = TRUE;
			break;
		}
		case WAIT_RNG_INIT:
		{
			instance_data_t *inst = instance_get_local_structure_ptr(0);
			this->discovery_mode_duration = inst->durationWaitRangeInit_ms;
			this->discovery_mode_expires = TRUE;
			break;
		}
		case WAIT_SEND_SUG:
		{
			//find common frame start time among neighbors
			instance_data_t *inst = instance_get_local_structure_ptr(0);

			//count the number of UWBs that belong to each subnetwork
			//select the subnetwork with the largest number of UWBs
			//find the common frame start time among the UWBs in that network

			//keep track of which number each UWB belongs to
			//then keep track of which was selected so we can iterate over it later in this function
			uint8 num_sub_networks = 0;
			uint8 sub_network_membership[UWB_LIST_SIZE] = {};
			uint8 sub_network_selected = 0;
			uint8 sub_network_members[UWB_LIST_SIZE-1] = {0}; //cannot be more subnetworks than other UWBs
			uint64 sub_network_tsfs[UWB_LIST_SIZE-1] = {0};
			uint8 sub_network_base_framelength[UWB_LIST_SIZE-1] = {0};
			uint64 time_now_us = portGetTickCntMicro();
			uint64 tcommon = 0;
			uint64 shortestFrameDuration = this->maxFramelength*this->slotDuration_us;


			for(int i=1; i < inst->uwbListLen; i++) //zero reserved for self
			{
				if(this->uwbListTDMAInfo[i].connectionType != UWB_LIST_NEIGHBOR)
				{
					continue;
				}

				struct TDMAInfo *info_i = &this->uwbListTDMAInfo[i];
				uint64 timeSinceFrameStart_us = get_dt64(this->uwbListTDMAInfo[i].frameStartTime, time_now_us);

				if(timeSinceFrameStart_us > 100000000)
				{
					sub_network_membership[i] = 255; //disregard
					continue;
				}

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

			uint64 tnext[max_num];
			uint8 neighborIndices[max_num];
			uint64 latest_tnext = 0;
			uint8 nidx = 0;
			uint64 slotDuration_us = this->slotDuration_us;

			for(int i = 1; i < inst->uwbListLen; i++)//0 reserved for self
			{
				if(this->uwbListTDMAInfo[i].connectionType == UWB_LIST_NEIGHBOR && sub_network_membership[i] == sub_network_selected)
				{
					neighborIndices[nidx] = i;
					tnext[nidx] = this->uwbListTDMAInfo[i].frameStartTime;
					while(time_now_us > tnext[nidx])
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

			tcommon = tnext[0];
			latest_tnext = tcommon;
			bool converged = FALSE;
			while(converged == FALSE)
			{
				converged = TRUE;
				for(int i = 0; i < nidx; i++)
				{
					uint64 frameduration = this->uwbListTDMAInfo[neighborIndices[i]].framelength*slotDuration_us;
					while(tnext[i] < tcommon && tcommon - tnext[i] >= frameduration - this->frameSyncThreshold_us)
					{
						tnext[i] += frameduration;
					}

					if(tnext[i] > tcommon && tnext[i] - tcommon >= this->frameSyncThreshold_us)
					{
						//increment the value based on tnext[0] to guarantee all uwbs
						//in this subnetwork will have frame sync errors within the threshold
						tcommon += shortestFrameDuration;
						converged = FALSE;
					}

					if(tnext[i] > latest_tnext)
					{
						latest_tnext = tnext[i];
					}
				}
			}

			//expire as the beginning of the common frame start time
			this->discovery_mode_duration = (uint32)(get_dt64(time_now_us, latest_tnext)/1000);
			this->discovery_mode_expires = TRUE;

			this->free_slots(&this->uwbListTDMAInfo[0]);
			this->deconflict_slot_assignments(this);
			//assign self slot
			this->find_assign_slot(this);
			//construct SUG packet
			this->populate_inf_msg(this, RTLS_DEMO_MSG_INF_SUG);


			//back-track the frame start time so we can inform the need to rebase
			//and keep in sync with the subnetwork we initially chose to sync with
			this->uwbListTDMAInfo[0].frameStartTime = latest_tnext;
			uint64 myFrameDuration = this->uwbListTDMAInfo[0].framelength*this->slotDuration_us;
			while(this->uwbListTDMAInfo[0].frameStartTime > time_now_us)
			{
				this->uwbListTDMAInfo[0].frameStartTime -= myFrameDuration;
			}

			uint64 myTimeSinceFrameStart = get_dt64(this->uwbListTDMAInfo[0].frameStartTime, time_now);
			uint8 slot = myTimeSinceFrameStart/this->slotDuration_us; //integer division rounded down
			this->lastSlotStartTime64 = this->uwbListTDMAInfo[0].frameStartTime + (uint64)(this->slotDuration_us*slot);

			break;
		}
		case SEND_SUG:
		{
			this->discovery_mode_duration = this->slotStartDelay_us*2;
			this->discovery_mode_expires = TRUE;
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
					inst->testAppState = TA_RXE_WAIT; //still collect RNG_REPORT messages while we wait to send our SUG message
				}
				else if(this->discovery_mode == WAIT_SEND_SUG)
				{
					inst->testAppState = TA_TX_SELECT;
					new_mode = SEND_SUG;
				}

				this->set_discovery_mode(this, new_mode, time_now);
			}
		}
	}
}


static bool check_timeouts(struct TDMAHandler *this)
{
	instance_data_t *inst = instance_get_local_structure_ptr(0);
	uint32 delta_t = 0;
	bool rangingUWBTimeout = FALSE;
	bool setInactive = FALSE;
	bool updateINF = FALSE;
	bool noNeighbors = FALSE;

	uint8 max_framelength = 4;
	for(int i=0; i < inst->uwbListLen; i++)
	{
		struct TDMAInfo *info = &this->uwbListTDMAInfo[i];
		if (info->connectionType != UWB_LIST_SELF && info->connectionType != UWB_LIST_INACTIVE)
		{
			if(info->framelength > max_framelength)
			{
				max_framelength = info->framelength;
			}
		}
	}
	inst->durationUwbCommTimeout_ms = 2*max_framelength*CEIL_DIV(inst->durationSlotMax_us,1000);


	for(int i=1; i < inst->uwbListLen; i++)//0 reserved for self, timeout not applicable
	{
		struct TDMAInfo *info = &this->uwbListTDMAInfo[i];
		if(info->connectionType == UWB_LIST_INACTIVE)
		{
			continue;
		}

		switch (info->connectionType)
		{
			case UWB_LIST_NEIGHBOR:
			{



				delta_t = get_dt32(info->lastCommNeighbor, portGetTickCnt()); //get time now here in case rx interrupt occurs before get_dt call

				if(delta_t > inst->durationUwbCommTimeout_ms)
				{
					info->connectionType = UWB_LIST_HIDDEN;
					updateINF = TRUE;

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
				delta_t = get_dt32(info->lastCommHidden, portGetTickCnt()); //get time now here in case rx interrupt occurs before get_dt call

				if(delta_t > inst->durationUwbCommTimeout_ms)
				{
					info->connectionType = UWB_LIST_TWICE_HIDDEN;
					updateINF = TRUE;
				}

				break;
			}
			case UWB_LIST_TWICE_HIDDEN:
			{
				delta_t = get_dt32(info->lastCommTwiceHidden, portGetTickCnt()); //get time now here in case rx interrupt occurs before get_dt call

				if(delta_t > inst->durationUwbCommTimeout_ms)
				{
					info->connectionType = UWB_LIST_INACTIVE;
					this->free_slots(info);
					setInactive = TRUE;
					updateINF = TRUE;
					info->lastCommNeighbor = 0;
					info->lastCommHidden = 0;
					info->lastCommTwiceHidden = 0;
				}

				break;
			}
			case UWB_LIST_INACTIVE:
			{
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
	//re-optimize our TDMA assignments and repopulate the inf message
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


static struct TDMAHandler new(uint64 slot_duration){
	struct TDMAHandler ret = {};

	ret.slot_transition = &slot_transition;
	ret.frame_sync = &frame_sync;
	ret.update_frame_start = &update_frame_start;
	ret.tx_sync_msg = &tx_sync_msg;
	ret.tx_select  = &tx_select;
	ret.check_blink  = &check_blink;

	ret.populate_inf_msg = &populate_inf_msg;
	ret.update_inf_tsfs = &update_inf_tsfs;
	ret.process_inf_msg = &process_inf_msg;

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


	ret.deconflict_slot_assignments = &deconflict_slot_assignments;
	ret.deconflict_uwb_pair = &deconflict_uwb_pair;
	ret.deconflict_slot_pair = &deconflict_slot_pair;
	ret.self_conflict = &self_conflict;

	ret.slotDuration_us = slot_duration;
	ret.slotDuration_ms = slot_duration/1000 + (slot_duration%1000 == 0 ? 0 : 1);

	uint64 time_now_us = portGetTickCntMicro();
	uint32 time_now = portGetTickCnt();

	ret.maxFramelength = (uint8)MIN_FRAMELENGTH;
	while(ret.maxFramelength < (uint8)UWB_LIST_SIZE + 1)
	{
		ret.maxFramelength *= 2;
	}

	for(int i = 0; i < UWB_LIST_SIZE; i++)
	{
		ret.uwbListTDMAInfo[i].framelength = (uint8)MIN_FRAMELENGTH;
		ret.uwbListTDMAInfo[i].slots = NULL;
		ret.uwbListTDMAInfo[i].slotsLength = 0;
		ret.uwbListTDMAInfo[i].frameStartTime = time_now_us;
		ret.uwbListTDMAInfo[i].connectionType = UWB_LIST_INACTIVE;
		ret.uwbListTDMAInfo[i].lastCommNeighbor = 0;
		ret.uwbListTDMAInfo[i].lastCommHidden = 0;
		ret.uwbListTDMAInfo[i].lastCommTwiceHidden = 0;
		ret.uwbListTDMAInfo[i].lastRange = time_now;
	}
	ret.uwbListTDMAInfo[0].connectionType = UWB_LIST_SELF;

    ret.lastSlotStartTime64 = time_now_us;
    ret.infSentThisSlot = FALSE;
    ret.firstPollSentThisSlot = FALSE;
    ret.firstPollResponse = FALSE;
    ret.firstPollComplete = FALSE;
    ret.secondPollSentThisSlot = FALSE;
    ret.nthOldest = 1;
    ret.nthOldestPlus = 2;
    ret.slotStartDelay_us = SLOT_START_BUFFER_US;
    ret.frameSyncThreshold_us = ret.slotStartDelay_us;
    ret.infMessageLength = 0;

    ret.enter_discovery_mode(&ret);
    ret.collectInfDuration = ret.maxFramelength*ret.slotDuration_ms;
	ret.waitInfDuration = ret.collectInfDuration;
	ret.blinkPeriodRand = (uint32)rand()%BLINK_PERIOD_RAND_MS;

	return ret;
}

const struct TDMAHandlerClass TDMAHandler={.new=&new};


