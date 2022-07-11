#ifndef TDMA_HANDLER_H_
#define TDMA_HANDLER_H_

//#include "llist.h"
#include "deca_types.h"
#include "application_definitions.h"

struct TDMAInfo
{
	uint8 connectionType;       //UWB_LIST_SELF, UWB_LIST_NEIGHBOR, UWB_LIST_HIDDEN, UWB_LIST_INACTIVE
	uint32 lastCommNeighbor;	//milliseconds
	uint32 lastCommHidden;		//milliseconds
	uint32 lastCommTwiceHidden;		//milliseconds
	uint32 lastRange;			//milliseconds
	uint64 frameStartTime;		//microseconds
	uint8 framelength;
	uint8 largestFramelength;
	uint8 slotsLength;
	uint8 *slots;

	uint8 connected_nodes[UWB_LIST_SIZE];
	uint8 connected_connection_types[UWB_LIST_SIZE];
};

typedef enum inf_process_mode
{
	CLEAR_ALL_COPY,
	CLEAR_LISTED_COPY,
	COPY
}
INF_PROCESS_MODE;

typedef enum frame_sync_mode
{
	FS_ADOPT,
	FS_AVERAGE,
	FS_COLLECT,
	FS_EVAL
}
FRAME_SYNC_MODE;

struct TDMAHandler
{
    //TDMA class variables
    uint8 maxFramelength;
	uint8 slotAssingmentSelfIndex;

	struct TDMAInfo uwbListTDMAInfo[UWB_LIST_SIZE];

	uint64 lastSlotStartTime64;
	uint32 slotDuration_ms;
	uint32 slotDuration_us;
	bool infSentThisSlot;
	bool firstPollSentThisSlot;
	bool firstPollResponse;
	bool firstPollComplete;
	bool secondPollSentThisSlot;
	uint8 nthOldest;
//	uint8 nthOldestPlus; TODO
	bool reassignOnModifiedTDMA;
	bool reassigSlots;
	bool tdmaIsDirty;

	uint64 slotStartDelay_us; //time between slot start and transmission within that slot
	uint64 frameSyncThreshold_us;

	//discovery variables
	DISCOVERY_MODE discovery_mode;
	uint32 last_blink_time;	   //timestamp of most recent blink
	uint32 blinkPeriodRand;		//random number used to vary blink message transmission interval
	uint32 discoveryStartTime; //time that we started listening for other UWBs
	uint32 discovery_mode_start_time;
	uint32 discovery_mode_duration;
	bool discovery_mode_expires;
	uint32 collectInfStartTime;
	uint32 collectInfDuration;
	uint32 waitInfDuration;

	uint16 infMessageLength;

    //class functions
	bool (*slot_transition)(struct TDMAHandler *this);
	void (*frame_sync)(struct TDMAHandler *this, event_data_t *dw_event, uint8 framelength, uint64 timeSinceFrameStart_us, uint8 srcIndex, FRAME_SYNC_MODE mode);
	uint64 (*update_frame_start)(struct TDMAHandler *this);
	bool (*tx_sync_msg)(struct TDMAHandler *this);
	void (*update_inf_tsfs)(struct TDMAHandler *this);
	uint8 (*get_largest_framelength)(struct TDMAHandler *this);
	bool (*tx_select)(struct TDMAHandler *this);
    bool (*check_blink)(struct TDMAHandler *this);
    void (*populate_inf_msg)(struct TDMAHandler *this, uint8 inf_msg_type);
    bool (*process_inf_msg)(struct TDMAHandler *this, uint8 *messageData, uint8 srcIndex, INF_PROCESS_MODE mode);
    bool (*poll_delay)(struct TDMAHandler *this, uint32 time_now_offset, uint32 offset);
    void (*enter_discovery_mode)(struct TDMAHandler *this);
    void (*set_discovery_mode)(struct TDMAHandler *this, DISCOVERY_MODE mode, uint32 time_now);
    void (*check_discovery_mode_expiration)(struct TDMAHandler *this);
    bool (*check_timeouts)(struct TDMAHandler *this);
    void (*remove_uwbinfo)(struct TDMAHandler *this, uint8 uwb_index);
    bool (*slot_assigned)(struct TDMAInfo *info, uint8 slot);
    bool (*assign_slot)(struct TDMAInfo *info, uint8 slot, bool safeAssign);
    void (*free_slot)(struct TDMAInfo *info, uint8 slot);
    void (*free_slots)(struct TDMAInfo *info);
    void (*uwblist_free_slots)(struct TDMAHandler *this, uint8 uwb_index);
	void (*tdma_free_all_slots)(struct TDMAHandler *this);
	void (*find_assign_slot)(struct TDMAHandler *this);
	void (*build_new_network)(struct TDMAHandler *this);


	//run through all uwb pairs
    bool (*deconflict_slot_assignments)(struct TDMAHandler *this);
    //run through each slot of two uwbs
    bool (*deconflict_uwb_pair)(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b, uint8 *uwbAddr_a, uint8 *uwbAddr_b);
    //deconflict two specific slots
    void (*deconflict_slot_pair)(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b, uint8 slot_idx_a, uint8 slot_idx_b, uint8 *uwbAddr_a, uint8 *uwbAddr_b);
    bool (*self_conflict)(struct TDMAHandler *this);
    bool (*check_contention)(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b);
};


extern const struct TDMAHandlerClass
{
	struct TDMAHandler (*new)(uint64 slot_duration);

} TDMAHandler;


#endif
