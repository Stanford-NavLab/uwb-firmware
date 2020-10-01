#ifndef TDMA_HANDLER_H_
#define TDMA_HANDLER_H_

//#include "llist.h"
#include "deca_types.h"
#include "application_definitions.h"

struct TDMAInfo
{
	uint8 uwbIndex; //TODO remove if unused
	uint8 framelength;
	uint8 slotsLength;
	uint8 *slots;
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
	FS_COLLECT
//	FS_REBASE //TODO remove
}
FRAME_SYNC_MODE;

struct TDMAHandler
{

    //TDMA class variables
    uint8 maxFramelength;
	uint8 slotAssingmentSelfIndex;

	struct TDMAInfo uwbListTDMAInfo[UWB_LIST_SIZE];

	//TODO use smaller data types where possible

	uint64 lastINFtx;
	uint64 lastINFrx;


	uint64 uwbFrameStartTimes64[UWB_LIST_SIZE];
	uint64 lastFST;
	uint64 lastSlotStartTime64;
	uint32 slotDuration_ms;   //TODO make variable in duration based on UWB_LIST_SIZE
	uint32 slotDuration_us;   //TODO make variable in duration based on UWB_LIST_SIZE
	bool infSentThisSlot;
	bool pollSentThisSlot;
	bool rebase_pending;
	bool rebase_tx;
	uint64 rebase_frameStartTime64; //TODO rename 64 to us!

	//TODO can probably use a smaller data type...
	uint64 slotStartDelay_us; //time between slot start and transmission within that slot

	//discovery variables
	DISCOVERY_MODE discovery_mode;
	uint32 last_blink_time;	   //timestamp of most recent blink
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
	void (*frame_sync)(struct TDMAHandler *this, event_data_t *dw_event, uint8 *messageData, uint8 srcIndex, FRAME_SYNC_MODE mode);
	void (*update_inf_tsfs)(struct TDMAHandler *this);
	bool (*tx_select)(struct TDMAHandler *this);
    bool (*check_blink)(struct TDMAHandler *this);
    void (*populate_inf_msg)(struct TDMAHandler *this, uint8 inf_msg_type);
    bool (*process_inf_msg)(struct TDMAHandler *this, uint8 *messageData, uint8 srcIndex, INF_PROCESS_MODE mode);
    bool (*check_tdma_diff)(struct TDMAHandler *this, uint8 *messageData, uint8 *srcAddr);
    bool (*poll_delay)(struct TDMAHandler *this, uint32 time_now_offset, uint32 offset);
    void (*enter_discovery_mode)(struct TDMAHandler *this);
    void (*set_discovery_mode)(struct TDMAHandler *this, DISCOVERY_MODE mode, uint32 time_now);
    void (*check_discovery_mode_expiration)(struct TDMAHandler *this);
    void (*usb_dump_tdma)(struct TDMAHandler *this);

    //TODO left off here. updating messageData as well as
    //TODO revisit anything that works with messageData!!!
    bool (*slot_assigned)(struct TDMAInfo *info, uint8 slot);
    bool (*assign_slot)(struct TDMAInfo *info, uint8 slot, bool safeAssign);
    void (*free_slot)(struct TDMAInfo *info, uint8 slot);
    void (*free_slots)(struct TDMAInfo *info);
    void (*uwblist_free_slots)(struct TDMAHandler *this, uint8 uwb_index);
	void (*tdma_free_all_slots)(struct TDMAHandler *this);
	void (*find_assign_slot)(struct TDMAHandler *this);
	void (*build_new_network)(struct TDMAHandler *this);


	//run through all uwb pairs
    bool (*deconflict_slot_assignments)(struct TDMAHandler *this); //TODO implement
    //run through each slot of two uwbs
    bool (*deconflict_uwb_pair)(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b);
    //deconflict two specific slots
    void (*deconflict_slot_pair)(struct TDMAHandler *this, struct TDMAInfo *info_a, struct TDMAInfo *info_b, uint8 slot_idx_a, uint8 slot_idx_b);
    bool (*self_conflict)(struct TDMAHandler *this);
};


extern const struct TDMAHandlerClass
{
	struct TDMAHandler (*new)();

} TDMAHandler;


#endif
