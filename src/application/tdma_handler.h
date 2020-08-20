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
	FS_AVERAGE
}
FRAME_SYNC_MODE;

struct TDMAHandler
{

    //TDMA class variables
    uint8 maxFramelength;
	uint8 slotAssingmentSelfIndex;

//	struct TDMAInfo myTDMAInfo;

	struct TDMAInfo uwbListTDMAInfo[UWB_LIST_SIZE];

	uint32 uwbFrameStartTimes[UWB_LIST_SIZE]; //TODO propagate the use of this
//	uint32 frameStartTime;
	uint32 lastSlotStartTime;
	uint32 slotDuration;   //TODO make variable in duration based on UWB_LIST_SIZE
	bool infPollSentThisSlot;

	uint32 slotStartDelay; //time between slot start and transmission within that slot

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

//	bool waitForRngInit;
//	bool waitForInf;
//	uint32 waitForInfStart;
//	uint32 waitForRngInitStart;

    //class functions
	bool (*slot_transition)(struct TDMAHandler *this);
//	void (*frame_sync)(struct TDMAHandler *this, uint8 *messageData, uint16 rxLength, uint8 srcIndex, FRAME_SYNC_MODE mode);
	void (*frame_sync)(struct TDMAHandler *this, event_data_t *dw_event, uint8 *messageData, uint8 srcIndex, FRAME_SYNC_MODE mode);
	void (*update_inf_tsfs)(struct TDMAHandler *this);
	bool (*tx_select)(struct TDMAHandler *this);
//	bool (*rx_accept)(struct TDMAHandler *this, uint8 uwb_index, uint8 *rxd_event, uint8 *msgu, uint8 fcode_index);
    bool (*check_blink)(struct TDMAHandler *this);
//    void (*rebuild_slot_assignments)(struct TDMAHandler *this);
    void (*populate_inf_msg)(struct TDMAHandler *this, uint8 inf_msg_type);
//    void (*process_inf_msg)(struct TDMAHandler *this); //TODO implement
    bool (*process_inf_msg)(struct TDMAHandler *this, uint8 *messageData, uint8 srcIndex, INF_PROCESS_MODE mode);
    bool (*check_tdma_diff)(struct TDMAHandler *this, uint8 *messageData, uint8 *srcAddr);
//    void (*populate_sug_msg)(struct TDMAHandler *this);//TODO remove this
//    void (*process_sug_msg)(struct TDMAHandler *this, uint8 *messageData, uint8 *srcAddr);//TODO remove this
    bool (*poll_delay)(struct TDMAHandler *this, uint32 time_now_offset, uint32 offset);
    void (*enter_discovery_mode)(struct TDMAHandler *this);
    void (*set_discovery_mode)(struct TDMAHandler *this, DISCOVERY_MODE mode, uint32 time_now);
    void (*check_discovery_mode_expiration)(struct TDMAHandler *this);
    void (*usb_dump_tdma)(struct TDMAHandler *this);

    //TODO left off here. updating messageData as well as
    //TODO revisit anything that works with messageData!!!
    bool (*slot_assigned)(struct TDMAInfo *info, uint8 slot);
    bool (*assign_slot)(struct TDMAInfo *info, uint8 slot);
    void (*free_slot)(struct TDMAInfo *info, uint8 slot);
    void (*free_slots)(struct TDMAInfo *info);
    void (*uwblist_free_slots)(struct TDMAHandler *this, uint8 uwb_index);
	void (*tdma_free_all_slots)(struct TDMAHandler *this);
	void (*find_assign_slot)(struct TDMAHandler *this);
	void (*build_new_network)(struct TDMAHandler *this);

//    void (*uwblist_collect_slot_assignment)(struct TDMAHandler *this, uint8 uwb_index, uint8 slot_index);

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
