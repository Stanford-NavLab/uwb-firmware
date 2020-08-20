#ifndef TDMA_HANDLER_H_
#define TDMA_HANDLER_H_

#include "llist.h"
#include "deca_types.h"
#include "instance.h"

typedef struct
{
	uint32 time;      			//time the node is scheduled to be used.
	uint32 duration;            //expected time the node event will take before completion
	uint8 index;            	//the node index

} tdma_timing_node;


struct TDMASHandler
{

	llist list;					//list of tdma_timing_nodes

	//instance variables
	instance_data_t *inst;
//	uint8 uwb_max_list_length;
//	uint8 *uwb_list_length;

//	uint8 (*uwb_list)[][8];             //pointer to uwbList
//	uint8 *uwb_timeout_list;          //pointer to uwbTimeout
	//might need uwbliststate pointer here


    //TDMA class variables
    uint8 framelength;
	uint8 maxFramelength;
	uint16 *slotAssignments; //TODO may need two slot assignments!
	uint8 *uwbFramelengths; //[UWB_LIST_SIZE]
	uint32 frameStartTime; //TODO how to sync
	uint32 lastSlotStartTime;
	uint32 slotDuration;   //TODO 30 ms?
	uint32 discoveryStartTime; //time that we started listening for other UWBs

	//discovery variables
//	uint32 blink_period;
	uint32 last_blink_time;	   //timestamp of most recent blink


    //class functions
	void (*set_slot)(struct TDMAHandler *this);
    bool (*add_node)(struct TDMAHandler *this, tdma_timing_node *node);

};


extern const struct TDMAHandlerClass
{
//	struct TDMAScheduler (*new)(float blink_frequency, uint32 blink_duration, uint32 range_duration, uint8 (*uwb_list)[][8], uint8 *uwb_list_length, uint8 uwb_max_list_length, uint8 uwb_timeout_list[]);
	struct TDMAHandler (*new)(instance_data_t *inst);

} TDMAHandler;


#endif
