#include "llist.h"
#include "deca_types.h"

typedef struct
{
	uint32 time;      			//time the node is scheduled to be used.
	uint32 duration;            //expected time the node event will take before completion
	uint8 index;            	//the node index

} tx_timing_node;


struct TXScheduler
{

	llist list;					//list of tx_timing_nodes





	uint8 uwb_max_list_length;
	uint8 *uwb_list_length;

	uint8 (*uwb_list)[][8];             //pointer to uwbList
	uint8 *uwb_timeout_list;          //pointer to uwbTimeout

	float blink_frequency;
	uint32 blink_period;
	uint32 blink_duration;
	uint32 range_duration;


	float poll_frequency;
	uint32 poll_frequency_period;

    uint32 last_blink_time;      //timestamp of most recent blink
//    uint32 accepted_node_end_time;
    uint8 last_select_index;     //index of most recently selected uwb
    uint32 expected_tx_end_time;
    uint32 time_reject_select;

    int  (*tx_select)(struct TXScheduler *this);
    bool (*add_node)(struct TXScheduler *this, tx_timing_node *node);

};


extern const struct TXSchedulerClass
{
	struct TXScheduler (*new)(float blink_frequency, uint32 blink_duration, uint32 range_duration, uint8 (*uwb_list)[][8], uint8 *uwb_list_length, uint8 uwb_max_list_length, uint8 uwb_timeout_list[]);

} TXScheduler;


