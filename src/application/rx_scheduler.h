#include "llist.h"
#include "deca_types.h"


typedef enum rx_node_type
{
    RX_ACCEPT,                    	//0
    RX_REJECT                	  	//1
} RX_NODE_TYPE;

typedef enum rx_message_type
{
    RX_BLINK,                   	//0
    RX_POLL                	  		//1
} RX_MESSAGE_TYPE;


typedef struct
{
	uint32 time;      			//time the node is scheduled to be used.
	uint32 duration;            //expected time the node event will take before completion
	uint8 index;            	//the node index
	RX_NODE_TYPE type;

} rx_timing_node;


struct RXScheduler
{

	llist list;					//list of rx_timing_nodes

	uint8 (*uwb_list)[][8];             //pointer to uwbList
	uint8 *uwb_timeout_list;          //pointer to uwbTimeout
	uint8 *time_till_next_reported_list; //pointer to time_next_rx_accept_reported
	uint8 uwb_max_list_length;
	uint8 *uwb_list_length;

	uint32 blink_frequency;
	uint32 blink_period;
	uint32 blink_duration;
	uint32 range_duration;
// TODO add blink reserve window?


//	uint32 time_reject_all;
	uint32 accepted_node_end_time;
	uint8 last_accept_index;

	uint32 time_next_rx_accept;



    bool (*rx_accept)(struct RXScheduler *this, uint8 index, RX_MESSAGE_TYPE message_type);
    bool (*add_node)(struct RXScheduler *this, rx_timing_node *node);
    bool (*reschedule_node)(struct RXScheduler *this, rx_timing_node *node);
    bool (*past_accepted_end_Time)(struct RXScheduler *this);
    void (*remove_expired)(struct RXScheduler *this, uint32 time_now_offset, uint32 offset);
    uint32 (*get_time_till_next_rx_accept)(struct RXScheduler *this);

};


extern const struct RXSchedulerClass
{
	struct RXScheduler (*new)(float blink_frequency, uint32 blink_duration, uint32 range_duration, uint8 (*uwb_list)[][8], uint8 *uwb_list_length, uint8 uwb_max_list_length, uint8 *uwb_timeout_list, uint8 *time_till_next_reported_list);

} RXScheduler;


