#include "llist.h"
#include "deca_types.h"



///////////////////////////////////////////////////////
// Timed Scheduler Base Class
///////////////////////////////////////////////////////

typedef struct
{
    // uint8 index;            	//the node index
	uint32 time;      			//time the node is scheduled to be used.
    // uint32 frequency;           //how often the node should be timed. if 0, append to end of list after calling.
	uint32 duration;            //expected time the node event will take before completion

} timing_node ;



struct TimedScheduler
{
    llist list;

    uint32 blink_frequency;
    uint32 blink_duration;
    uint32 range_duration;

    // timing_node *(*select_node)(struct TimedScheduler *this);
    // bool (*add_node)(struct TimedScheduler *this, timing_node *node);
    // bool (*remove_node)(struct TimedScheduler *this, timing_node *node);
};

extern const struct TimedSchedulerClass
{
	struct TimedScheduler (*new)();
    
} TimedScheduler;

///////////////////////////////////////////////////////
// TX Scheduler Class
///////////////////////////////////////////////////////

typedef struct
{
    timing_node base_node;      //contains timing and duration
    uint8 index;            	//the node index

} tx_timing_node;


struct TXScheduler
{
    // llist list;
    // use list to store uwbs on cooldown
    struct TimedScheduler TimedScheduler;

    uint32 last_blink_time;      //timestamp of most recent blink

    uint8 *uwb_list;             //pointer to uwbList
    uint8 *uwb_timeout;          //pointer to uwbTimeout
    uint8 last_select_index;     //index of most recently selected uwb

    int *(*tx_select)(struct TXScheduler *this);
    bool (*add_node)(struct TXScheduler *this, tx_timing_node *node);
    bool (*remove_node)(struct TXScheduler *this, tx_timing_node *node);
    void (*reschedule_node)(struct TXScheduler *this, tx_timing_node *node);
};

extern const struct TXSchedulerClass
{
	struct TXScheduler (*new)(uint8 *uwb_list, uint8 *uwb_timeout_list);
    
} TXScheduler;




