//#include "llist.h"
//#include "deca_types.h"
//
////test 123
//
//typedef struct
//{
//    // uint8 index;            	//the node index
//	uint32 time;      			//time the node is scheduled to be used.
//    // uint32 frequency;           //how often the node should be timed. if 0, append to end of list after calling.
//	uint32 duration;            //expected time the node event will take before completion
//
//} timing_node ;
//
//
//
//struct CommScheduler
//{
//    llist list;
//
//    uint32 blink_frequency;
//    uint32 blink_duration;
//    uint32 range_duration;
//
//    // timing_node *(*select_node)(struct TimedScheduler *this);
//    // bool (*add_node)(struct TimedScheduler *this, timing_node *node);
//    // bool (*remove_node)(struct TimedScheduler *this, timing_node *node);
//};
//
//extern const struct CommSchedulerClass
//{
//	struct CommScheduler (*new)(uint32 frequency_blink, uint32 duration_blink, uint32 duration_range);
//
//} CommScheduler;
