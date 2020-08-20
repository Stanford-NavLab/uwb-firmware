//#include "comm_scheduler.h"
//#include "port.h"
//
////private mehtods
////int node_compare(llist_node first, llist_node second)
////{
////    timing_node *first_node = (timing_node *)first;
////    timing_node *second_node = (timing_node *)second;
////
////    if(first_node->time == second_node->time)
////    {
////        return 0;
////    }
////    else if(first_node->time > second_node->time)
////    {
////        return 1;
////    }
////    else
////    {
////        return -1;
////    }
////	return 0;
////}
////
////
////bool node_equal(llist_node first, llist_node second)
////{
////    if(first == second){return true;}
////
////    return false;
////}
//
//
////class methods
//
//static struct CommScheduler new(float blink_frequency, uint32 blink_duration, uint32 range_duration){
//    return (struct CommScheduler){
////        .list = llist_create(node_compare, node_equal, 0),
//		.blink_frequency = blink_frequency,
//		.blink_duration = blink_duration,
//		.range_duration = range_duration,
//		.blink_period = (int)(1.0/blink_frequency)
//    };
//}
//
//const struct CommSchedulerClass CommScheduler={.new=&new};
