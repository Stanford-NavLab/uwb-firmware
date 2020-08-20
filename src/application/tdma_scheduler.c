#include "tdma_handler.h"
#include "port.h"


extern void usb_run(void);
extern void send_usbmessage(uint8*, int);

//private methods
int tdma_node_compare(llist_node first, llist_node second)
{
    tdma_timing_node *first_node = (tdma_timing_node *)first;
    tdma_timing_node *second_node = (tdma_timing_node *)second;

	if(first_node->time == second_node->time)
	{
        return 0;
    }
	else if(first_node->time > second_node->time)
	{
        return 1;
    }
    else
    {
        return -1;
    }
	return 0;
}


bool tdma_node_equal(llist_node first, llist_node second)
{
    if(first == second){return true;}

    return false;
}


//class methods
static void set_slot(struct TDMAHandler *this)
{
//	uint32 time_now = portGetTickCnt();
//	uint32 offset = 0;
//
//	//force number wrapping in case the timestamp is getting close to wrapping
//	if(time_now > 4000000000){
//		offset = 1000000000;
////		time_now += offset;
//	}
//	uint32 time_now_offset = time_now + offset;
//
////	if(time_now_offset < this->time_reject_select + offset){
////		return -1;
////	}
//
////	uint8 debug_msg[100];
////	int n = sprintf((char*)&debug_msg[0], "[0] %u [1] %u", this->uwb_timeout_list[0], this->uwb_timeout_list[1]);
////	send_usbmessage(&debug_msg[0], n);
////	usb_run();
//
//	//start by removing expired nodes
//	while(true){
//
//		if(llist_size(this->list) == 0){break;}
//
//		tdma_timing_node *node = llist_get_head(this->list);
//
//		if(time_now_offset > node->time + node->duration + offset){ //expired
//			llist_delete_node(this->list, node, true, NULL);
//		}
//		else{
//			break;
//		}
//	}
//
//	//check if it's time to blink
//	if(time_now_offset > this->last_blink_time + this->blink_period + offset){
//		this->last_blink_time = time_now;
//		return 255;
//	}
//
//	//don't range if it will carry over into blink time
//	if(time_now_offset + this->range_duration > this->last_blink_time + this->blink_period + offset){
//		return -1;
//	}
//
//	//TODO could have a set flag for this like uwb_active...
//	int cooldown_arr[*this->uwb_list_length];
//	memset(cooldown_arr, 0, sizeof(cooldown_arr));
//
//	tx_timing_node *node = llist_get_head(this->list);
//	//dont select if it starts in or ends in a cooldown period...
//	while(node != NULL){
//		if(cooldown_arr[node->index] == 0)
//		{
//			if((time_now_offset > node->time + offset && time_now_offset < node->time + node->duration + offset) ||
//			   (time_now_offset + this->range_duration > node->time + offset && time_now_offset + this->range_duration < node->time + node->duration + offset) )
//			{
//				cooldown_arr[node->index] = 1;
//			}
//		}
//		node = llist_get_next(this->list, node);
//	}
//
//
//	uint8 uwb_list_len = *this->uwb_list_length;
//	uint8 num_checked = 0;
//	uint8 index = (this->last_select_index + 1)%uwb_list_len;
//	while(num_checked < uwb_list_len){
//		if(cooldown_arr[index] == 0 && this->uwb_timeout_list[index] == 0){
//			this->last_select_index = index;
//			this->time_reject_select = time_now + this->range_duration;
//			this->expected_tx_end_time = time_now + this->range_duration;
//
//
//        	//add cooldown to prevent polling same tag too rapidly (can block reception of blinks from other tags)
//        	tx_timing_node *node = malloc(sizeof(tx_timing_node));
//			node->index = index;
//			node->duration = this->poll_frequency_period;
//			node->time = time_now;
//
//			this->add_node(this, node);
//
//			return index;
//		}
//
//		index = (index + 1)%uwb_list_len;
//		num_checked++;
//	}

	return;
}


static bool add_node(struct TDMAHandler *this, tdma_timing_node *node) //TODO handle unsuccessful add
{
//	char debug_msg[100];
//	n = sprintf((char*)&debug_msg[0], "node added, index %i", *node->index);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

	if(llist_is_empty(this->list))
	{
		return llist_add_node(this->list, node, ADD_NODE_REAR);
	}
	else{
		uint32 offset = 0;

		//force number wrapping in case the timestamp is getting close to wrapping
		if(node->time > 4000000000){
			offset = 1000000000;
		}

		tdma_timing_node *pos_node = llist_get_head(this->list);
		while(pos_node != NULL){

			if(node->time + offset < pos_node->time + offset){
				return llist_insert_node(this->list, node, pos_node, ADD_NODE_BEFORE);
			}

			pos_node = llist_get_next(this->list, node);
		}

		return llist_add_node(this->list, node, ADD_NODE_REAR);
	}
}

static struct TDMAHandler new(instance_data_t *inst){
	struct TDMAHandler ret = {};
//	ret.blink_period = (int)(1.0/(int)BLINK_PERIOD_MS);
//	ret.uwb_list = uwb_list;
//	ret.uwb_list_length = uwb_list_length;
//	ret.uwb_max_list_length = uwb_max_list_length;
//	ret.uwb_timeout_list = uwb_timeout_list; //might need uwbliststate pointer here

	ret.list = llist_create(tdma_node_compare, tdma_node_equal, 0);

	ret.add_node = &add_node;
	ret.set_slot = &set_slot;

	uint32 time_now = portGetTickCnt();

	ret.last_blink_time = time_now;
	ret.framelength = MIN_FRAMELENGTH;
	ret.maxFramelength = min_framelength;
	while(ret.maxFramelength < ret.uwb_max_list_length + 1)
	{
		ret.maxFramelength *= 2;
	}
    ret.slotDuration = 30; //TODO use a #define or something?

    ret.slotAssignments = malloc(sizeof(uint16)*ret.maxFramelength);
    for(int i = 0; i < ret.maxFramelength; i++)
    {
    	uint16 blank = 0x0000;
    	memcpy(&ret.slotAssignments[i], &blank, 2);
    }

    ret.uwbFramelengths = malloc(sizeof(uint8)*ret.maxFramelength);
    for(int i = 0; i < 	ret.uwb_max_list_length; i++)
    {
    	ret.uwbFramelengths[i] = 0;
    }

    ret.frameStartTime = time_now;
    ret.lastSlotStartTime = time_now;
    ret.discoveryStartTime = time_now;
    ret.last_blink_time = time_now;

	return ret;
}

const struct TDMAHandlerClass TDMASHandler={.new=&new};


