#include "tx_scheduler.h"
#include "port.h"


extern void usb_run(void);
extern void send_usbmessage(uint8*, int);

//private methods
int tx_node_compare(llist_node first, llist_node second)
{
    tx_timing_node *first_node = (tx_timing_node *)first;
    tx_timing_node *second_node = (tx_timing_node *)second;

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


bool tx_node_equal(llist_node first, llist_node second)
{
    if(first == second){return true;}

    return false;
}


//class methods

//return index of uwb to initiate comms with
//index 0-244 for uwb, 255 for blink, -1 for none
static int tx_select(struct TXScheduler *this)
{
	uint32 time_now = portGetTickCnt();
	uint32 offset = 0;

	//force number wrapping in case the timestamp is getting close to wrapping
	if(time_now > 4000000000){
		offset = 1000000000;
//		time_now += offset;
	}
	uint32 time_now_offset = time_now + offset;

//	if(time_now_offset < this->time_reject_select + offset){
//		return -1;
//	}

//	uint8 debug_msg[100];
//	int n = sprintf((char*)&debug_msg[0], "[0] %u [1] %u", this->uwb_timeout_list[0], this->uwb_timeout_list[1]);
//	send_usbmessage(&debug_msg[0], n);
//	usb_run();

	//start by removing expired nodes
	while(true){

		if(llist_size(this->list) == 0){break;}

		tx_timing_node *node = llist_get_head(this->list);

		if(time_now_offset > node->time + node->duration + offset){ //expired
			llist_delete_node(this->list, node, true, NULL);
		}
		else{
			break;
		}
	}

	//check if it's time to blink
	if(time_now_offset > this->last_blink_time + this->blink_period + offset){
		this->last_blink_time = time_now;
		return 255;
	}

	//don't range if it will carry over into blink time
	if(time_now_offset + this->range_duration > this->last_blink_time + this->blink_period + offset){
		return -1;
	}

	//TODO could have a set flag for this like uwb_active...
	int cooldown_arr[*this->uwb_list_length];
	memset(cooldown_arr, 0, sizeof(cooldown_arr));

	tx_timing_node *node = llist_get_head(this->list);
	//dont select if it starts in or ends in a cooldown period...
	while(node != NULL){
		if(cooldown_arr[node->index] == 0)
		{
			if((time_now_offset > node->time + offset && time_now_offset < node->time + node->duration + offset) ||
			   (time_now_offset + this->range_duration > node->time + offset && time_now_offset + this->range_duration < node->time + node->duration + offset) )
			{
				cooldown_arr[node->index] = 1;
			}
		}
		node = llist_get_next(this->list, node);
	}


	uint8 uwb_list_len = *this->uwb_list_length;
	uint8 num_checked = 0;
	uint8 index = (this->last_select_index + 1)%uwb_list_len;
	while(num_checked < uwb_list_len){
		if(cooldown_arr[index] == 0 && this->uwb_timeout_list[index] == 0){
			this->last_select_index = index;
			this->time_reject_select = time_now + this->range_duration;
			this->expected_tx_end_time = time_now + this->range_duration;


        	//add cooldown to prevent polling same tag too rapidly (can block reception of blinks from other tags)
        	tx_timing_node *node = malloc(sizeof(tx_timing_node));
			node->index = index;
			node->duration = this->poll_frequency_period;
			node->time = time_now;

			this->add_node(this, node);

			return index;
		}

		index = (index + 1)%uwb_list_len;
		num_checked++;
	}

	return -1;
}

static bool add_node(struct TXScheduler *this, tx_timing_node *node) //TODO handle unsuccessful add
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

		tx_timing_node *pos_node = llist_get_head(this->list);
		while(pos_node != NULL){

			if(node->time + offset < pos_node->time + offset){
				return llist_insert_node(this->list, node, pos_node, ADD_NODE_BEFORE);
			}

			pos_node = llist_get_next(this->list, node);
		}

		return llist_add_node(this->list, node, ADD_NODE_REAR);
	}
}

static struct TXScheduler new(float blink_frequency, uint32 blink_duration, uint32 range_duration, uint8 (*uwb_list)[][8], uint8 *uwb_list_length, uint8 uwb_max_list_length, uint8 uwb_timeout_list[]){
	struct TXScheduler ret = {.uwb_list = uwb_list, .uwb_list_length = uwb_list_length, .uwb_max_list_length=uwb_max_list_length, .uwb_timeout_list = uwb_timeout_list};
	ret.list = llist_create(tx_node_compare, tx_node_equal, 0);
	ret.blink_frequency = blink_frequency;
	ret.blink_duration = blink_duration;
	ret.range_duration = range_duration;
	ret.blink_period = (int)(1.0/blink_frequency);
	ret.tx_select = &tx_select;
	ret.add_node = &add_node;
	ret.last_blink_time = 0;
	ret.last_select_index = 0;
	ret.time_reject_select = 0;
	ret.poll_frequency = .05;
	ret.poll_frequency_period = (uint32)(1.0/ret.poll_frequency);
	return ret;
}

const struct TXSchedulerClass TXScheduler={.new=&new};


