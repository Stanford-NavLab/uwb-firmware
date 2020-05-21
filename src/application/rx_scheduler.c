#include "rx_scheduler.h"
#include "port.h"


extern void usb_run(void);
extern void send_usbmessage(uint8*, int);

//private methods
int rx_node_compare(llist_node first, llist_node second)
{
    rx_timing_node *first_node = (rx_timing_node *)first;
    rx_timing_node *second_node = (rx_timing_node *)second;



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


//TODO might want to change to match addresses/index... ?
bool rx_node_equal(llist_node first, llist_node second)
{
    if(first == second){return true;}

    return false;
}


//class methods

//this function is only called when:
//	we RX a blink message from a TAG that isn't active in our list
//  we RX a poll message from a TAG
//index is the index of the TAG in our uwbList
//message_type is whether the message is a blink or a poll
//return whether or not to accept comms from the TAG
static bool rx_accept(struct RXScheduler *this, uint8 index, RX_MESSAGE_TYPE message_type)
{

	uint32 time_now = portGetTickCnt();
	uint32 offset = 0;

	//force number wrapping in case the timestamp is getting close to wrapping
	if(time_now > 4000000000){
		offset = 1000000000;
	}
	uint32 time_now_offset = time_now + offset;

	this->remove_expired(this, time_now_offset, offset);

	uint8 debug_msg[500];
	int n = 0;
	int n_char = 0;
	bool debug = true;
	if(debug){
		n = sprintf((char*)&debug_msg[0], "rx_accept, time_now %lu, index %u \n", time_now, index);
		n_char = n;
	}

	//don't allow comms if the duration reserved by the most recent ranging event has not passed
	//allow last_accept_index in case of RX error
	//allow blink messages through, finding new TAGs is the highest priority
	if(time_now_offset < this->accepted_node_end_time + offset && index != this->last_accept_index && message_type != RX_BLINK){
//	if(time_now_offset < this->accepted_node_end_time + offset && message_type != RX_BLINK){

		if(debug){
			n = sprintf((char*)&debug_msg[n_char], "rejected-too soon");
			n_char += n;
			send_usbmessage(&debug_msg[0], n_char);
			usb_run();
		}

		return false;
	}


	uint32 duration = this->range_duration;
	if(message_type == RX_BLINK){
		duration += this->blink_duration;
	}

	rx_timing_node *accept_node_match = NULL;
	uint8 num_accept = 0;

	rx_timing_node *node = llist_get_head(this->list);
	while(node != NULL){
		if(time_now_offset + duration < node->time + offset){
			break;
		}

		if((time_now_offset < node->time + node->duration + offset && time_now_offset > node->time + offset) ||
		   (time_now_offset + duration < node->time + node->duration + offset && time_now_offset + duration > node->time + offset)){

			if(node->type == RX_ACCEPT){
				num_accept++;
				if(index == node->index){
					if(debug){
						n = sprintf((char*)&debug_msg[n_char], "accept_node_match found");
						n_char += n;
					}
					accept_node_match = node;
				}
			}
			else if(node->type == RX_REJECT){
				if(node->index == index && message_type != RX_BLINK){
					//don't range if starts or finishes inside a matching REJECT
					if(debug){
						n = sprintf((char*)&debug_msg[n_char], "rejected-starts or finishes inside a matching REJECT");
						n_char += n;
						send_usbmessage(&debug_msg[0], n_char);
						usb_run();
					}

					return false;
				}
			}
		}

		node = llist_get_next(this->list, node);
	}

	if(num_accept == 0){

		if(debug){
			n = sprintf((char*)&debug_msg[n_char], "num_accept == 0");
			n_char += n;
			send_usbmessage(&debug_msg[0], n_char);
			usb_run();
		}

		this->last_accept_index = index;
		this->accepted_node_end_time = time_now + duration - 10; //TODO fix the 10. include TOF?

		return true;
	}


	if(accept_node_match != NULL || message_type == RX_BLINK){
		if(debug){
			n = sprintf((char*)&debug_msg[n_char], "accepted-RX_ACCEPT yes");
			n_char += n;
			send_usbmessage(&debug_msg[0], n_char);
			usb_run();
		}

		llist_delete_node(this->list, accept_node_match, true, NULL);

		this->last_accept_index = index;
		this->accepted_node_end_time = time_now + duration - 10; //TODO fix the 10. include TOF?

		return true;
	}
	else{

		if(debug){
			n = sprintf((char*)&debug_msg[n_char], "rejected-RX_ACCEPT no");
			n_char += n;
			send_usbmessage(&debug_msg[0], n_char);
			usb_run();
		}
		//don't range if starts of finishes inside a non-matching ACCEPT
		return false;
	}

//	//TODO could have a set flag for this like uwb_active...
//	int num_accept = 0;
//	int accept_arr[*this->uwb_list_length];
//	memset(accept_arr, -1, sizeof(accept_arr));
//
//	rx_timing_node *node = llist_get_head(this->list);
//	while(node != NULL){
//		if(time_now_offset + duration < node->time + offset){
//			break;
//		}
//
//		if((time_now_offset < node->time + node->duration + offset && time_now_offset > node->time + offset) ||
//		   (time_now_offset + duration < node->time + node->duration + offset && time_now_offset + duration > node->time + offset)){
//
//			if(node->type == RX_ACCEPT){
//				accept_arr[node->index] = 1;
//				num_accept++;
//			}
//			else if(node->type == RX_REJECT){
//				if(node->index == index){
//					//don't range if starts or finishes inside a matching REJECT
//					if(debug){
//						n = sprintf((char*)&debug_msg[n_char], "rejected-starts or finishes inside a matching REJECT");
//						n_char += n;
//						send_usbmessage(&debug_msg[0], n_char);
//						usb_run();
//					}
//
//					return false;
//				}
//			}
//		}
//
//		node = llist_get_next(this->list, node);
//	}
//
//	if(num_accept == 0){
//
//		if(debug){
//			n = sprintf((char*)&debug_msg[n_char], "accepted-num_accept==0");
//			n_char += n;
//			send_usbmessage(&debug_msg[0], n_char);
//			usb_run();
//		}
//
////		if(duration == this->range_duration){
////			//10 found experimentally. used to sync with expected tag poll time
//////			this->time_reject_all = time_now + duration - 10;//how to know if this is from a blink request...? //TODO maybe add comm_type?
////			this->accepted_node_end_time = time_now + duration - 10;
////		}
//
//		this->last_accept_index = index;
//		this->accepted_node_end_time = time_now + duration - 10;
//
//		return true;
//	}
//
//	if(accept_arr[index] == 1){ //TODO remove the associated node upon acceptance!!!?
//		if(debug){
//			n = sprintf((char*)&debug_msg[n_char], "accepted-RX_ACCEPT yes");
//			n_char += n;
//			send_usbmessage(&debug_msg[0], n_char);
//			usb_run();
//		}
//
//
//
//		return true;
//	}
//	else{
//		if(debug){
//			n = sprintf((char*)&debug_msg[n_char], "rejected-RX_ACCEPT no");
//			n_char += n;
//			send_usbmessage(&debug_msg[0], n_char);
//			usb_run();
//		}
//		//don't range if starts of finishes inside a non-matching ACCEPT
//		return false;
//	}
}


static bool past_accepted_end_Time(struct RXScheduler *this)
{
	uint32 time_now = portGetTickCnt();
	uint32 offset = 0;
	if(time_now > 4000000000){
		offset = 1000000000;
	}
	uint32 time_now_offset = time_now + offset;

	if(time_now_offset > this->accepted_node_end_time + offset){
		return true;
	}
	else{
		return false;
	}
}

static bool add_node(struct RXScheduler *this, rx_timing_node *node) //TODO handle unsuccessful add
{

//	uint32 time_now = portGetTickCnt();
//	if(node->type == RX_ACCEPT)
//	{
//		uint8 debug_msg[100];
//		int n = sprintf((char *)&debug_msg, "RX_ACCEPT-add_node, time_now %lu, duration %lu, index %d", time_now, node->duration, node->index);
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();
//	}
//	else if(node->type == RX_REJECT)
//	{
//		uint8 debug_msg[100];
//		int n = sprintf((char *)&debug_msg, "RX_REJECT-add_node, time_now %lu, index %u", time_now, node->index);
//		send_usbmessage(&debug_msg[0], n);
//		usb_run();
//	}


	uint32 time_now = portGetTickCnt();
	uint32 offset = 0;
	if(time_now > 4000000000){
		offset = 1000000000;
	}
	uint32 time_now_offset = time_now + offset;

	this->remove_expired(this, time_now_offset, offset);

	//TODO update time_till---still need to update upon expiration!?
	if(node->type == RX_ACCEPT){
		uint32 time_next = this->time_next_rx_accept;
		bool update = false;

		//TODO this could act funky with wrap arounds if its been a very long time since an RX_ACCEPT node has been added...
		if(time_next + offset > time_now_offset)
		{
			if(node->time + offset > time_now_offset && node->time + offset <  time_next + offset){
				update = true;
			}
		}
		else
		{
			if(node->time + offset > time_now_offset){
				update = true;
			}
		}

		if(update == true){
			this->time_next_rx_accept = node->time;
			for (int i = 0; i < *this->uwb_list_length; i++) {
				this->time_till_next_reported_list[i] = 0;
			}

//			uint8 debug_msg[100];
//			int n = sprintf((char *)&debug_msg, "time_next_rx_accept updated, time_next %lu, list[0]... %d, %d, %d,", this->time_next_rx_accept, this->time_till_next_reported_list[0], this->time_till_next_reported_list[1], this->time_till_next_reported_list[2]);
//			send_usbmessage(&debug_msg[0], n);
//			usb_run();
		}
	}


	if(llist_is_empty(this->list))
	{
		return llist_add_node(this->list, node, ADD_NODE_REAR);
	}
	else{

		rx_timing_node *pos_node = llist_get_head(this->list);
		while(true){

			if(node->time + offset < pos_node->time + offset){
				return llist_insert_node(this->list, node, pos_node, ADD_NODE_BEFORE);
			}

			pos_node = llist_get_next(this->list, node);
			if(pos_node == NULL){
				break;
			}
		}

		return llist_add_node(this->list, node, ADD_NODE_REAR);
	}
}

static bool reschedule_node(struct RXScheduler *this, rx_timing_node *node){

	llist_delete_node(this->list, node, false, NULL);
	node->time += this->blink_period;
	return this->add_node(this, node);
}

static void remove_expired(struct RXScheduler *this, uint32 time_now_offset, uint32 offset){

	bool update_time_next = false;

	//start by removing expired nodes
	while(true){
		int num_nodes = llist_size(this->list);
		if(num_nodes == 0){break;}

		rx_timing_node *node = llist_get_head(this->list);

		if(time_now_offset >= node->time + node->duration + offset){ //expired

			if(node->type == RX_ACCEPT){
				update_time_next = true;
			}

			llist_delete_node(this->list, node, true, NULL);
		}
		else{
			break;
		}
	}

	if(update_time_next){
		rx_timing_node *node = llist_get_head(this->list);
		while(node != NULL){
			if(node->type == RX_ACCEPT)
			{
				this->time_next_rx_accept = node->time;
				for (int i = 0; i < *this->uwb_list_length; i++) {
					this->time_till_next_reported_list[i] = 0;
				}

				break;
			}

			node = llist_get_next(this->list, node);
		}
	}


}

static uint32 get_time_till_next_rx_accept(struct RXScheduler *this){

	uint32 time_now = portGetTickCnt();
	uint32 offset = 0;
	if(time_now > 4000000000){
		offset = 1000000000;
	}
	uint32 time_now_offset = time_now + offset;

	this->remove_expired(this, time_now_offset, offset);

	if(time_now_offset >= this->time_next_rx_accept + offset){
		return 0;
	}

	return (this->time_next_rx_accept + offset) - time_now_offset;
}

static struct RXScheduler new(float blink_frequency, uint32 blink_duration, uint32 range_duration, uint8 (*uwb_list)[][8], uint8 *uwb_list_length, uint8 uwb_max_list_length, uint8 *uwb_timeout_list, uint8 *time_till_next_reported_list){
	struct RXScheduler ret = {.uwb_list = uwb_list, .uwb_list_length = uwb_list_length, .uwb_max_list_length=uwb_max_list_length, .uwb_timeout_list = uwb_timeout_list};
	ret.list = llist_create(rx_node_compare, rx_node_equal, 0);
	ret.blink_frequency = blink_frequency;
	ret.blink_duration = blink_duration;
	ret.range_duration = range_duration;
	ret.blink_period = (uint32)(1.0/blink_frequency);
	ret.rx_accept = &rx_accept;
	ret.add_node = &add_node;
	ret.remove_expired = &remove_expired;
	ret.reschedule_node = &reschedule_node;
	ret.past_accepted_end_Time = &past_accepted_end_Time;
	ret.get_time_till_next_rx_accept = &get_time_till_next_rx_accept;
	ret.time_till_next_reported_list = time_till_next_reported_list;
	ret.time_next_rx_accept = 0;
//	ret.time_reject_all = 0;
	ret.accepted_node_end_time = 0;
	ret.last_accept_index = 0;
	return ret;
}

const struct RXSchedulerClass RXScheduler={.new=&new};


