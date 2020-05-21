#include "uwb_select.h"
#include "port.h"

//private mehtods
int node_compare(llist_node first, llist_node second)
{
    timing_node *first_node = (timing_node *)first;
    timing_node *second_node = (timing_node *)second;

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


bool node_equal(llist_node first, llist_node second)
{
    if(&first == &second){return true;}

    return false;
}

    
//class methods

//return timing_node pointer if valid timing_node exists
//return NULL otherwise
 static timing_node *select_node(struct TimedScheduler *this)
 { 
    uint32 time_now = portGetTickCnt();
    timing_node *node = llist_get_head(this->list);
    
    while(true)
    {
        if(node == NULL)
        {
            return NULL;
        }
        else if(node->time < time_now - (uint32)TIMER_WINDOW) //expired - reschedule and check next node
        {
            llist_delete_node(this->list, node, true, NULL); //TODO reschedule rather than delete
            
            node = llist_get_head(this->list);                        
        }
        else if(node->time < time_now) //within window - return node
        {
            return node;
        } 
        else //node not yet entered - return NULL 
        { 
            return NULL;
        }
    }
}


static bool add_node(struct TimedScheduler *this, timing_node *node) //TODO handle unsuccessful add
{    
    return llist_add_node(this->list, node, 0); //TODO fix return type
    // timing_node *head = llist_get_head(this->list);

    // if(llist_is_empty(this->list))
    // {
    //     node->time = portGetTickCnt();
    //     return llist_add_node(this->list, node, 0); //TODO fix return type
    // }

    // for(int i=0; i<llist_size(this->list); i++)
    // {
    //     // timing_node *n = llist_find_node();
    // }
}

 static bool remove_node(struct TimedScheduler *this, timing_node *node){ //TODO handle unsuccessful delete
     return llist_delete_node(this->list, node, true, NULL); //TODO fix return type
 }

static struct TimedScheduler new(){
    return (struct TimedScheduler){
        .list = llist_create(node_compare, node_equal, 0),
        .add_node = &add_node,
        .select_node = &select_node,
        .remove_node = &remove_node
    };
}

const struct TimedSchedulerClass TimedScheduler={.new=&new};
