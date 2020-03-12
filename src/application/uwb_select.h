#include "llist.h"
#include "deca_types.h"


#define TIMER_WINDOW = 200    // TODO rename


typedef struct
{
    uint8 index;            	//the node index
	uint32 time;      			//time the node is scheduled to be used.
    uint32 frequency;           //how often the node should be timed. if 0, append to end of list after calling.
	uint32 duration;            //expected time the node event will take before completion

} timing_node ;


struct TimedSelector
{
    llist list;

    timing_node *(*select)(struct TimedSelector *this);
    bool (*add_node)(struct TimedSelector *this, timing_node *node);
    bool (*remove_node)(struct TimedSelector *this, timing_node *node);
};

extern const struct TimedSelectorClass
{
	struct TimedSelector (*new)();
    
} TimedSelector;


