from enum import Enum
import time
import numpy as np
import matplotlib.pyplot as plt


class UWB_TYPE(Enum):
    TAG = 1
    ANCHOR = 2

class UWB_STATE(Enum):
    IDLE = 1
    ATTEMPTING_COMMS = 2
    MAKING_COMMS = 3

    def tostring(uwb_state):
        if uwb_state == UWB_STATE.IDLE:
            return "IDLE"
        elif uwb_state == UWB_STATE.ATTEMPTING_COMMS:
            return "ATTEMPTING COMMS"
        elif uwb_state == UWB_STATE.MAKING_COMMS:
            return "MAKING COMMS"
        else:
            return "INVALID UWB_STATE"

class UWB_EVENT_TYPE(Enum):
    BLINK = 1
    POLL = 2
    MAKE_COMMS = 3
    COMPLETE_COMMS = 4
    TIMEOUT = 5

    def tostring(event_type):
        if event_type == UWB_EVENT_TYPE.BLINK:
            return "BLINK"
        elif event_type == UWB_EVENT_TYPE.POLL:
            return "POLL"
        elif event_type == UWB_EVENT_TYPE.MAKE_COMMS:
            return "MAKE COMMS"
        elif event_type == UWB_EVENT_TYPE.COMPLETE_COMMS:
            return "COMPLETE COMMS"
        elif event_type == UWB_EVENT_TYPE.TIMEOUT:
            return "TIMEOUT"
        else:
            return "INVALID UWB_EVENT_TYPE"

class UWBEvent():

    def __init__(self, event_type, event_time, target_address):
        self.type = event_type
        self.time = event_time
        self.target_address = target_address

class TimerNode():

    def __init__(self, index, duration, frequency):
        self.index = index
        self.duration = duration
        self.frequency = frequency
        self.time = 0


# class TXScheduler():


# # right now we are using this only to make ANCHORS available for blink message pairing
# class RXScheduler(TimedScheduler):


#     #check if the ANCHOR is available an incomming message
#     def check_available(self):
#         pass

#     #add a timer node that specifies time reserved for RX with a given tag
#     def add_node(self, node):
         
        



class TimedScheduler():

    window = 100                # ms
    blink_duration = 154        # ms
    range_duration = 348        # ms
    schedule_buffer = 2         # ms
    blink_frequency = 0.001     # mHz
    blink_time_tolerance = 100

    def __init__(self, time_now):
        self.list = [] #auto add a blink node
        # self.list.append(TimerNode(255, TimedScheduler.blink_duration, TimedScheduler.blink_frequency))
        # self.list[0].time = time_now
        self.recent_blink_time = time_now #ms

    def print_list(self):
        for i in range(0,len(self.list)):
            node = self.list[i]
            print("node; time: " + str(node.time) + " index: " + str(node.index))

    def select(self, time_now):
        return self.select_method_2(time_now)

    # in this method, dont have blink nodes and dont set times for the nodes
    # use node duration and blink frequency and the most recent blink time to
    # decide if we should blink or use try the next node in the list. 
    # in this method we could consider some +/- window for the blink to happen in
    # so that we can poll more frequently.
    # lets also poll immediately after sucessfull blink handshake
    def select_method_2(self, time_now):

        use_tolerance = False
        tol = 0
        if use_tolerance:
            tol = TimedScheduler.blink_time_tolerance

        if len(self.list) > 0:
            node = self.list[0]
            if time_now + node.duration < self.recent_blink_time + 1.0/TimedScheduler.blink_frequency + tol:
                self.list.remove(node)
                self.list.append(node)
                return node.index
            elif  time_now >= self.recent_blink_time + 1.0/TimedScheduler.blink_frequency - tol:
                if self.recent_blink_time == 0:
                    self.recent_blink_time = time_now
                else:
                    self.recent_blink_time = self.recent_blink_time + 1.0/TimedScheduler.blink_frequency
                return 255
            else:
                return -1  
        else:
            if time_now >= self.recent_blink_time + 1.0/TimedScheduler.blink_frequency - tol:
                if self.recent_blink_time == 0:
                    self.recent_blink_time = time_now
                else:
                    self.recent_blink_time = self.recent_blink_time + 1.0/TimedScheduler.blink_frequency
                return 255
            else:
                return -1
            
    def select_method_1(self, time_now):
        
        # self.print_list()

        while True:
            timer_node = self.list[0]

            # node not ready yet
            if time_now < timer_node.time:
                return -1
            
            # node timing missed
            if time_now - TimedScheduler.window > timer_node.time:
                # print("NODEMISS")
                # print("NODEMISS: time_now - TimedScheduler.window", time_now - TimedScheduler.window)
                # print("NODEMISS: timer_node.time", timer_node.time)
                # print("NODEMISS: timer_node.index", timer_node.index)
                if timer_node.index == 255:
                    self.recent_blink_time = timer_node.time

                self.reschedule_node(timer_node)
                # self.print_list()
                continue  

            # node timing valid
            if time_now - TimedScheduler.window <= timer_node.time:
                
                if timer_node.index == 255:
                    self.recent_blink_time = timer_node.time
                    
                    last_blink_index = -1

                    for i in reversed(range(0,len(self.list))):
                        if self.list[i].index == 255:
                            last_blink_index = i
                            break
                    
                    if last_blink_index == 0: # or -1?
                        self.reschedule_node(timer_node)
                        # self.print_list()
                        return timer_node.index   
                    else:
                        # print('blink node popped')
                        # print('last_blink_index',last_blink_index)
                        # print('timer_node.index',timer_node.index)
                        # print('self.list[last_blink_index].index',self.list[last_blink_index].index)
                        # print('len(self.list)',len(self.list))
                        # print('timer_node == self.list[last_blink_index]',timer_node == self.list[last_blink_index])
                        # self.print_list()
                        return self.pop_node(timer_node).index
                
                else:
                    self.reschedule_node(timer_node)
                    # self.print_list()
                    return timer_node.index

    def add_node_front(self, node):
        self.list.insert(0, node)

    def add_node(self, node):
        
        last_blink_node = None
        for i in reversed(range(0,len(self.list))):
            if self.list[i].index == 255:
                last_blink_node = self.list[i]
                break

        #handle reschedule of a blink node when it is the only one in the list
        if node.index == 255 and last_blink_node == None:
            node.time = self.recent_blink_time + 1.0/TimedScheduler.blink_frequency + TimedScheduler.schedule_buffer
            self.list.append(node)
            return
        
        last_node = self.list[-1]

        #last node is a blink, see if we can fit our node in beforehand
        if last_node.index == 255: 
            second_last_node_end = Sim.time
            if len(self.list) > 1:
                second_last_node = self.list[-2]
                second_last_node_end = second_last_node.time + second_last_node.duration 
        
            if last_node.time - second_last_node_end > node.duration + TimedScheduler.schedule_buffer:
                #fits before, put second to last 
                node.time = second_last_node_end + TimedScheduler.schedule_buffer
                self.list.insert(-1 , node)
            else:
                #doesn't fit before, put after

                # print('NOFIT: last_node.time - second_last_node_end', last_node.time - second_last_node_end)
                # print('NOFIT: node.duration + TimedScheduler.schedule_buffer', node.duration + TimedScheduler.schedule_buffer)
                node.time = last_node.time + last_node.duration + TimedScheduler.schedule_buffer
                self.list.append(node)
            
            return

        
        if last_blink_node == None: #this shouldn't happen...
            print('len(self.list)',len(self.list))
            print('self.list[0].index', self.list[0].index)
            print('node.index', node.index)

        #last node not blink, check if another blink node needs to be inserted to maintain blink frequency
        last_node_end = last_node.time + last_node.duration
        # print('last_node_end',last_node_end)
        if last_node_end + TimedScheduler.schedule_buffer + node.duration + TimedScheduler.schedule_buffer - last_blink_node.time > 1.0/TimedScheduler.blink_frequency:
            # print("node.duration",node.duration)
            # print("last_blink_node.time",last_blink_node.time)
            # print("1.0/TimedScheduler.blink_frequency",1.0/TimedScheduler.blink_frequency)
            self.list.append(TimerNode(255, TimedScheduler.blink_duration, TimedScheduler.blink_frequency))
            self.list[-1].time = last_blink_node.time + 1.0/TimedScheduler.blink_frequency + TimedScheduler.schedule_buffer
            last_node = self.list[-1]
            last_node_end = last_node.time + last_node.duration

        node.time = last_node_end + TimedScheduler.schedule_buffer
        self.list.append(node)

    def reschedule_node(self, node):
        self.list.remove(node)
        self.add_node(node)    
    
    def pop_node(self, node):
        self.list.remove(node)
        return node

class UWB:

    timeout_duration = 20 # ms

    def __init__(self, uwb_type, uwb_address, time_now):
        # body of the constructor
        self.uwb_list = []
        self.event_list = []
        self.type = uwb_type
        self.address = uwb_address
        self.state = UWB_STATE.IDLE
        self.state_end_time = 0
        self.uwb_selector = TimedScheduler(time_now)
        self.selected_index = -1
        self.comms_attempted = False        

    def check_state_end(self, time_now):
        if time_now > self.state_end_time:
            target_address = "NONE"
            if self.selected_index == 255:
                    target_address = "BLINK"
            elif self.selected_index > -1:
                target_address = self.uwb_list[self.selected_index].address

            if self.state == UWB_STATE.ATTEMPTING_COMMS:    
                uwb_event = UWBEvent(UWB_EVENT_TYPE.TIMEOUT, time_now, target_address)
                self.event_list.append(uwb_event)
                print(str(self.address) + " " + UWB_EVENT_TYPE.tostring(UWB_EVENT_TYPE.TIMEOUT))
            elif self.state == UWB_STATE.MAKING_COMMS:
                uwb_event = UWBEvent(UWB_EVENT_TYPE.COMPLETE_COMMS, time_now, target_address)
                self.event_list.append(uwb_event)
                print(str(self.address) + " " + UWB_EVENT_TYPE.tostring(UWB_EVENT_TYPE.COMPLETE_COMMS))

            self.state = UWB_STATE.IDLE
            self.comms_attempted = False
            self.selected_index = -1
            self.state_end_time = 0

    def run(self, time_now, comm_handler):
        
        #TODO add an active list and check for timeouts?
        
        # if self.type is UWB_TYPE.TAG:
            # print(self.address + " run()")

        if self.state == UWB_STATE.MAKING_COMMS:
            # if self.type is UWB_TYPE.TAG:
                # print("MAKING COMMS")
            return
        
        if self.type is UWB_TYPE.TAG:
            
            if self.state == UWB_STATE.ATTEMPTING_COMMS:
                # print("ATTEMPTING COMMS. index:", self.selected_index)
                anchor = None
                if self.selected_index != 255 and self.selected_index != -1:
                    anchor = self.uwb_list[self.selected_index]
                comm_handler.attempt_comms(self, anchor, time_now)        
                
            #if not already working on something, select a uwb to range with
            if self.state == UWB_STATE.IDLE:
                index = self.uwb_selector.select(time_now)
                # print("IDLE-SELECTING. index:", index)
                if index != -1:
                    # self.comms_attempted = False
                    self.selected_index = index
                    self.state = UWB_STATE.ATTEMPTING_COMMS
                    self.state_end_time = time_now + UWB.timeout_duration


class UWBCommHandler():

    def __init__(self, uwb_list):
        self.uwb_list = uwb_list

    def attempt_comms(self, tag, anchor, time_now):

        if tag.comms_attempted:
            # print("comms already attempted, return")
            return

        tag.comms_attempted = True

        #blink!
        if anchor == None:
            # print("BLINK")
            uwb_event = UWBEvent(UWB_EVENT_TYPE.BLINK, time_now, "NONE")
            tag.event_list.append(uwb_event) 
            for i in range(0, len(self.uwb_list)):
                if self.uwb_list[i].type == UWB_TYPE.ANCHOR:
                    anchor = self.uwb_list[i]
                    
                    if anchor.state != UWB_STATE.IDLE:
                        # print("ANCHOR " + anchor.address + " not idle, continue.")
                        continue

                    #TODO here is where I should check for active/timeout
                    if (tag not in anchor.uwb_list) and (anchor not in tag.uwb_list):
                        anchor.uwb_list.append(tag)
                        tag.uwb_list.append(anchor)
                        
                        tag.state = UWB_STATE.MAKING_COMMS
                        tag.state_end_time = time_now + TimedScheduler.blink_duration
                        
                        timing_node = TimerNode(len(tag.uwb_list)-1, TimedScheduler.range_duration, 0)
                        # tag.uwb_selector.add_node(timing_node)
                        tag.uwb_selector.add_node_front(timing_node)

                        anchor.state_end_time = time_now + TimedScheduler.blink_duration
                        anchor.state = UWB_STATE.MAKING_COMMS
                        
                        uwb_event = UWBEvent(UWB_EVENT_TYPE.MAKE_COMMS, time_now, anchor.address)
                        tag.event_list.append(uwb_event)
                        
                        uwb_event = UWBEvent(UWB_EVENT_TYPE.MAKE_COMMS, time_now, tag.address)
                        anchor.event_list.append(uwb_event)
                        
                        break
            
            
        #range!
        else:
            # print("POLL")
            uwb_event = UWBEvent(UWB_EVENT_TYPE.POLL, time_now, anchor.address)
            tag.event_list.append(uwb_event)

            if anchor.state != UWB_STATE.IDLE:
                # print("ANCHOR nod idle, return")
                return

            tag.state = UWB_STATE.MAKING_COMMS
            tag.state_end_time = time_now + TimedScheduler.range_duration
            
            anchor.state_end_time = time_now + TimedScheduler.range_duration
            anchor.state = UWB_STATE.MAKING_COMMS

            uwb_event = UWBEvent(UWB_EVENT_TYPE.MAKE_COMMS, time_now, anchor.address)
            tag.event_list.append(uwb_event)
            
            uwb_event = UWBEvent(UWB_EVENT_TYPE.MAKE_COMMS, time_now, tag.address)
            anchor.event_list.append(uwb_event)
    

class Sim:

    num_anchors = 3
    num_tags = 3
    blink_duration = 154    #ms
    range_duration = 348    #ms

    uwb_list = []
    uwb_comm_handler = None
    
    dt = 1                  #ms
    time = 0                #ms
    
    duration = 10000    #ms
    
    def __init__(self):
        # body of the constructor
        for i in range(0,Sim.num_anchors):
            anchor = UWB(UWB_TYPE.ANCHOR, "A"+str(i), Sim.time)
            Sim.uwb_list.append(anchor)

        for i in range(0,Sim.num_tags):
            tag = UWB(UWB_TYPE.TAG, "T"+str(i), Sim.time)
            Sim.uwb_list.append(tag)

        Sim.uwb_comm_handler = UWBCommHandler(Sim.uwb_list)
        

    def run(self):
        Sim.time = Sim.time + Sim.dt
        time_now = Sim.time

        # print("Sim Run (time "  + str(Sim.time) + ")")

        # check if state duration complete 
        for i in range(0,len(Sim.uwb_list)):
            Sim.uwb_list[i].check_state_end(time_now)

        # check for blink/ranging event 
        for i in range(0,len(Sim.uwb_list)):
            Sim.uwb_list[i].run(time_now, Sim.uwb_comm_handler)

    def plot_uwb_events():
        t = np.arange(0,Sim.duration,Sim.dt)
        plt.ylim(0,len(Sim.uwb_list) + 1)
        plt.xlim(-100,t[-1])

        for i in range(0, len(Sim.uwb_list)):
        # for i in range(3, 4):
            height = i+1
            uwb = Sim.uwb_list[i]

            plt.text(-100,height,uwb.address)

            prev_event = None
            for j in range(0,len(uwb.event_list)):
                event = uwb.event_list[j]

                if j != 0:
                    prev_event = uwb.event_list[j-1]

                if(event.type == UWB_EVENT_TYPE.BLINK):
                    plt.text(event.time, height, "B")
                    
                elif(event.type == UWB_EVENT_TYPE.POLL):
                    plt.text(event.time, height, "P")
                    plt.text(event.time, height-0.1, event.target_address)

                elif(event.type == UWB_EVENT_TYPE.MAKE_COMMS):
                    if uwb.type == UWB_TYPE.ANCHOR:
                        plt.text(event.time, height-0.1, event.target_address)
                    
                elif(event.type == UWB_EVENT_TYPE.COMPLETE_COMMS):
                    tj = np.array([prev_event.time, event.time])
                    yj = np.array([height,height])
                    plt.plot(tj,yj,'b')
                elif(event.type == UWB_EVENT_TYPE.TIMEOUT):
                    tj = np.array([prev_event.time, event.time])
                    yj = np.array([height,height])
                    plt.plot(tj,yj,'r')
                    plt.text(event.time, height+0.1, "TO")
                    
                    
        # plt.text(0,1,"HELL0")
        # plt.text(10,1,"WORLD")
        # s1 = np.sin((2 * np.pi * t)%1000)
        # plt.plot(t,s1)
        
        plt.show()

# class UWB_EVENT_TYPE(Enum):
#     BLINK = 1
#     POLL = 2
#     MAKE_COMMS = 3
#     COMPLETE_COMMS = 4
#     TIMEOUT = 5

# class UWBEvent():

#     def __init__(self, event_type, event_time, target_address):
#         self.type = event_type
#         self.time = event_time
#         self.target_address = target_address

# Initialize the sim
sim = Sim()
print('sim init')
print("uwb_list addresses")
for i in range(0, len(Sim.uwb_list)):
    print(sim.uwb_list[i].address)

# Run the sim
while(Sim.time < Sim.duration):
    sim.run()

for i in range(0,len(Sim.uwb_list[3].event_list)):
    uwb = Sim.uwb_list[3]
    event = uwb.event_list[i]
    event_string = uwb.address + "," + event.target_address + "," +  UWB_EVENT_TYPE.tostring(event.type) + "," + str(event.time)
    print(event_string)
    #  print(''Sim.uwb_list[3].event_list[i])

Sim.plot_uwb_events()
#print('len(sim.uwb_list[3].uwb_list)', len(sim.uwb_list[3].uwb_list))
# print('Sim.uwb_list[3].event_list', Sim.uwb_list[3].event_list)

print('el fin')


