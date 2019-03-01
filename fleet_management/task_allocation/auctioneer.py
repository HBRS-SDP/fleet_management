from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts
from datetime import timedelta
import time
import collections
import logging

SLEEP_TIME = 0.350

""" 
Implements the multi-robot task task_allocation algorithm TeSSI or TeSSIduo depending on the 
allocation_method specified in the config file.
"""


class Auctioneer(RopodPyre):
    def __init__(self, config_params, ccu_store):
        self.logger = logging.getLogger('fms.task.allocation.auctioneer')
        self.ccu_store = ccu_store

        self.robot_ids = config_params.robots
        self.method = config_params.allocation_method

        self.auction_opened = False
        self.auction_closure_time = -1
        self.auction_time = timedelta(seconds=config_params.auction_time)

        self.zyre_params = config_params.task_allocator_zyre_params
        node_name = 'auctioneer_' + self.method

        super().__init__(node_name, self.zyre_params.groups, self.zyre_params.message_types)

        self.unallocated_tasks = list()
        self.allocate_next_task = False
        self.received_updated_schedule = False
        self.done = False

        self.allocations = dict()
        self.unsuccessful_allocations = list()
        self.robot_schedules = dict()
        self.timetable = dict()
        self.makespan = dict()

        # Bids received in one allocation iteration
        self.received_bids = list()
        self.n_bids_received = 0
        self.n_no_bids_received = 0
        self.n_round = 0

    def __str__(self):
        auctioneer_info = list()
        auctioneer_info.append("Auctioneer")
        auctioneer_info.append("Method = " + self.method)
        auctioneer_info.append("Zyre groups")
        auctioneer_info.append("{}".format(self.groups()))
        return '\n '.join(auctioneer_info)

    def receive_tasks(self, tasks):
        if isinstance(tasks, list):
            for task in tasks:
                self.unallocated_tasks.append(task)
            self.logger.debug('Auctioneer received a list of tasks')
        else:
            self.unallocated_tasks.append(tasks)
            self.logger.debug('Auctioneer received one task')

        self.allocate_next_task = True
        self.received_updated_schedule = True
        self.done = False

    """
        Triggers the task_allocation process.
    The task_allocation process consists n rounds of auctions, where n is the number of tasks 
    in the list of unallocated_tasks.
    One task is allocated per round until there are no more tasks left to allocate. 
    Robots that have allocated task(s) in previous round(s) still participate in the next round(s).

    In each round:
    - The auctionner announces all unallocated tasks.
    - Each robot calculates a bid for each unallocated task and submits 
      the smallest one (i.e, each robot submits only one bid per round).
    - The auctionner allocates the task to the robot with the smallest bid.
    - The auctioneer removes the allocated task from the list of unallocated_tasks.
    """

    def announce_task(self):

        if self.unallocated_tasks and self.allocate_next_task and self.received_updated_schedule:

            self.allocate_next_task = False
            self.received_updated_schedule = False
            self.reinitialize_auction_variables()

            self.logger.debug("Starting round: %s", self.n_round)
            self.logger.debug("Number of tasks to allocate: %s", len(self.unallocated_tasks))

            # Create task announcement message that contains all unallocated tasks
            task_announcement = dict()
            task_announcement['header'] = dict()
            task_announcement['payload'] = dict()
            task_announcement['header']['type'] = 'TASK-ANNOUNCEMENT'
            task_announcement['header']['metamodel'] = 'ropod-msg-schema.json'
            task_announcement['header']['msgId'] = generate_uuid()
            task_announcement['header']['timestamp'] = ts.get_time_stamp()

            task_announcement['payload']['metamodel'] = 'ropod-task-announcement-schema.json'
            task_announcement['payload']['round'] = self.n_round
            task_announcement['payload']['tasks'] = dict()

            for task in self.unallocated_tasks:
                task_announcement['payload']['tasks'][task.id] = task.to_dict()

            self.logger.debug("Auctioneer announces tasks %s", [unallocated_task.id for unallocated_task in self.unallocated_tasks])

            auction_open_time = ts.get_time_stamp()
            self.auction_opened = True
            self.auction_closure_time = ts.get_time_stamp(self.auction_time)

            self.logger.debug("Auction opened at %s and will close at %s", auction_open_time, self.auction_closure_time)

            self.shout(task_announcement, 'TASK-ALLOCATION')

        elif not self.unallocated_tasks and self.allocate_next_task and self.received_updated_schedule:
            self.logger.info("Task announcement finished")
            self.done = True
            self.n_round = 0

    def reinitialize_auction_variables(self):
        self.received_bids = list()
        self.n_bids_received = 0
        self.n_no_bids_received = 0
        self.n_round += 1

    ''' Call funcion elect_winner if the current time exceeds the auction closure time
    '''

    def check_auction_closure_time(self):
        if self.auction_opened:
            current_time = ts.get_time_stamp()
            if current_time >= self.auction_closure_time:
                self.logger.debug("Closing auction at %s", current_time)
                self.auction_opened = False
                self.elect_winner()

    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            return
        message_type = dict_msg['header']['type']

        if message_type == 'BID':
            bid = dict()
            bid['task_id'] = dict_msg['payload']['task_id']
            bid['robot_id'] = dict_msg['payload']['robot_id']
            bid['bid'] = dict_msg['payload']['bid']
            self.received_bids.append(bid)
            self.n_bids_received += 1
            self.logger.debug("Received bid %s", bid)

        if message_type == 'NO-BID':
            no_bid = dict()
            no_bid['robot_id'] = dict_msg['payload']['robot_id']
            no_bid['cause'] = dict_msg['payload']['cause']
            self.n_no_bids_received += 1
            self.logger.debug("Received NO-BID from %s %s", no_bid['robot_id'], no_bid['cause'])

        elif message_type == 'SCHEDULE':
            robot_id = dict_msg['payload']['robot_id']
            robot_schedule = dict_msg['payload']['robot_schedule']
            timetable = dict_msg['payload']['timetable']
            makespan = dict_msg['payload']['makespan']

            self.robot_schedules[robot_id] = robot_schedule
            self.timetable[robot_id] = timetable
            self.makespan[robot_id] = makespan

            self.logger.debug("Auctioneer received schedule %s of robot %s", robot_schedule, robot_id)

            self.ccu_store.update_robot_schedule(robot_id, robot_schedule)
            self.logger.debug("Auctioneer wrote schedule of robot %s to the ccu_store", robot_id)

            self.received_updated_schedule = True

    def elect_winner(self):
        if self.received_bids:
            self.logger.debug("Number of bids received: %s ", len(self.received_bids))
            lowest_bid = float('Inf')
            ordered_bids = dict()
            robots_tied = list()

            for bid in self.received_bids:
                if bid['task_id'] not in ordered_bids:
                    ordered_bids[bid['task_id']] = dict()
                    ordered_bids[bid['task_id']]['robot_id'] = list()
                    ordered_bids[bid['task_id']]['bids'] = list()

                ordered_bids[bid['task_id']]['bids'].append(bid['bid'])
                ordered_bids[bid['task_id']]['robot_id'].append(bid['robot_id'])

            # Order dictionary by task_id
            ordered_bids = collections.OrderedDict(sorted(ordered_bids.items()))

            # Resolve ties. If more than one task has the same bid,
            # select the task with the lowest_id.
            # If for that task, more than a robot has a bid, select the robot with the lowest id

            for task_id, values in ordered_bids.items():
                if min(values['bids']) < lowest_bid:

                    lowest_bid = min(values['bids'])
                    allocated_task = task_id
                    robots_tied = list()
                    for i, robot in enumerate(values['robot_id']):
                        if values['bids'][i] == lowest_bid:
                            robots_tied.append(values['robot_id'][i])

            if len(robots_tied) > 1:
                self.logger.debug("For task %s there is a tie between: %s", allocated_task, [robot_id for robot_id in
                                                                                             robots_tied])
                robots_tied.sort(key=lambda x: int(x.split('_')[-1]))

            winning_robot = robots_tied[0]

            self.logger.info("Robot %s wins task %s", winning_robot, allocated_task)
            self.announce_winner(allocated_task, winning_robot)

            self.allocations[allocated_task] = [winning_robot]

            # Remove allocated task from self.unallocated_tasks
            for i, task in enumerate(self.unallocated_tasks):
                if task.id == allocated_task:
                    self.logger.debug("Removing task %s from unallocated_tasks", task.id)
                    del self.unallocated_tasks[i]
        else:
            self.logger.info("Tasks in unallocated tasks could not be allocated")
            for unallocated_task in self.unallocated_tasks:
                self.unsuccessful_allocations.append(unallocated_task.id)
            self.unallocated_tasks = list()

        self.allocate_next_task = True

    def announce_winner(self, allocated_task, winning_robot):
        # Create allocation message
        allocation = dict()
        allocation['header'] = dict()
        allocation['payload'] = dict()
        allocation['header']['type'] = 'ALLOCATION'
        allocation['header']['metamodel'] = 'ropod-msg-schema.json'
        allocation['header']['msgId'] = generate_uuid()
        allocation['header']['timestamp'] = ts.get_time_stamp()

        allocation['payload']['metamodel'] = 'ropod-allocation-schema.json'
        allocation['payload']['task_id'] = allocated_task
        allocation['payload']['winner_id'] = winning_robot

        self.logger.debug("Announcing winners...")
        self.shout(allocation, 'TASK-ALLOCATION')

        # Sleep so that the winner robot has time to process the allocation
        time.sleep(SLEEP_TIME)

    ''' Returns a dictionary of allocations
    key - task_id
    value - list of robot_ids assigned to the task_id
    If no argument is given, returns all allocations. 
    If an argument (list of tasks) is given, returns the allocations of the given tasks
    '''

    def get_allocations(self, tasks=list()):
        allocations = dict()

        if tasks:
            task_ids = [task.id for task in tasks]
            for task_id, robot_ids in self.allocations.items():
                if task_id in task_ids:
                    allocations[task_id] = robot_ids
        else:
            allocations = self.allocations

        return allocations

    ''' Returns a list of tasks that could not be allocated in the task_allocation process
    '''

    def get_unsuccessful_allocations(self):
        return self.unsuccessful_allocations

    ''' Returns a dictionary with the task_ids scheduled to all robots
    key - robot_id
    value - list of task_ids
    '''

    def get_scheduled_tasks(self):
        return self.robot_schedules

    ''' Returns a dictionary with the start time and finish time of each allocated task.
        timetable[robot_id][task_id]['start_time']
        timetable[robot_id][task_id]['finish_time']
    '''

    def get_tasks_schedule(self):

        return self.timetable

    def get_tasks_schedule_robot(self, task_id, robot_id):

        if self.timetable[robot_id].get(task_id):
            return self.timetable[robot_id][task_id]
