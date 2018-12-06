from ropod.pyre_communicator.base_class import PyreBaseCommunicator
import uuid
import time
import collections
SLEEP_TIME = 0.350

""" 
Implements the multi-robot task task_allocation algorithm TeSSI or TeSSIduo depending on the 
allocation_method specified in the config file.
"""


class Auctioneer(PyreBaseCommunicator):
    def __init__(self, config_params, verbose_mrta=False):

        self.robot_ids = config_params.ropods
        self.method = config_params.allocation_method
        self.zyre_params = config_params.task_allocator_zyre_params
        node_name = 'auctioneer_' + self.method

        super().__init__(node_name, self.zyre_params.groups, self.zyre_params.message_types)

        self.unallocated_tasks = list()
        self.allocate_next_task = False
        self.done = False

        self.allocations = dict()
        self.unsuccessful_allocations = list()
        self.task_schedule_index = dict()
        self.timetable = dict()
        self.makespan = dict()

        # Bids received in one allocation iteration
        self.received_bids = list()
        self.n_bids_received = 0
        self.n_no_bids_received = 0
        self.n_round = 0

        # Define function for printing debug information
        self.verbose_mrta = verbose_mrta
        self.verboseprint = print if self.verbose_mrta else lambda *a, **k: None

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
            self.verboseprint('[INFO] Auctioneer received a list of tasks')
        else:
            self.unallocated_tasks.append(tasks)
            self.verboseprint('[INFO] Auctioneer received one task')

        self.allocate_next_task = True
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

        if self.unallocated_tasks and self.allocate_next_task:

            self.allocate_next_task = False
            self.reinitialize_auction_variables()

            print("[INFO] Starting round: ", self.n_round)
            print("[INFO] Number of tasks to allocate: ", len(self.unallocated_tasks))

            # Create task announcement message that contains all unallocated tasks
            task_announcement = dict()
            task_announcement['header'] = dict()
            task_announcement['payload'] = dict()
            task_announcement['header']['type'] = 'TASK-ANNOUNCEMENT'
            task_announcement['header']['metamodel'] = 'ropod-msg-schema.json'
            task_announcement['header']['msgId'] = self.generate_uuid()
            task_announcement['header']['timestamp'] = self.get_time_stamp()

            task_announcement['payload']['metamodel'] = 'ropod-task-announcement-schema.json'
            task_announcement['payload']['round'] = self.n_round
            task_announcement['payload']['tasks'] = dict()

            for task in self.unallocated_tasks:
                task_announcement['payload']['tasks'][task.id] = task.to_dict()

            self.verboseprint("[INFO] Auctioneer announces tasks.")
            self.shout(task_announcement, 'TASK-ALLOCATION')

        elif not self.unallocated_tasks and self.allocate_next_task:
            print("Done")
            self.done = True

    def reinitialize_auction_variables(self):
        self.received_bids = list()
        self.n_bids_received = 0
        self.n_no_bids_received = 0
        self.n_round += 1

    def check_n_received_bids(self):
        if (self.n_bids_received + self.n_no_bids_received) == len(self.robot_ids):
            self.verboseprint("[INFO] Auctioneer has received a message from all robots")
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
            self.verboseprint("[INFO] Received bid {}".format(bid))
            self.check_n_received_bids()

        if message_type == 'NO-BID':
            no_bid = dict()
            no_bid['robot_id'] = dict_msg['payload']['robot_id']
            self.n_no_bids_received += 1
            self.verboseprint("[INFO] Received NO-BID from", no_bid['robot_id'])
            self.check_n_received_bids()

        elif message_type == 'SCHEDULE':
            robot_id = dict_msg['payload']['robot_id']
            task_schedule_index = dict_msg['payload']['task_schedule_index']
            timetable = dict_msg['payload']['timetable']
            makespan = dict_msg['payload']['makespan']

            self.task_schedule_index[robot_id] = task_schedule_index
            self.timetable[robot_id] = timetable
            self.makespan[robot_id] = makespan

            self.verboseprint("[INFO] Auctioneer received schedule {} of robot {}".format(task_schedule_index, robot_id))

    def elect_winner(self):
        if self.received_bids:
            self.verboseprint("[INFO] Number of bids received: ", len(self.received_bids))
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
                self.verboseprint("[INFO] For task {} there is a tie between: {}".format(allocated_task,
                                                                                         [robot_id for robot_id in
                                                                                          robots_tied]))
                robots_tied.sort(key=lambda x: int(x.split('_')[-1]))

            winning_robot = robots_tied[0]

            self.verboseprint("[INFO] Robot {} wins task {}".format(winning_robot, allocated_task))
            self.announce_winner(allocated_task, winning_robot)

            self.allocations[allocated_task] = [winning_robot]

            # Remove allocated task from self.unallocated_tasks
            for i, task in enumerate(self.unallocated_tasks):
                if task.id == allocated_task:
                    self.verboseprint("[INFO] Removing task {} from unallocated_tasks".format(task.id))
                    del self.unallocated_tasks[i]
        else:
            self.verboseprint("[INFO] Tasks in unallocated tasks could not be allocated")
            for unallocated_task in self.unallocated_tasks:
                self.unsuccessful_allocations.append(unallocated_task.id)
            self.allocate_next_task = True
            self.unallocated_tasks = list()

    def announce_winner(self, allocated_task, winning_robot):
        # Create allocation message
        allocation = dict()
        allocation['header'] = dict()
        allocation['payload'] = dict()
        allocation['header']['type'] = 'ALLOCATION'
        allocation['header']['metamodel'] = 'ropod-msg-schema.json'
        allocation['header']['msgId'] = self.generate_uuid()
        allocation['header']['timestamp'] = self.get_time_stamp()

        allocation['payload']['metamodel'] = 'ropod-allocation-schema.json'
        allocation['payload']['task_id'] = allocated_task
        allocation['payload']['winner_id'] = winning_robot

        self.verboseprint("[INFO] Accouncing winner...")
        self.shout(allocation, 'TASK-ALLOCATION')

        # Sleep so that the winner robot has time to process the allocation
        time.sleep(SLEEP_TIME)
        self.allocate_next_task = True

    ''' Returns a dictionary of allocations
    key - task_id
    value - list of robot_ids assigned to the task_id
    '''
    def get_allocations(self):
        return self.allocations

    ''' Returns a list of tasks that could not be allocated in the task_allocation process
    '''
    def get_unsuccessful_allocations(self):
        return self.unsuccessful_allocations

    ''' Returns a dictionary with the task_ids scheduled to all robots
    key - robot_id
    value - list of task_ids
    '''
    def get_scheduled_tasks(self):
        return self.task_schedule_index

    ''' Returns a dictionary with the start time and finish time of each allocated task.
        timetable[robot_id][task_id]['start_time']
        timetable[robot_id][task_id]['finish_time']
    '''
    def get_tasks_schedule(self):

        return self.timetable

    def get_tasks_schedule_robot(self, task_id, robot_id):

        if self.timetable[robot_id].get(task_id):
            return self.timetable[robot_id][task_id]
