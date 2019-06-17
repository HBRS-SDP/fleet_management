from ropod.pyre_communicator.base_class import RopodPyre
from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts
from fleet_management.exceptions.task_allocator import UnsuccessfulAllocationAlternativeTimeSlot
from datetime import timedelta
import time
import collections
import logging
import numpy as np

SLEEP_TIME = 0.350

"""
Implements the multi-robot task task_allocation algorithm TeSSI or TeSSIduo depending on the
allocation_method specified in the config file.
"""


class Auctioneer(RopodPyre):
    MISMATCHED_SCHEDULES = 1
    UNSUCCESSFUL_ALLOCATION = 2

    def __init__(self, robot_ids, allocation_method, ccu_store, api_config, auction_time=5, **kwargs):
        zyre_config = api_config.get('zyre')  # Arguments for the zyre_base class
        self.logger = logging.getLogger('fms.task.allocation.auctioneer')
        self.ccu_store = ccu_store

        self.robot_ids = robot_ids
        self.method = allocation_method

        self.auction_opened = False
        self.auction_closure_time = -1
        self.auction_time = timedelta(seconds=auction_time)

        # node_name = 'auctioneer_' + self.method
        super().__init__(zyre_config)

        self.request_timeslot_opened = False
        self.request_closure_time = -1

        self.tasks_to_allocate = list()
        self.unsuccessful_allocations = list()

        self.allocate_next_task = False
        self.received_updated_schedule = False
        self.next_request = False
        self.done = False

        self.allocations = dict()
        self.robot_schedules = dict()
        self.timetable = dict()
        self.makespan = dict()

        # Bids received in one allocation iteration
        self.received_bids = list()
        self.received_no_bids = dict()
        self.n_round = 0

        self.received_timeslots = dict()

    def __str__(self):
        auctioneer_info = list()
        auctioneer_info.append("Auctioneer")
        auctioneer_info.append("Method = " + self.method)
        auctioneer_info.append("Zyre groups")
        auctioneer_info.append("{}".format(self.groups()))
        return '\n '.join(auctioneer_info)

    ''' Allocates a single task or a list of tasks.
            Returns a dictionary
            key - task_id
            value - list of robot_ids assigned to the task_id
            @param task an object of type Task
            or a list of objects of type Task
            '''

    def allocate(self, tasks):
        self.receive_tasks(tasks)
        while True:
            self.announce_task()
            self.check_auction_closure_time()
            self.request_timeslot()
            alternative_timeslots = self.check_request_closure_time()
            time.sleep(0.8)
            if self.done is True:
                break

        if not isinstance(tasks, list):
            allocations = self.get_allocations([tasks])
        else:
            allocations = self.get_allocations(tasks)

        if alternative_timeslots:
            raise UnsuccessfulAllocationAlternativeTimeSlot(alternative_timeslots)

        return allocations

        # # Return allocations of tasks allocated in the current allocation process
        # if not isinstance(tasks, list):
        #     return self.get_allocations([tasks])
        # return self.get_allocations(tasks)

    ''' If no argument is given, returns all allocations.
        If an argument (list of tasks) is given, returns the allocations of the given tasks '''

    def receive_tasks(self, tasks):
        if isinstance(tasks, list):
            for task in tasks:
                self.tasks_to_allocate.append(task)
            self.logger.debug('Auctioneer received a list of tasks')
        else:
            self.tasks_to_allocate.append(tasks)
            self.logger.debug('Auctioneer received one task')

        self.allocate_next_task = True
        self.received_updated_schedule = True
        self.done = False

    """
        Triggers the task_allocation process.
    The task_allocation process consists of n rounds of auctions, where n is the number of tasks
    in the list of tasks_to_allocate.
    One task is allocated per round until there are no more tasks left to allocate.
    Robots that have allocated task(s) in previous round(s) still participate in the next round(s).

    In each round:
    - The auctioneer announces all unallocated tasks.
    - Each robot calculates a bid for each unallocated task and submits
      the smallest one (i.e, each robot submits only one bid per round).
    - The auctioneer allocates the task to the robot with the smallest bid.
    - The auctioneer removes the allocated task from the list of tasks_to_allocate.
    """
    def announce_task(self):

        if self.tasks_to_allocate and self.allocate_next_task and self.received_updated_schedule:

            self.allocate_next_task = False
            self.received_updated_schedule = False
            self.reinitialize_auction_variables()

            self.logger.debug("Starting round: %s", self.n_round)
            self.logger.debug("Number of tasks to allocate: %s", len(self.tasks_to_allocate))

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

            for task in self.tasks_to_allocate:
                task_announcement['payload']['tasks'][task.id] = task.to_dict()

            self.logger.debug("Auctioneer announces tasks %s", [unallocated_task.id for unallocated_task in self.tasks_to_allocate])

            auction_open_time = ts.get_time_stamp()
            self.auction_opened = True
            self.auction_closure_time = ts.get_time_stamp(self.waiting_time)
            self.logger.debug("Auction opened at %s and will close at %s", auction_open_time, self.auction_closure_time)

            self.shout(task_announcement, 'TASK-ALLOCATION')

        elif not self.tasks_to_allocate and self.allocate_next_task and self.received_updated_schedule and not self.request_timeslot_opened:
            self.terminate_allocation()

    def terminate_allocation(self):
        self.logger.info("Task allocation finished")
        self.done = True
        self.n_round = 0

    def reinitialize_auction_variables(self):
        self.received_bids = list()
        self.received_no_bids = dict()
        self.n_round += 1

    ''' Call funcion elect_winner if the current time exceeds the auction closure time
    '''
    def check_auction_closure_time(self):
        if self.auction_opened:
            current_time = ts.get_time_stamp()
            if current_time >= self.auction_closure_time:
                self.logger.debug("Closing auction at %s", current_time)
                self.next_request = True
                self.auction_opened = False
                self.elect_winner()

    def check_request_closure_time(self):
        if self.request_timeslot_opened:
            current_time = ts.get_time_stamp()
            if current_time >= self.request_closure_time:
                self.logger.debug("Closing suggestion process at %s", current_time)
                selected_timeslots = self.select_timeslots()
                self.request_timeslot_opened = False
                self.terminate_allocation()
                return selected_timeslots

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
            self.logger.debug("Received bid %s", bid)

        if message_type == 'NO-BID':
            robot_id = dict_msg['payload']['robot_id']
            cause = dict_msg['payload']['cause']
            task_id = dict_msg['payload']['task_id']

            if task_id in self.received_no_bids:
                self.received_no_bids[task_id] += 1
            else:
                self.received_no_bids[task_id] = 1

            self.logger.info("Received NO-BID from %s for task %s. Cause: %s",
                             robot_id, task_id, cause)

            if cause == self.UNSUCCESSFUL_ALLOCATION:
                self.check_need_alternative_timeslots()

        if message_type == 'SCHEDULE':
            robot_id = dict_msg['payload']['robot_id']
            robot_schedule = dict_msg['payload']['robot_schedule']
            timetable = dict_msg['payload']['timetable']

            self.robot_schedules[robot_id] = robot_schedule
            self.timetable[robot_id] = timetable

            self.logger.debug("Auctioneer received schedule of robot %s", robot_id)
            self.ccu_store.update_robot_schedule(robot_id, robot_schedule)
            self.logger.debug("Auctioneer wrote schedule of robot %s to the ccu_store", robot_id)
            self.received_updated_schedule = True

        elif message_type == 'TASK-ALTERNATIVE-TIMESLOT':
            task_alternative_timeslot = dict()
            task_id = dict_msg['payload']['task_id']
            task_alternative_timeslot['robot_id'] = dict_msg['payload']['robot_id']
            task_alternative_timeslot['start_time'] = dict_msg['payload']['start_time']
            self.received_timeslots[task_id].append(task_alternative_timeslot)
            self.logger.debug("Auctioneer received task_alternative_timeslot %s", task_alternative_timeslot)

    """ If the number of no-bids for a task is equal to the number of robots, add the task
    to the list of unallocated tasks"""
    def check_need_alternative_timeslots(self):
        for task_id, n_no_bids in self.received_no_bids.items():
            if n_no_bids == len(self.robot_ids):
                for i, task in enumerate(self.tasks_to_allocate):
                    if task.id == task_id:
                        self.logger.debug("Delete task from tasks to allocate %s", task_id)
                        del self.tasks_to_allocate[i]
                        self.logger.debug("Adding task %s to unsuccessful_allocations", task_id)
                        self.unsuccessful_allocations.append(task)

    ''' Requests start time suggestions for tasks in unsuccessful_allocations list'''
    def request_timeslot(self):
        if self.unsuccessful_allocations and self.next_request:

            self.next_request = False
            for task in self.unsuccessful_allocations:

                request_suggestion = dict()
                request_suggestion['header'] = dict()
                request_suggestion['payload'] = dict()
                request_suggestion['header']['type'] = 'REQUEST-TIMESLOT'
                request_suggestion['header']['metamodel'] = 'ropod-msg-schema.json'
                request_suggestion['header']['msgId'] = generate_uuid()
                request_suggestion['header']['timestamp'] = ts.get_time_stamp()
                request_suggestion['payload']['metamodel'] = 'ropod-request-time-slot-schema.json'
                request_suggestion['payload']['task'] = task.to_dict()

                self.received_timeslots[task.id] = list()

                self.logger.debug("Request alternative time slot for task %s", task.id)
                self.shout(request_suggestion, 'TASK-ALLOCATION')

            request_suggestion_open_time = ts.get_time_stamp()
            self.request_timeslot_opened = True
            self.request_closure_time = ts.get_time_stamp(self.waiting_time)

            self.logger.debug("Request opened at %s and will close at %s", request_suggestion_open_time,
                              self.request_closure_time)

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

            # Remove allocated task from self.tasks_to_allocate
            for i, task in enumerate(self.tasks_to_allocate):
                if task.id == allocated_task:
                    self.logger.debug("Removing task %s from tasks_to_allocate", task.id)
                    del self.tasks_to_allocate[i]

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
        self.allocate_next_task = True

    def select_timeslots(self):
        self.logger.debug("Selecting the timeslot closest to the desired earliest start time")
        smallest_diff = np.inf
        selected_time_slots = dict()

        for task_id, timeslots in self.received_timeslots.items():
            task = [task for task in self.unsuccessful_allocations if task.id == task_id][0]
            self.logger.debug("Desired start time %s", task.earliest_start_time)
            for timeslot in timeslots:
                if abs(timeslot['start_time'] - task.earliest_start_time) < smallest_diff:
                    smallest_diff = abs(timeslot['start_time'] - task.earliest_start_time)
                    selected_robot = timeslot['robot_id']
                    selected_start_time = timeslot['start_time']
            self.logger.debug("Selected timeslot for task %s: robot %s with start time %s", task.id, selected_robot,
                  selected_start_time)

            selected_time_slots[task_id] = dict()
            selected_time_slots[task_id]['robot_id'] = selected_robot
            selected_time_slots[task_id]['start_time'] = selected_start_time

        return selected_time_slots

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

    ''' Returns a list with the task_ids allocated to the robot with id=ropod_id
    '''
    def get_allocations_robot(self, ropod_id):
        allocations = self.get_allocations()
        allocations_robot = list()
        if allocations:
            for task_id, robot_ids in allocations.items():
                if ropod_id in robot_ids:
                    allocations_robot.append(task_id)
                else:
                    self.logger.info("There are no tasks allocated to %s ", ropod_id)
        else:
            self.logger.info("There are no allocated tasks")

        return allocations_robot

    ''' Returns a dictionary with the task_ids scheduled to all robots
    key - robot_id
    value - list of task_ids
    '''
    def get_scheduled_tasks(self):
        return self.robot_schedules

    ''' Returns a dictionary with the start time and finish time of the tasks assigned to the robot with id=ropod_id
    keys:
    '''
    def get_task_schedule(self, task_id, robot_id):
        if self.timetable[robot_id].get(task_id):
            return self.timetable[robot_id][task_id]

    ''' Returns a list with the task_ids scheduled (in the order they will be executed) to the robot with id=ropod_id
    '''
    def get_scheduled_tasks_robot(self, ropod_id):
        scheduled_tasks = self.get_scheduled_tasks()
        scheduled_tasks_robot = list()

        if ropod_id in scheduled_tasks:
            scheduled_tasks_robot = scheduled_tasks[ropod_id]
        else:
            self.logger.info("No tasks scheduled to %s", ropod_id)

        return scheduled_tasks_robot

    ''' Returns a dictionary with the start time and finish time of each allocated task.
        timetable[robot_id][task_id]['start_time']
        timetable[robot_id][task_id]['finish_time']
    '''
    def get_tasks_schedule(self):
        return self.timetable
