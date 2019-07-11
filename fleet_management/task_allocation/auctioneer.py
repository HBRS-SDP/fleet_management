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


class Auctioneer(object):
    MISMATCHED_SCHEDULES = 1
    UNSUCCESSFUL_ALLOCATION = 2

    def __init__(self, robot_ids, allocation_method, ccu_store, api_config, auction_time=5, **kwargs):
        self.api = api_config
        self.logger = logging.getLogger('fms.task.allocation.auctioneer')
        self.ccu_store = ccu_store

        self.robot_ids = robot_ids
        self.method = allocation_method

        # New incoming tasks are added to self.tasks_to_allocate
        self.tasks_to_allocate = list()
        self.auction_opened = False
        self.auction_closure_time = -1
        self.auction_time = timedelta(seconds=auction_time)
        self.allocate_next_task = True
        self.received_updated_schedule = True

        # Bids received in current allocation round
        self.received_bids = list()
        self.received_no_bids = dict()
        self.n_round = 0

        self.allocations = dict()
        self.robot_schedules = dict()
        self.timetable = dict()

        # Flag that indicates when an allocation is completed
        self.allocation_completed = False

        # Tasks that could not be allocated in their given timewindow are added to self.unsuccessful_allocations
        self.unsuccessful_allocations = list()
        self.request_timeslot_opened = False
        self.request_timeslot_closure_time = -1
        self.request_next_timeslot = False

        # Received time_slots in a request_timeslot round
        self.received_timeslots = dict()

        # Timeslots selected in current request_timeslot round
        self.selected_timeslots = dict()

    def __str__(self):
        auctioneer_info = list()
        auctioneer_info.append("Auctioneer")
        auctioneer_info.append("Method = " + self.method)
        return '\n '.join(auctioneer_info)

    def run(self):
        if self.tasks_to_allocate and self.allocate_next_task and self.received_updated_schedule:
            self.announce_task()

        if self.unsuccessful_allocations:
            self.request_timeslot()

        if self.auction_opened:
            self.check_auction_closure_time()

        if self.request_timeslot_opened:
            self.check_request_timeslot_closure_time()

    def allocate(self, tasks):
        if isinstance(tasks, list):
            for task in tasks:
                self.tasks_to_allocate.append(task)
            self.logger.debug('Auctioneer received a list of tasks')
        else:
            self.tasks_to_allocate.append(tasks)
            self.logger.debug('Auctioneer received one task')

    """
    The task_allocation process consists of n rounds of auctions, where n is the number of tasks
    in the list of tasks_to_allocate.
    One task is announced per round until there are no more tasks left to allocate.

    In each round:
    - The auctioneer announces all unallocated tasks.
    - Each robot calculates a bid for each unallocated task and submits
    the smallest one (i.e, each robot submits only one bid per round).
    - The auctioneer allocates the task to the robot with the smallest bid.
    - The auctioneer removes the allocated task from the list of tasks_to_allocate.
    """
    def announce_task(self):

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

        self.api.publish(task_announcement, 'TASK-ALLOCATION')

        self.start_allocation_round()

    def start_allocation_round(self):
        round_open_time = ts.get_time_stamp()
        self.auction_closure_time = ts.get_time_stamp(self.auction_time)
        self.logger.debug("Auction round opened at %s and will close at %s", round_open_time, self.auction_closure_time)
        self.auction_opened = True

    def start_request_timeslot_round(self):
        request_open_time = ts.get_time_stamp()
        self.request_timeslot_closure_time = ts.get_time_stamp(self.auction_time)
        self.logger.debug("Request timeslot round opened at %s and will close at %s", request_open_time, self.request_timeslot_closure_time)
        self.request_timeslot_opened = True

    def finish_round(self, allocated_task):
        self.allocation_completed = True
        self.allocated_task = allocated_task

    def reinitialize_auction_variables(self):
        self.allocation_completed = False
        self.received_bids = list()
        self.received_no_bids = dict()
        self.n_round += 1

    def check_auction_closure_time(self):
        current_time = ts.get_time_stamp()
        if current_time >= self.auction_closure_time:
            self.logger.debug("Closing auction round at %s", current_time)
            self.auction_opened = False
            self.elect_winner()

    def check_request_timeslot_closure_time(self):
        current_time = ts.get_time_stamp()
        if current_time >= self.request_timeslot_closure_time:
            self.logger.debug("Closing request timeslot round at %s", current_time)
            self.request_timeslot_opened = False
            self.select_timeslots()

    def bid_cb(self, msg):
        bid = dict()
        bid['task_id'] = msg['payload']['task_id']
        bid['robot_id'] = msg['payload']['robot_id']
        bid['bid'] = msg['payload']['bid']
        self.received_bids.append(bid)
        self.logger.debug("Received bid %s", bid)

    def no_bid_cb(self, msg):
        robot_id = msg['payload']['robot_id']
        cause = msg['payload']['cause']
        task_id = msg['payload']['task_id']

        if task_id in self.received_no_bids:
            self.received_no_bids[task_id] += 1
        else:
            self.received_no_bids[task_id] = 1

        self.logger.info("Received NO-BID from %s for task %s. Cause: %s",
                         robot_id, task_id, cause)

        if cause == self.UNSUCCESSFUL_ALLOCATION:
            self.check_need_alternative_timeslots()

    def schedule_cb(self, msg):
        robot_id = msg['payload']['robot_id']
        robot_schedule = msg['payload']['robot_schedule']
        timetable = msg['payload']['timetable']

        self.robot_schedules[robot_id] = robot_schedule
        self.timetable[robot_id] = timetable

        self.logger.debug("Auctioneer received schedule of robot %s", robot_id)
        self.ccu_store.update_robot_schedule(robot_id, robot_schedule)
        self.logger.debug("Auctioneer wrote schedule of robot %s to the ccu_store", robot_id)
        self.received_updated_schedule = True

    def alternative_timeslot_cb(self, msg):
        task_alternative_timeslot = dict()
        task_id = msg['payload']['task_id']
        task_alternative_timeslot['robot_id'] = msg['payload']['robot_id']
        task_alternative_timeslot['start_time'] = msg['payload']['start_time']
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
            self.api.publish(request_suggestion, 'TASK-ALLOCATION')
            self.start_request_timeslot_round()

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

            self.finish_round(allocated_task)

        else:
            self.logger.debug("No bids received")

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
        self.api.publish(allocation, 'TASK-ALLOCATION')

        # Sleep so that the winner robot has time to process the allocation
        time.sleep(SLEEP_TIME)

    def select_timeslots(self):
        self.logger.debug("Selecting the timeslot closest to the desired earliest start time")
        smallest_diff = np.inf

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

            self.selected_timeslots[task_id] = dict()
            self.selected_timeslots[task_id]['robot_id'] = selected_robot
            self.selected_timeslots[task_id]['start_time'] = selected_start_time

    ''' Returns dictionary with allocation for task_id
     key - task_id
     value - list of robot_ids assigned to the task_id
    If there are alternative timeslots it raises an exception
    '''
    def get_allocation(self, task_id):
        allocation = dict()

        if task_id in self.allocations:
            robot_ids = self.allocations.pop(task_id)
            allocation[task_id] = robot_ids
            self.logger.debug("Allocation %s", allocation)
        else:
            self.logger.debug("Task %s has not been allocated", task_id)

        if self.selected_timeslots:
           raise UnsuccessfulAllocationAlternativeTimeSlot(self.selected_timeslots)

        return allocation

    ''' Returns a list of tasks that could not be allocated in the task_allocation process
    '''
    def get_unsuccessful_allocations(self):
        return self.unsuccessful_allocations

    ''' Returns a list with the task_ids allocated to the robot with id=ropod_id
    '''
    def get_allocations_robot(self, ropod_id):
        allocations = self.__get_allocations()
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
