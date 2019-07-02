import argparse
import time
import logging
import datetime
import copy
import numpy as np
import os.path

from ropod.pyre_communicator.base_class import RopodPyre

from ropod.utils.uuid import generate_uuid
from ropod.utils.timestamp import TimeStamp as ts
from ropod.utils.logging.config import config_logger

from ropod.structs.task import Task
from ropod.structs.area import Area
from fleet_management.config.loader import Config

SLEEP_TIME = 0.250

""" Implements the multi-robot task task_allocation algorithm TeSSI or TeSSIduo depending on
the allocation_method passed to the constructor.

TeSSI uses the makespan for the bid calulation (it does not include travel distance in the bidding rule)

TesSSIduo uses a dual objective heuristic bidding rule, which combines makespan and distance traveled.
"""


class Robot(RopodPyre):
    MISMATCHED_SCHEDULES = 1
    UNSUCCESSFUL_ALLOCATION = 2

    def __init__(self, robot_id, allocation_method, api_config, ccu_store, path_planner, auctioneer):

        self.logger = logging.getLogger('fms.resources.robot.%s' % robot_id)

        self.id = robot_id
        self.method = allocation_method
        self.ccu_store = ccu_store
        self.path_planner = path_planner
        self.auctioneer = auctioneer

        zyre_config = api_config.get('zyre')  # Arguments for the zyre_base class
        zyre_config['node_name'] = robot_id + '_proxy'
        super().__init__(zyre_config)

        # Read initial position from the mongodb database
        self.position = Area()
        robot = self.ccu_store.get_robot(robot_id)
        self.logger.debug("Robot: %s", robot)
        robot_status = robot.status
        self.logger.debug("Robot status: %s", robot_status)
        self.position = robot_status.current_location
        self.logger.debug("Position: %s Floor %s", self.position.name, self.position.floor_number)

        # TODO: self.position should reflect the current position of the robot.
        # A Zyre node should be updating the robot position.

        # Read schedule from the mongodb. List of Tasks(obj) scheduled to be performed in the future.
        self.scheduled_tasks = self.ccu_store.get_robot_schedule(self.id)
        self.logger.debug("Robot %s schedule %s", self.id, self.scheduled_tasks)

        # Simple Temporal network of the tasks in self.scheduled_tasks
        self.stn = list()

        # Tasks received by the robot in a round of an auction process
        self.received_tasks_round = list()
        # Numeric bid placed on a round of an auction process
        self.bid_round = 0.
        # Scheduled tasks used for calculating the bid on a round of
        # an auction process
        self.bid_scheduled_tasks_round = list()
        # STN used for calculating the bid on a round of an auction process
        self.bid_stn_round = list()

        # Time to complete the tasks in self.scheduled_tasks
        self.completion_time = 0.
        # Cost to travel to all scheduled tasks
        self.travel_cost = 0.
        # Weighting factor used for the dual bidding rule
        self.alpha = 0.5

        # Travel cost for visiting the tasks in self.bid_scheduled_tasks_round
        self.travel_cost_round = 0.
        # Time to complete the tasks in self.bid_scheduled_tasks_round
        self.completion_time_round = 0.

    def __str__(self):
        robot_info = list()
        robot_info.append("Robot id = " + self.id)
        robot_info.append("Position: " + str(self.position))
        robot_info.append("Zyre groups")
        robot_info.append("{}".format(self.groups()))
        return '\n '.join(robot_info)

    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            return
        message_type = dict_msg['header']['type']

        if message_type == 'TASK-ANNOUNCEMENT':
            self.reinitialize_auction_variables()
            n_round = dict_msg['payload']['round']
            tasks = dict_msg['payload']['tasks']
            self.logger.debug("Robot %s received a TASK-ANNOUNCEMENT for tasks %s", self.id, tasks)

            scheduled_tasks = self.ccu_store.get_robot_schedule(self.id)

            schedule1 = [task.to_dict() for task in scheduled_tasks]
            schedule2 = [task.to_dict() for task in self.scheduled_tasks]

            pairs_tasks = zip(schedule1, schedule2)

            if len(schedule1) != len(schedule2) or any(x != y for x, y in pairs_tasks):
                self.logger.warning("Schedule in the ccu_store %s does not match local schedule %s", [task.id for task in scheduled_tasks], [task.id for task in self.scheduled_tasks])

                for d1, d2 in zip(schedule1, schedule2):
                    for key, value in d1.items():
                        if value != d2[key]:
                            self.logger.warning("Different keys!", key, value, d2[key])

                cause_empty_bid = self.MISMATCHED_SCHEDULES
                self.send_empty_bid(n_round, cause_empty_bid)
            else:
                self.logger.debug("Schedules %s , %s matched. Calculating bids", [task.id for task in scheduled_tasks], [task.id for task in self.scheduled_tasks])
                self.expand_schedule(tasks, n_round)

        if message_type == "ALLOCATION":
            allocation = dict()
            allocation['task_id'] = dict_msg['payload']['task_id']
            allocation['winner_id'] = dict_msg['payload']['winner_id']
            if allocation['winner_id'] == self.id:
                self.allocate_to_robot(allocation['task_id'])

        if message_type == "REQUEST-TIMESLOT":
            task_dict = dict_msg['payload']['task']
            task = Task.from_dict(task_dict)
            self.logger.debug("Robot %s received a request for alternative timeslot for task %s", self.id, task.id)
            self.propose_alternative_timeslot(task)

        elif message_type == "RESET-SCHEDULE":
            self.logger.debug("Robot %s reset its schedule", self.id)
            scheduled_tasks = self.ccu_store.get_robot_schedule(self.id)
            self.logger.debug("Scheduled tasks %s", scheduled_tasks)
            self.scheduled_tasks = list()

    def reinitialize_auction_variables(self):
        self.received_tasks_round = list()
        self.bid_round = None
        self.bid_scheduled_tasks_round = list()
        self.bid_stn_round = list()
        if self.method == 'tessiduo':
            self.travel_cost_round = 0.
            self.completion_time_round = 0.

    ''' Builds a schedule for each task received in the TASK-ANNOUNCEMENT
    '''
    def expand_schedule(self, tasks, n_round):
        bids = dict()

        for task_id, task_info in tasks.items():
            task = Task.from_dict(task_info)

            self.logger.debug("Inserting task %s with earliest_start_time %s and latest start time %s", task.id, task.earliest_start_time, task.latest_start_time)
            best_bid, best_schedule, best_stn = self.insert_task(task)

            if best_bid != np.inf:
                bids[task_id] = dict()
                bids[task_id]['bid'] = best_bid
                bids[task_id]['scheduled_tasks'] = best_schedule
                bids[task_id]['stn'] = best_stn
            else:
                self.logger.info("Robot % does not bid for {}".format(self.id, task.id))
                cause_empty_bid = self.UNSUCCESSFUL_ALLOCATION
                self.logger.info("STN becomes inconsistent when adding task %s", self.id)

        if bids:
            self.get_smallest_bid(bids, n_round)
        else:
            self.send_empty_bid(n_round, cause_empty_bid, task_id)

    '''
        Get the smallest bid among all bids.
        Each robot submits only its smallest bid in each round
        If two or more tasks have the same bid, the robot bids for the task with the lowest task_id
    '''
    def get_smallest_bid(self, bids, n_round):
        smallest_bid = np.inf
        task_bid = None
        scheduled_tasks = list()
        stn = list()
        lowest_task_id = ''

        for task_id, bid_info in bids.items():
            if bid_info['bid'] < smallest_bid:
                smallest_bid = bid_info['bid']
                task_bid = task_id
                scheduled_tasks = bid_info['scheduled_tasks']
                stn = bid_info['stn']
                lowest_task_id = task_id

            elif bid_info['bid'] == smallest_bid and task_id < lowest_task_id:
                task_bid = task_id
                scheduled_tasks = bid_info['scheduled_tasks']
                stn = bid_info['stn']
                lowest_task_id = task_id

        if smallest_bid != np.inf:
            tasks = [task.id for task in scheduled_tasks]

            self.logger.debug("Round %s: Robot_id %s bids %s for task %s and schedule %s",
                              n_round, self.id, smallest_bid, task_bid, tasks)

            self.send_bid(n_round, task_bid, smallest_bid, scheduled_tasks, stn)
        else:
            self.logger.info("Robot %s does not place a bid in round %s", self.id, n_round)

    '''
    Computes the travel cost (distance traveled) for performing all tasks in the schedule (list of tasks)
    '''
    def compute_travel_cost(self, schedule):
        previous_position = self.position
        distance = 0

        for task in schedule:
            distance += self.path_planner.get_estimated_path_distance(previous_position.floor_number,
                                                                     task.pickup_pose.floor_number,
                                                                     previous_position.name,
                                                                      task.pickup_pose.name)
            previous_position = task.delivery_pose

        self.logger.debug("Travel Distance for performing tasks in the schedule : %s", distance)
        return distance

    def compute_bid(self, stn):
        if self.method == 'tessiduo':
            completion_time = self.compute_completion_time(stn)
            travel_cost = self.compute_travel_cost(self.scheduled_tasks)
            bid = (self.alpha * completion_time) + (1 - self.alpha) * (travel_cost - self.travel_cost)

        else:
            bid = self.compute_completion_time(stn)

        return bid

    ''' Inserts a task in each possible position in the list of scheduled
    tasks.
    '''
    def insert_task(self, task):
        best_bid = np.inf
        best_schedule = list()
        best_stn = list()

        if self.scheduled_tasks:
            n_scheduled_tasks = len(self.scheduled_tasks)

            for i in range(0, n_scheduled_tasks + 1):
                self.scheduled_tasks.insert(i, task)
                stn = self.build_stn(self.scheduled_tasks)
                if stn:
                    minimal_stn = self.floyd_warshall(stn)
                    bid = self.compute_bid(minimal_stn)
                    if bid < best_bid:
                        best_bid = bid
                        best_schedule = copy.deepcopy(self.scheduled_tasks)
                        best_stn = copy.deepcopy(minimal_stn)

                # Restore new_schedule for the next iteration
                self.scheduled_tasks.pop(i)

        else:
            stn = self.initialize_stn(task)
            if stn:
                best_stn = self.floyd_warshall(stn)
                if self.is_consistent(best_stn):
                    best_bid = self.compute_bid(best_stn)
                    best_schedule = [task]

        return best_bid, best_schedule, best_stn

    """ Calculates the time for going from the robot's position to the pickup location of the first task in the schedule.
    The earliest_start_time at which the first task will be executed is whatever value is bigger (travel time to
    the task or earliest_start_time). The STN can only be build if such value is smaller than the latest_start_time.
    """
    def travel_constraint_first_task(self, task):
        # Get estimated time to go from the initial position of the robot to the pickup_pose of the first task
        distance = self.path_planner.get_estimated_path_distance(self.position.floor_number, task.pickup_pose.floor_number,
                                                                 self.position.name, task.pickup_pose.name)

        # Assuming 1m/s constant velocity
        estimated_time = distance
        travel_time = estimated_time + (datetime.datetime.now()).timestamp()
        earliest_start_time = max(travel_time, task.earliest_start_time)

        if earliest_start_time < task.latest_start_time:
            return earliest_start_time
        else:
            return False

    """ Creates a STN (Simple Temporal Network) with one task
    A task consists of:
        Time points:
            - Start Time
            - End Time
        Constraints:
            - Earliest Start Time (ESt)
            - Latest Start Time (LSt)
            - Earliest Finish time (EFt = ESt + Dur)
            - Latest Finish time (LFt = LSt + Dur)
            - Duration (Dur)

    Constraint between tasks:
    - Travel time from previous task to next task

    Each time point is represented as a vertex in the STN.
    Each constraint is represented as an edge in the STN.
    The origin time point is time 0 (it counts as a vertex in the STN)

    Number of tasks in the schedule: n_tasks
    Number of constraints between tasks: n_tasks

    Number of vertices in the STN: 2*n_tasks + 1
    Number of edges in the STN: 5*n_tasks + n_tasks-1

    Each edge is represented as a list
    [start_vertex, end_vertex, weight]
    All edges are stored in a list called stn (Simple Temporal Network)
    """
    def initialize_stn(self, task):
        edges = list()
        stn = list()
        # Check if the robot can make it to the pickup location of task
        earliest_start_time = self.travel_constraint_first_task(task)

        if earliest_start_time:
            n_vertices = 3

            e_s_t = [1, 0, -earliest_start_time]
            l_s_t = [0, 1, task.latest_start_time]
            l_f_t = [0, 2, task.latest_finish_time]
            e_f_t = [2, 0, -task.earliest_finish_time]
            duration = [2, 1, -task.estimated_duration]
            # Add the constraints as edges in the STN
            edges.append(e_s_t)
            edges.append(l_s_t)
            edges.append(l_f_t)
            edges.append(e_f_t)
            edges.append(duration)

            # Initialize the stn with entries equal to inf
            for i in range(0, n_vertices):
                stn.append([])
                for j in range(0, n_vertices):
                    stn[i].append(np.inf)

            # Store edges with their weights
            for i in range(0, len(edges)):
                for info in edges:
                    start_point = info[0]
                    end_point = info[1]
                    weight = info[2]
                    stn[start_point][end_point] = weight

            # Set paths of each vertex to itself to zero
            for i in range(0, n_vertices):
                stn[i][i] = 0

        else:
            self.logger.debug("Robot %s cannot build a STN for %s", self.id, task.id)

        return stn

    """ Builds a STN for the tasks in new_schedule
    Each edge is represented as a list
    [start_vertex, end_vertex, weight]
    All edges are stored in a list
    """
    def build_stn(self, new_scheduled_tasks):
        edges = list()
        stn = list()
        # Check if the robot can make it to the pickup location of task
        earliest_start_time = self.travel_constraint_first_task(new_scheduled_tasks[0])

        if earliest_start_time:
            for position, task in enumerate(new_scheduled_tasks):
                start = position * 2 + 1
                end = start + 1
                l_s_t = [0, start, task.latest_start_time]
                l_f_t = [0, end, task.latest_finish_time]
                e_f_t = [end, 0, -task.earliest_finish_time]
                duration = [end, start, -task.estimated_duration]
                edges.append(l_s_t)
                edges.append(l_f_t)
                edges.append(e_f_t)
                edges.append(duration)

                if position == 0:
                    e_s_t = [start, 0, -earliest_start_time]
                else:
                    e_s_t = [start, 0, -task.earliest_start_time]
                    travel_time = self.get_travel_time(new_scheduled_tasks[position - 1], task)
                    t_t = [start, start - 1, -travel_time]
                    edges.append(t_t)

                edges.append(e_s_t)

            # Initialize the stn with entries equal to inf
            n_tasks = len(new_scheduled_tasks)
            n_vertices = n_tasks * 2 + 1

            for i in range(0, n_vertices):
                stn.append([])
                for j in range(0, n_vertices):
                    stn[i].append(np.inf)

            # Store edges with their weights
            for i in range(0, len(edges)):
                for info in edges:
                    start_point = info[0]
                    end_point = info[1]
                    weight = info[2]
                    stn[start_point][end_point] = weight

            # Set paths of each vertex to itself to zero
            for i in range(0, n_vertices):
                stn[i][i] = 0

        return stn

    """ Returns the estimated time the robot will need to travel
    from the delivery pose of the previous task to the pickup pose
    of the next task
    """
    def get_travel_time(self, previous_task, next_task):
        distance = self.path_planner.get_estimated_path_distance(previous_task.delivery_pose.floor_number,
                                                                  next_task.pickup_pose.floor_number,
                                                                  previous_task.delivery_pose.name,
                                                                  next_task.pickup_pose.name)
        # Assuming a constant velocity of 1m/s
        estimated_time = distance
        return estimated_time

    ''' Computes the smallest distances between each pair of vertices in the stn.
    '''
    def floyd_warshall(self, stn):
        n_vertices = len(stn)
        for k in range(0, n_vertices):
            for i in range(0, n_vertices):
                for j in range(0, n_vertices):
                    if stn[i][j] > stn[i][k] + stn[k][j]:
                        stn[i][j] = stn[i][k] + stn[k][j]
        return stn

    ''' The stn is consistent if it does not contain negative cycles
    '''
    def is_consistent(self, distances):
        consistent = True
        n_vertices = len(distances)
        # check for negative cycles
        for i in range(0, n_vertices):
            if distances[i][i] != 0:
                consistent = False

        return consistent

    '''
        Returns the completion time (time between the lowest interval of the first scheduled task and the lowest interval of the last scheduled task)
    '''
    def compute_completion_time(self, minimal_stn):
        # Earliest start time of the first task in the schedule
        start_time = - minimal_stn[1][0]  # Row 1, Column 0
        # Earliest finish time of the last task in the schedule
        finish_time = - minimal_stn[-1][0]  # Last row Column 0
        completion_time = round(finish_time - start_time, 2)
        return completion_time

    """
    Create bid_msg and send it to the auctioneer
    """
    def send_bid(self, n_round, task_id, bid, scheduled_tasks, stn):
        bid_msg = dict()
        bid_msg['header'] = dict()
        bid_msg['payload'] = dict()
        bid_msg['header']['type'] = 'BID'
        bid_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        bid_msg['header']['msgId'] = generate_uuid()
        bid_msg['header']['timestamp'] = ts.get_time_stamp()

        bid_msg['payload']['metamodel'] = 'ropod-bid-schema.json'
        bid_msg['payload']['robot_id'] = self.id
        bid_msg['payload']['n_round'] = n_round
        bid_msg['payload']['task_id'] = task_id
        bid_msg['payload']['bid'] = bid

        # Numeric bid placed on this round and schedule of tasks and stn used
        # for calculating the bid
        if self.method == 'tessiduo':
            self.travel_cost_round = self.compute_travel_cost(scheduled_tasks)
        self.bid_round = bid
        self.bid_scheduled_tasks_round = scheduled_tasks
        self.bid_stn_round = stn
        self.whisper(bid_msg, peer=self.auctioneer)
        self.logger.info("Sent bid for task %s", task_id)

    """
    Create empty_bid_msg and send it to the auctioneer
    """
    def send_empty_bid(self, n_round, cause, task_id=0):
        empty_bid_msg = dict()
        empty_bid_msg['header'] = dict()
        empty_bid_msg['payload'] = dict()
        empty_bid_msg['header']['type'] = 'NO-BID'
        empty_bid_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        empty_bid_msg['header']['msgId'] = generate_uuid()
        empty_bid_msg['header']['timestamp'] = ts.get_time_stamp()

        empty_bid_msg['payload']['metamodel'] = 'ropod-bid-schema.json'
        empty_bid_msg['payload']['robot_id'] = self.id
        empty_bid_msg['payload']['n_round'] = n_round
        empty_bid_msg['payload']['task_id'] = task_id
        empty_bid_msg['payload']['cause'] = cause

        self.logger.debug("Robot %s sends empty bid for task %s. Cause: %s", self.id, task_id, cause)
        self.whisper(empty_bid_msg, peer=self.auctioneer)

    def allocate_to_robot(self, task_id):
        # Update the schedule and stn with the values bid only if the robot placed a bid in the current round
        if self.bid_round:
            self.scheduled_tasks = copy.deepcopy(self.bid_scheduled_tasks_round)
            self.stn = copy.deepcopy(self.bid_stn_round)

            self.logger.info("Robot %s allocated task %s", self.id, task_id)

            tasks = [task.id for task in self.scheduled_tasks]
            self.logger.info("Tasks scheduled to robot %s:%s", self.id, tasks)

            if self.method == 'tessiduo':
                self.travel_cost = self.travel_cost_round
                self.logger.debug("Robot %s current travel cost %s", self.id, self.travel_cost)

            self.send_schedule()
        else:
            self.logger.info("Robot %s cannot allocate task %s because it did not place a bid for it", self.id, task_id)

    """ Sends the updated schedule of the robot to the auctioneer.
    """
    def send_schedule(self):
        schedule_msg = dict()
        schedule_msg['header'] = dict()
        schedule_msg['payload'] = dict()
        schedule_msg['header']['type'] = 'SCHEDULE'
        schedule_msg['header']['metamodel'] = 'ropod-msg-schema.json'
        schedule_msg['header']['msgId'] = generate_uuid()
        schedule_msg['header']['timestamp'] = ts.get_time_stamp()

        schedule_msg['payload']['metamodel'] = 'ropod-msg-schema.json'
        schedule_msg['payload']['robot_id'] = self.id
        schedule_msg['payload']['robot_schedule'] = list()

        for i, task in enumerate(self.scheduled_tasks):
            schedule_msg['payload']['robot_schedule'].append(task.to_dict())

        timetable = self.get_timetable()
        schedule_msg['payload']['timetable'] = timetable

        self.logger.debug("Robot sends its updated schedule to the auctioneer.")

        self.whisper(schedule_msg, peer=self.auctioneer)

    ''' Suggest to start a task at the end of the robot's schedule
    '''
    def propose_alternative_timeslot(self, task):
        finish_time_last_task = - self.stn[-1][0]  # Last row Column 0
        last_task = self.scheduled_tasks[-1]

        # The travel time to go from the delivery pose of the last task of the schedule to the
        # pickup pose of task is equivalent to the distance
        travel_time = self.path_planner.get_estimated_path_distance(last_task.delivery_pose.floor_number,
                                                                 task.pickup_pose.floor_number,
                                                                 last_task.delivery_pose.name, task.pickup_pose.name)
        suggested_start_time = finish_time_last_task + travel_time

        task_alternative_timeslot = dict()
        task_alternative_timeslot['header'] = dict()
        task_alternative_timeslot['payload'] = dict()
        task_alternative_timeslot['header']['type'] = 'TASK-ALTERNATIVE-TIMESLOT'
        task_alternative_timeslot['header']['metamodel'] = 'ropod-msg-schema.json'
        task_alternative_timeslot['header']['msgId'] = generate_uuid()
        task_alternative_timeslot['header']['timestamp'] = ts.get_time_stamp()
        task_alternative_timeslot['payload']['metamodel'] = 'ropod-task_alternative_timeslot-schema.json'
        task_alternative_timeslot['payload']['robot_id'] = self.id
        task_alternative_timeslot['payload']['task_id'] = task.id
        task_alternative_timeslot['payload']['start_time'] = suggested_start_time

        self.whisper(task_alternative_timeslot, peer=self.auctioneer)

    """ Returns a dictionary with the start and finish times of all tasks in the STN
        timetable[task_id]['start_time']
        timetable[task_id]['finish_time']
    """
    def get_timetable(self):
        distances = self.floyd_warshall(self.stn)
        n_vertices = len(distances)
        timetable = dict()

        first_column = list()
        # Get the first column in the stn (list of lists)
        for i in range(0, n_vertices):
            first_column.append(distances[i][0])

        # Remove first element of the list
        first_column.pop(0)

        e_s_times = [first_column[i] for i in range(0, len(first_column)) if int(i) % 2 == 0]
        e_f_times = [first_column[i] for i in range(0, len(first_column)) if int(i) % 2 != 0]

        for i in range(0, len(e_s_times)):
            timetable[self.scheduled_tasks[i].id] = dict()
            timetable[self.scheduled_tasks[i].id]['start_time'] = - e_s_times[i]
            timetable[self.scheduled_tasks[i].id]['finish_time'] = -e_f_times[i]

        return timetable


if __name__ == '__main__':
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)

    config = Config('../../config/fms_config-v2.yaml', False)
    ccu_store = config.configure_ccu_store()
    path_planner = config.configure_path_planner()

    parser = argparse.ArgumentParser()
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()
    ropod_id = args.robot_id

    robot_config = config.configure_robot_proxy(ropod_id, ccu_store, path_planner)

    log_config_file = os.path.join(main_dir, '../config/logging.yaml')
    config_logger(log_config_file, ropod_id)

    time.sleep(5)

    robot = Robot(**robot_config)
    robot.start()

    try:
        while True:
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        logging.info("Terminating %s proxy ...", ropod_id)
        robot.shutdown()
        logging.info("Exiting...")
