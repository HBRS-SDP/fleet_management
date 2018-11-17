from ropod.pyre_communicator.base_class import PyreBaseCommunicator
import uuid
import time
SLEEP_TIME = 0.350

''' Implements the multi-robot task task_allocation algorighm TeSSI or TeSSIduo depending on the allocation_method specified in the config file.
'''


class Auctioneer(PyreBaseCommunicator):
    def __init__(self, config_params, verbose_mrta=False):
        self.robot_ids = config_params.ropods
        self.method = config_params.allocation_method
        self.zyre_params = config_params.task_allocator_zyre_params
        node_name = 'auctioneer_' + self.method
        super().__init__(node_name, self.zyre_params.groups, self.zyre_params.message_types)

        self.unallocated_tasks = list()
        self.allocations = dict()
        self.unsuccessful_allocations = list()
        self.schedule_robots = dict()
        self.time_schedule_robots = dict()
        self.makespan_robots = dict()

        self.received_bids_round = list()
        self.received_no_bids_round = list()

        # Define function for printing debug information
        self.verbose_mrta = verbose_mrta
        self.verboseprint = print if self.verbose_mrta else lambda *a, **k: None

    def __str__(self):
        auctioneer_info = []
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

    '''
        Triggers the task_allocation process.
    The task_allocation process consists n rounds of auctions, where n is the number of tasks in the list of unallocated_tasks.
    One task is allocated per round until there are no more tasks left to allocate. Robots that have allocated task(s) in previous round(s) still participate in the next round(s).

    In each round:
    - The auctionner announces all unallocated tasks.
    - Each robot calculates a bid for each unallocated task and submits the smallest one (i.e, each robot submits only one bid per round).
    - The auctionner allocates the task to the robot with the smallest bid.
    - The auctioneer removes the allocated task from the list of unallocated_tasks.
    '''
    def announce_tasks(self):
        if self.unallocated_tasks:
            n_tasks = len(self.unallocated_tasks)
            n_round = 0

            while len(self.unallocated_tasks) > 0:
                self.verboseprint("[INFO] Starting round: ", n_round)
                self.verboseprint("[INFO] Number of tasks to allocate: ", len(self.unallocated_tasks))
                # Create task announcement message that contains all
                # unallocated tasks
                task_announcement = dict()
                task_announcement['header'] = dict()
                task_announcement['payload'] = dict()
                task_announcement['header']['type'] = 'TASK-ANNOUNCEMENT'
                task_announcement['header']['metamodel'] = 'ropod-msg-schema.json'
                task_announcement['header']['msgId'] = str(uuid.uuid4())
                task_announcement['header']['timestamp'] = int(round(time.time()) * 1000)

                task_announcement['payload']['metamodel'] = 'ropod-task-announcement-schema.json'
                task_announcement['payload']['round'] = n_round
                task_announcement['payload']['tasks'] = dict()

                for task in self.unallocated_tasks:
                    task_dict = task.to_dict()
                    task_announcement['payload']['tasks'][task.id] = dict()
                    task_announcement['payload']['tasks'][task.id]['id'] = task.id
                    task_announcement['payload']['tasks'][task.id]['pickup_pose'] = task.pickup_pose.to_dict()
                    task_announcement['payload']['tasks'][task.id]['delivery_pose'] = task.delivery_pose.to_dict()
                    task_announcement['payload']['tasks'][task.id]['earliest_start_time'] = task.earliest_start_time
                    task_announcement['payload']['tasks'][task.id]['latest_start_time'] = task.latest_start_time
                    task_announcement['payload']['tasks'][task.id]['estimated_duration'] = task.estimated_duration

                self.verboseprint("[INFO] Auctioneer announces tasks.")
                self.shout(task_announcement, 'TASK-ALLOCATION')

                allocation = self.start_round(n_round)

                # Check if an task_allocation was performed on this round
                if allocation:
                    self.announce_winner(allocation)
                    task_id = allocation[0]
                    robot_id = allocation[1]
                    self.allocations[task_id] = [robot_id]

                    # Remove allocated task from self.unallocated_tasks
                    for i, task in enumerate(self.unallocated_tasks):
                        if task.id == task_id:
                            self.verboseprint("[INFO] Removing task {} from unallocated_tasks".format(task.id))
                            del self.unallocated_tasks[i]
                else:
                    # Tasks could not be allocated
                    for unallocated_task in self.unallocated_tasks:
                        self.unsuccessful_allocations.append(unallocated_task)
                    break

                # Sleep so that the winner robot has time to process
                # the task_allocation
                n_round = n_round + 1
                time.sleep(SLEEP_TIME)
            self.verboseprint("[INFO] There are no more tasks to allocate")
        else:
            print("There are no tasks to allocate")

        return self.allocations

    def start_round(self, n_round):
        # Reinitialize auction_round variables
        self.received_bids_round = list()
        self.received_no_bids_round = list()

        self.verboseprint("[INFO] Starting auction process.")

        while True:
            # Wait until the auctioneer has received a reply from all the robots
            if (len(self.received_bids_round) + len(self.received_no_bids_round)) == len(self.robot_ids):
                self.verboseprint("[INFO] Auctioneer has received a message from all robots")
                break

        allocation = self.elect_winner()

        return allocation

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
            self.received_bids_round.append(bid)
            self.verboseprint("[INFO] Received bid {}".format(bid))
            print(len(self.received_bids_round))
            print(len(self.robot_ids))

        if message_type == 'NO-BID':
            no_bid = dict()
            no_bid['robot_id'] = dict_msg['payload']['robot_id']
            self.received_no_bids_round.append(no_bid)
            self.verboseprint("[INFO] Received NO-BID from", no_bid['robot_id'])

        elif message_type == 'SCHEDULE':
            robot_id = dict_msg['payload']['robot_id']
            schedule = dict_msg['payload']['schedule']
            time_schedule = dict_msg['payload']['time_schedule']
            makespan = dict_msg['payload']['makespan']

            self.schedule_robots[robot_id] = schedule
            self.time_schedule_robots[robot_id] = time_schedule
            self.makespan_robots[robot_id] = makespan

            self.verboseprint("[INFO] Auctioneer received schedule {} of robot {}".format(schedule, robot_id))

    def elect_winner(self):
        if self.received_bids_round:
            self.verboseprint("[INFO] Number of bids received: ", len(self.received_bids_round))
            lowest_bid = float('Inf')
            winning_task = None
            winning_robot = None
            robots_tied = dict()

            for bid in self.received_bids_round:
                if bid['bid'] < lowest_bid:
                    lowest_bid = bid['bid']
                    winning_task = bid['task_id']
                    winning_robot = bid['robot_id']
                    robots_tied[winning_task] = list()
                    robots_tied[winning_task].append(bid['robot_id'])

                elif bid['bid'] == lowest_bid and bid['task_id'] == winning_task:
                    robots_tied[winning_task].append(bid['robot_id'])

            # Resolve ties. Assign the task to the robot with the lowest id
            for task_id, robot_ids in robots_tied.items():
                if len(robot_ids) > 1:
                    self.verboseprint("[INFO] For task {} there is a tie between: {}".format(task_id, [robot_id for robot_id in robot_ids]))

                    robot_ids.sort()
                    winning_robot = robot_ids[0]
                    winning_task = task_id

            self.verboseprint("[INFO] Robot {} wins task {}".format(winning_robot, winning_task))
            allocation = (winning_task, winning_robot)
        else:
            self.verboseprint("[INFO] No bids received. Could not allocate a task in this round")
            allocation = None

        return allocation

    def announce_winner(self, allocation):
        winning_task = allocation[0]
        winning_robot = allocation[1]
        # Create task_allocation message
        allocation = dict()
        allocation['header'] = dict()
        allocation['payload'] = dict()
        allocation['header']['type'] = 'ALLOCATION'
        allocation['header']['metamodel'] = 'ropod-msg-schema.json'
        allocation['header']['msgId'] = str(uuid.uuid4())
        allocation['header']['timestamp'] = int(round(time.time()) * 1000)

        allocation['payload']['metamodel'] = 'ropod-task_allocation-schema.json'
        allocation['payload']['task_id'] = winning_task
        allocation['payload']['winner_id'] = winning_robot

        self.verboseprint("[INFO] Accouncing winner...")
        self.shout(allocation, 'TASK-ALLOCATION')

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

    ''' Returns a dictionary with the schedules of all robots
    key - robot_id
    value - list of task_ids
    '''
    def get_schedule(self):
        return self.schedule_robots

    ''' Returns a dictionary with the start time and finish time of each allocated task.
        tasks_start_finish_time[robot_id][task_id]['start_time']
        tasks_start_finish_time[robot_id][task_id]['finish_time']
    '''
    def get_time_schedule(self):
        tasks_start_finish_time = dict()

        if self.time_schedule_robots:

            for robot_id, time_schedule in self.time_schedule_robots.items():
                # List of tasks scheduled to robot_id
                schedule = self.schedule_robots[robot_id]

                if time_schedule:
                    tasks_start_finish_time[robot_id] = dict()

                    for i in range(0, len(schedule)):
                        task_id = schedule[i]
                        tasks_start_finish_time[robot_id][task_id] = dict()

                        tasks_start_finish_time[robot_id][task_id]['start_time'] = self.time_schedule_robots[robot_id][str(i)]['start_time']

                        tasks_start_finish_time[robot_id][task_id]['finish_time'] = self.time_schedule_robots[robot_id][str(i)]['finish_time']
                else:
                    tasks_start_finish_time[robot_id] = 'empty'

        else:
            print("There are no scheduled tasks")

        return tasks_start_finish_time
