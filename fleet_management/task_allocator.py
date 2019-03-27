import time
import logging

from fleet_management.task_allocation import Auctioneer


class TaskAllocator(object):
    def __init__(self, config_params, ccu_store):
        self.logger = logging.getLogger('fms.task.allocation')
        self.ropod_ids = config_params.ropods
        self.method = config_params.allocation_method
        self.zyre_params = config_params.task_allocator_zyre_params
        self.auctioneer = Auctioneer(config_params, ccu_store)

    def get_information(self):
        self.logger.debug(self.auctioneer)

    ''' Allocates a single task or a list of tasks.
        Returns a dictionary
        key - task_id
        value - list of robot_ids assigned to the task_id
        @param task an object of type Task
        or a list of objects of type Task
        '''

    def allocate(self, tasks):
        self.auctioneer.receive_tasks(tasks)
        while True:
            self.auctioneer.announce_task()
            self.auctioneer.check_auction_closure_time()
            time.sleep(0.8)
            if self.auctioneer.done is True:
                break

        # Return allocations of tasks allocated in the current allocation process
        if not isinstance(tasks, list):
            return self.get_allocations([tasks])
        return self.get_allocations(tasks)

    ''' If no argument is given, returns all allocations. 
        If an argument (list of tasks) is given, returns the allocations of the given tasks '''

    def get_allocations(self, tasks=list()):
        allocations = self.auctioneer.get_allocations(tasks)
        if allocations:
            for task_id, robot_ids in allocations.items():
                self.logger.info("Task %s allocated: %s", task_id, [robot_id for robot_id in robot_ids])
        else:
            self.logger.info("No allocations have been made")

        return allocations

    ''' Return a list of tasks that could not be allocated
    '''
    def get_unsuccessful_allocations(self):
        return self.auctioneer.get_unsuccessful_allocations()

    ''' Returns a list with the task_ids allocated to the robot with id=ropod_id
    '''
    def get_allocations_robot(self, ropod_id):
        allocations = self.auctioneer.get_allocations()
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

    ''' Returns a dictionary with the task_ids schedules to all robots
    key - robot_id
    value - list of task_ids
    The first task in the list of task_ids should be the fist one to be executed
    '''
    def get_scheduled_tasks(self):
        return self.auctioneer.get_scheduled_tasks()

    ''' Returns a list with the task_ids scheduled (in the order they will be executed) to the robot with id=ropod_id
    '''
    def get_scheduled_tasks_robot(self, ropod_id):
        scheduled_tasks = self.auctioneer.get_scheduled_tasks()
        scheduled_tasks_robot = list()

        if ropod_id in scheduled_tasks:
            scheduled_tasks_robot = scheduled_tasks[ropod_id]
        else:
            self.logger.info("No tasks scheduled to %s", ropod_id)

        return scheduled_tasks_robot

    ''' Returns a dictionary with the start time and finish time of each allocated task
    keys:
    [ropod_id][task_id]['start_time']
    ropod_id][task_id]['finish_time']
    '''
    def get_tasks_schedule(self):
        return self.auctioneer.get_tasks_schedule()

    ''' Returns a dictionary with the start time and finish time of the tasks assigned to the robot with id=ropod_id
    keys:
    '''
    def get_tasks_schedule_robot(self, task_id, robot_id):
        return self.auctioneer.get_tasks_schedule_robot(task_id, robot_id)

    def shutdown(self):
        self.auctioneer.shutdown()

    def start(self):
        self.auctioneer.start()
