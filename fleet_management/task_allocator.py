from __future__ import print_function
from fleet_management.task_allocation import Auctioneer
import time


class TaskAllocator(object):
    def __init__(self, config_params):
        self.ropod_ids = config_params.ropods
        self.method = config_params.allocation_method
        self.zyre_params = config_params.task_allocator_zyre_params
        self.auctioneer = Auctioneer(config_params, verbose_mrta=True)

    def get_information(self):
        print(self.auctioneer)

    ''' Allocates a single task or a list of tasks.
        Returns a dictionary
        key - task_id
        value - list of robot_ids assigned to the task_id
        @param task an object of type Task
        or a list of objects of type Task
        '''

    def get_assignment(self, tasks):
        self.auctioneer.receive_tasks(tasks)
        while True:
            self.auctioneer.announce_task()
            time.sleep(0.8)
            if self.auctioneer.done is True:
                break
        return self.get_allocations()

    def get_allocations(self):
        allocations = self.auctioneer.get_allocations()
        if allocations:
            for task_id, robot_ids in allocations.items():
                print("Task {} : {}".format(task_id, [robot_id for robot_id in robot_ids]))
        else:
            print("No allocations have been made")

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
                    print("There are no tasks allocated to ", ropod_id)

        else:
            print("There are no allocated tasks")

        return allocations_robot

    ''' Returns a dictionary with the schedules of all robots
    key - robot_id
    value - list of task_ids
    The first task in the list of task_ids will be the first task to be executed
    '''
    def get_schedule(self):
        return self.auctioneer.get_schedule()

    ''' Returns a list with the task_ids scheduled (in the order they will be executed) to the robot with id=ropod_id
    '''
    def get_schedule_robot(self, ropod_id):
        scheduled_tasks = self.auctioneer.get_schedule()
        schedule_robot = list()

        if ropod_id in scheduled_tasks:
            schedule_robot = scheduled_tasks[ropod_id]
        else:
            print("No tasks scheduled to ", ropod_id)

        return schedule_robot

    ''' Returns a dictionary with the start time and finish time of each allocated task
    keys:
    [ropod_id][task_id]['start_time']
    ropod_id][task_id]['finish_time']
    '''
    def get_time_schedule(self):
        return self.auctioneer.get_time_schedule()

    def shutdown(self):
        self.auctioneer.shutdown()
