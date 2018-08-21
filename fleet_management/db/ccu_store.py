import pymongo as pm

'''An interface for saving CCU data into and retrieving them from a database

@author Alex Mitrevski
@contact aleksandar.mitrevski@h-brs.de
'''
class CCUStore(object):
    def __init__(self, db_name):
        self.db_name = db_name

    '''Saves the given task to a database as a new document under the "tasks" collection

    Keyword arguments:
    @param task a reference to a const ccu::Task object representing a task

    '''
    def add_task(self, task):
        pass

    '''Saves the given task to a database as a new document under the "task_archive" collection

    Keyword arguments:
    @param task a previously scheduled task
    @param task_status task status description
    '''
    def archive_task(self, task, task_status):
        pass

    '''Saves the given task id to a database as a new document under the "ongoing_tasks" collection

    Keyword arguments:
    @param task_id UUID representing the id of an already scheduled task
    '''
    def add_ongoing_task(self, task_id):
        pass

    '''Adds a new task status document under the "ongoing_task_status" collection

    Keyword arguments:
    @param task_status task status description

    '''
    def add_task_status(self, task_status):
        pass

    '''Saves an updated status for the given task under the "ongoing_task_status" collection

    Keyword arguments:
    @param task_status task status description
    '''
    def update_task_status(self, task_status):
        pass

    '''Returns a vector of ids representing all tasks that are saved
    under the "ongoing_tasks" collection
    '''
    def get_ongoing_tasks(self):
        pass

    '''Returns a dictionary of task IDs and ccu::Task objects representing
    the scheduled tasks that are saved under the "tasks" collection
    '''
    def get_scheduled_tasks(self):
        pass

    '''Returns a dictionary of task IDs and ccu::TaskStatus objects representing
    the statuses of tasks under the that are saved under the "ongoing_task_status" collection
    '''
    def get_ongoing_task_statuses(self):
        pass

    '''Returns a dictionary of robot IDs and ccu::RobotStatus objects representing
    the statuses of robots saved under the "robot_statuses" collection
    '''
    def get_robot_statuses(self):
        pass

    '''Returns the task with the given id

    Keyword arguments:
    @param task_id UUID representing the id of a task
    '''
    def get_task(self, task_id):
        pass

    '''Returns the status of the task with the given id

    Keyword arguments:
    @param task_id UUID representing the id of a task
    '''
    def get_task_status(self, task_id):
        pass

    '''Returns a ccu::RobotStatus object representing the status of the robot with the given id

    Keyword arguments:
    @param robot_id id of a robot

    '''
    def get_robot_status(self, robot_id):
        pass

    '''Saves new ropods under the ropod collection

    Keyword arguments:
    @param ropod

    '''
    def add_robot(self, robot):
        pass

    def add_elevator(self, elevator):
        pass

    def add_elevator_call(self, request):
        pass
