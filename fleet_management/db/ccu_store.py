import pymongo as pm

from fleet_management.structs.task import Task
from fleet_management.structs.status import RobotStatus, TaskStatus
from fleet_management.structs.elevator import Elevator, ElevatorRequest

'''An interface for saving CCU data into and retrieving them from a database

@author Alex Mitrevski, Argentina Ortega Sainz
@contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
'''
class CCUStore(object):
    def __init__(self, db_name):
        self.db_name = db_name

    '''Saves the given task to a database as a new document under the "tasks" collection

    Keyword arguments:
    @param task a fleet_management.structs.task.Task object

    '''
    def add_task(self, task):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['tasks']
        json_task = task.to_json()
        collection.insert_one(json_task)

    '''Saves the given robot under the "robots" collection

    Keyword arguments:
    @param ropod a fleet_management.structs.robot.Robot object

    '''
    def add_robot(self, robot):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['robots']
        robot_json = robot.to_json(robot)
        collection.insert_one(robot_json)

    '''Saves the given elevator under the "elevators" collection

    Keyword arguments:
    @param elevator a fleet_management.structs.elevator.Elevator object

    '''
    def add_elevator(self, elevator):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['elevators']
        elevator_json = Elevator.to_json(elevator)
        collection.insert_one(elevator_json)

    '''Saves the given elevator request under the "eleabator_calls" collection

    Keyword arguments:
    @param request a fleet_management.structs.elevator.ElevatorRequest object

    '''
    def add_elevator_call(self, request):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['elevator_calls']
        request_json = ElevatorRequest.to_json(request)
        collection.insert_one(request_json)

    '''Saves the given task to a database as a new document under the "task_archive" collection

    Keyword arguments:
    @param task a previously scheduled task
    @param task_status task status description
    '''
    def archive_task(self, task, task_status):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]

        # adding the task to the "task_archive" collection
        json_task = task.to_json()
        # TODO: save the current timestamp
        json_task['task_status'] = task_status.status
        for robot_id in task.robot_actions:
            completed_actions = task_status.completed_robot_actions[robot_id]
            for i in xrange(len(task.robot_actions[robot_id])):
                action = task.robot_actions[robot_id]
                if action.id in completed_actions:
                    task.robot_actions[robot_id]['status'][i] = 'completed'

        archive_collection = db['task_archive']
        archive_collection.insert_one(json_task)

        # removing the task from the "ongoing_tasks" collection
        ongoing_tasks_collection = db['ongoing_tasks']
        ongoing_tasks_collection.delete_one({'task_id': task.id})

        # removing the task from the "ongoing_task_status" collection
        task_status_collection = db['ongoing_task_status']
        task_status_collection.delete_one({'task_id': task.id})

        # removing the task from the "tasks" collection
        scheduled_task_collection = db['tasks']
        scheduled_task_collection.delete_one({'id': task.id})

    '''Saves the given task id to a database as a new document under the "ongoing_tasks" collection

    Keyword arguments:
    @param task_id UUID representing the id of an already scheduled task
    '''
    def add_ongoing_task(self, task_id):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_tasks']
        # TODO: save the current timestamp
        collection.insert_one({'task_id': task_id})

    '''Adds a new task status document under the "ongoing_task_status" collection

    Keyword arguments:
    @param task_status task status description

    '''
    def add_task_status(self, task_status):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        json_task_status = task_status.to_json()
        # TODO: save the current timestamp
        collection.insert_one(json_task_status)

    '''Saves an updated status for the given task under the "ongoing_task_status" collection

    Keyword arguments:
    @param task_status task status description
    '''
    def update_task_status(self, task_status):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        json_task_status = task_status.to_json()
        collection.replace_one({'task_id': task_status.task_id},
                               json_task_status)

    '''Returns a vector of ids representing all tasks that are saved
    under the "ongoing_tasks" collection
    '''
    def get_ongoing_tasks(self):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_tasks']

        task_ids = list()
        for task_json in collection.find():
            task_ids.append(task_json['task_id'])
        return task_ids

    '''Returns a dictionary of task IDs and fleet_management.structs.task.Task objects
    representing the scheduled tasks that are saved under the "tasks" collection
    '''
    def get_scheduled_tasks(self):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['tasks']

        scheduled_tasks = dict()
        for task_json in collection.find():
            task_id = task_json['id']
            scheduled_tasks[task_id] = Task.from_json(task_json)
        return scheduled_tasks

    '''Returns a dictionary of task IDs and fleet_management.structs.status.TaskStatus objects
    representing the statuses of tasks under the that are saved under the "ongoing_task_status" collection
    '''
    def get_ongoing_task_statuses(self):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']

        task_statuses = dict()
        for status_json in collection.find():
            task_id = status_json['task_id']
            task_statuses[task_id] = TaskStatus.from_json(status_json)
        return task_statuses

    '''Returns a dictionary of robot IDs and fleet_management.structs.status.RobotStatus
    objects representing the statuses of robots saved under the "robot_statuses" collection
    '''
    def get_robot_statuses(self):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['robot_statuses']

        robot_statuses = dict()
        for status_json in collection.find():
            robot_id = status_json['robot_id']
            robot_statuses[robot_id] = RobotStatus.from_json(status_json)
        return robot_statuses

    '''Returns a fleet_management.structs.task.Task object
    representing the task with the given id

    Keyword arguments:
    @param task_id UUID representing the id of a task
    '''
    def get_task(self, task_id):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['tasks']
        task_json = collection.find_one({'id': task_id})
        task = Task.from_json(task_json)
        return task

    '''Returns a fleet_management.structs.status.TaskStatus object
    representing the status of the task with the given id

    Keyword arguments:
    @param task_id UUID representing the id of a task
    '''
    def get_task_status(self, task_id):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        status_json = collection.find_one({'task_id': task_id})
        status = TaskStatus.from_json(status_json)
        return status

    '''Returns a fleet_management.structs.status.RobotStatus object
    representing the status of the robot with the given id

    Keyword arguments:
    @param robot_id id of a robot

    '''
    def get_robot_status(self, robot_id):
        db_client = pm.MongoClient()
        db = db_client[self.db_name]
        collection = db['robot_statuses']
        status_json = collection.find_one({'robot_id': robot_id})
        status = RobotStatus.from_json(status_json)
        return status
