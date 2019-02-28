import pymongo as pm
import logging

from ropod.structs.task import Task
from ropod.structs.status import TaskStatus
from ropod.structs.elevator import Elevator, ElevatorRequest
from ropod.structs.robot import Robot
from ropod.structs.area import SubArea, SubAreaReservation
from datetime import timezone, datetime


class CCUStore(object):
    '''An interface for saving CCU data into and retrieving them from a database

    @author Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''

    def __init__(self, db_name, db_port=27017):
        self.db_name = db_name
        self.db_port = db_port
        self.logger = logging.getLogger('fms.db')

    def unique_insert(self, db, collection, dict_to_insert, key, value):
        '''Inserts an element to a given collection but only if it's key doesn't
           already exist
        '''
        found_dict = collection.find_one({key: value})

        if found_dict is None:
            collection.insert(dict_to_insert)
        else:
            self.logger.warning("Element:", dict_to_insert, "already exist. Not adding!")

    def add_task(self, task):
        '''Saves the given task to a database as a new document under the "tasks" collection

        Keyword arguments:
        @param task a ropod.structs.task.Task object

        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['tasks']
        dict_task = task.to_dict()
        self.unique_insert(db, collection, dict_task, 'task_id', dict_task['id'])

    def add_robot(self, robot):
        '''Saves the given robot under the "robots" collection

        Keyword arguments:
        @param robot a ropod.structs.robot.Robot object

        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['robots']
        robot_dict = robot.to_dict()
        self.unique_insert(db, collection, robot_dict, 'robotId', robot_dict['robotId'])

    def get_robot(self, robot_id):
        '''Returns a a ropod.structs.Robot object that has robot_id id
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['robots']

        robot_dict = collection.find_one({'robotId': robot_id})
        robot = Robot.from_dict(robot_dict)

        return robot

    def add_elevator(self, elevator):
        '''Saves the given elevator under the "elevators" collection

        Keyword arguments:
        @param elevator a ropod.structs.elevator.Elevator object

        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['elevators']
        elevator_dict = Elevator.to_dict(elevator)
        self.unique_insert(db, collection, elevator_dict, 'elevatorId', elevator_dict['elevatorId'])

    def add_elevator_call(self, request):
        '''Saves the given elevator request under the "eleabator_calls" collection

        Keyword arguments:
        @param request a ropod.structs.elevator.ElevatorRequest object

        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['elevator_calls']
        request_dict = ElevatorRequest.to_dict(request)
        collection.insert_one(request_dict)

    def archive_task(self, task, task_status):
        '''Saves the given task to a database as a new document under the "task_archive" collection

        Keyword arguments:
        @param task a previously scheduled task
        @param task_status task status description
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]

        # adding the task to the "task_archive" collection
        dict_task = task.to_dict()
        # TODO: save the current timestamp
        dict_task['task_status'] = task_status.status
        for robot_id in task.robot_actions:
            completed_actions = task_status.completed_robot_actions[robot_id]
            for i in range(len(task.robot_actions[robot_id])):
                action = task.robot_actions[robot_id]
                if action.id in completed_actions:
                    task.robot_actions[robot_id]['status'][i] = 'completed'

        archive_collection = db['task_archive']
        archive_collection.insert_one(dict_task)

        # removing the task from the "ongoing_tasks" collection
        ongoing_tasks_collection = db['ongoing_tasks']
        ongoing_tasks_collection.delete_one({'task_id': task.id})

        # removing the task from the "ongoing_task_status" collection
        task_status_collection = db['ongoing_task_status']
        task_status_collection.delete_one({'task_id': task.id})

        # removing the task from the "tasks" collection
        scheduled_task_collection = db['tasks']
        scheduled_task_collection.delete_one({'id': task.id})

    def add_ongoing_task(self, task_id):
        '''Saves the given task id to a database as a new document under the "ongoing_tasks" collection

        Keyword arguments:
        @param task_id UUID representing the id of an already scheduled task
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_tasks']
        # TODO: save the current timestamp
        collection.insert_one({'task_id': task_id})

    def add_task_status(self, task_status):
        '''Adds a new task status document under the "ongoing_task_status" collection

        Keyword arguments:
        @param task_status task status description

        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        dict_task_status = task_status.to_dict()
        # TODO: save the current timestamp
        collection.insert_one(dict_task_status)

    def update_task_status(self, task_status):
        '''Saves an updated status for the given task under the "ongoing_task_status" collection

        Keyword arguments:
        @param task_status task status description
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        dict_task_status = task_status.to_dict()
        collection.replace_one({'task_id': task_status.task_id},
                               dict_task_status)

    def update_elevator(self, elevator):
        '''Saves an updated version of a given elevator under the "elevator" collection

        Keyword arguments:
        @param elevator a ropod.structs.robot.Robot object
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['elevators']
        dict_elevator = elevator.to_dict()
        self.logger.debug("Attempting to update with: %s", dict_elevator)
        collection.replace_one({'elevatorId': elevator.elevator_id},
                               dict_elevator)

    def update_robot(self, robot_status):
        '''Saves an updated status for the given robot under the "robots" collection

        Keyword arguments:
        @param ropod_status a ropod.structs.robot.RobotStatus object
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['robots']

        robot = self.get_robot(robot_status.robot_id)
        robot.status = robot_status

        dict_robot = robot.to_dict()

        collection.replace_one({'robotId': robot_status.robot_id},
                               dict_robot)

    def get_ongoing_tasks(self):
        '''Returns a vector of ids representing all tasks that are saved
        under the "ongoing_tasks" collection
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_tasks']

        task_ids = list()
        for task_dict in collection.find():
            task_ids.append(task_dict['task_id'])
        return task_ids

    def get_scheduled_tasks(self):
        '''Returns a dictionary of task IDs and ropod.structs.task.Task objects
        representing the scheduled tasks that are saved under the "tasks" collection
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['tasks']

        scheduled_tasks = dict()
        for task_dict in collection.find():
            task_id = task_dict['id']
            scheduled_tasks[task_id] = Task.from_dict(task_dict)
        return scheduled_tasks

    def get_ongoing_task_statuses(self):
        '''Returns a dictionary of task IDs and ropod.structs.status.TaskStatus objects
        representing the statuses of tasks under the that are saved under the "ongoing_task_status" collection
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']

        task_statuses = dict()
        for status_dict in collection.find():
            task_id = status_dict['task_id']
            task_statuses[task_id] = TaskStatus.from_dict(status_dict)
        return task_statuses

    def get_elevators(self):
        '''Returns a dictionary of elevator IDs and elevator
           objects representing the current state of the elevators
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['elevators']

        elevators = dict()
        for elevator_dict in collection.find():
            elevator_id = elevator_dict['elevatorId']
            elevators[elevator_id] = Elevator.from_dict(elevator_dict)

        return elevators

    def get_robots(self):
        '''Returns a dictionary of robot IDs and ropod.structs.status.RobotStatus
        objects representing the statuses of robots
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['robots']

        robots = dict()
        for robot_dict in collection.find():
            robot_id = robot_dict['robotId']
            robots[robot_id] = Robot.from_dict(robot_dict)
        return robots

    def get_task(self, task_id):
        '''Returns a ropod.structs.task.Task object
        representing the task with the given id

        Keyword arguments:
        @param task_id UUID representing the id of a task
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['tasks']
        task_dict = collection.find_one({'id': task_id})
        task = Task.from_dict(task_dict)
        return task

    def get_task_status(self, task_id):
        '''Returns a ropod.structs.status.TaskStatus object
        representing the status of the task with the given id

        Keyword arguments:
        @param task_id UUID representing the id of a task
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['ongoing_task_status']
        status_dict = collection.find_one({'task_id': task_id})
        status = TaskStatus.from_dict(status_dict)
        return status

    def add_sub_area(self, sub_area):
        '''Adds sub area to the sub_areas table
        @param sub_area sub area object
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas']
        dict_sub_area = sub_area.to_dict()
        self.unique_insert(db, collection, dict_sub_area, 'id', dict_sub_area['id'])

    def get_sub_area(self, sub_area_id):
        '''Get sub area from sub_areas table using id
        @param sub_area_id sub area id (int)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas']
        sub_area_dict = collection.find_one({'id': sub_area_id})
        return SubArea.from_dict(sub_area_dict)

    def get_sub_areas(self, type):
        '''Get sub areas based on type
        @param type  sub area type (string)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas']
        sub_area_dicts = collection.find({'type': type})
        sub_areas = []
        for sub_area_dict in sub_area_dicts:
            sub_areas.append(SubArea.from_dict(sub_area_dict))
        return sub_areas

    def delete_sub_areas(self):
        '''Deletes all sub areas (used only for unit testing)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas']
        res = collection.delete_many({})
        return res.deleted_count

    def add_sub_area_reservation(self, sub_area_reservation):
        '''Adds new sub area reservation
        @param sub_area_reservation object
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas_reservations']
        dict_sub_area_reservation = sub_area_reservation.to_dict()
        return collection.insert_one(dict_sub_area_reservation).inserted_id

    def get_sub_area_reservation(self, sub_area_reservation_id):
        '''Gets sub area reservation
        @param sub_area_reservation_id (int)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas_reservations']
        sub_area_reservation_dict = collection.find_one({'_id': sub_area_reservation_id})
        return SubAreaReservation.from_dict(sub_area_reservation_dict)

    def delete_sub_area_reservations(self):
        '''Deletes all sub area reservations (used only for unit testing)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas_reservations']
        res = collection.delete_many({})
        return  res.deleted_count

    def get_all_future_reservations(self, sub_area_id):
        '''Gets all future reservations for given sub area
        @param sub_area_id (int)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas_reservations']
        future_reservation_dict_list = collection.find({'subAreaId': sub_area_id, 'startTime': {'$gte':
          datetime.now(timezone.utc).isoformat()}})
        future_reservations = []
        for future_reservation_dict in future_reservation_dict_list:
            future_reservations.append(SubAreaReservation.from_dict(future_reservation_dict))
        return future_reservations

    def update_sub_area_reservation(self, sub_area_reservation_id, status):
        '''Updates of existing sub area reservation
        @param sub_area_reservation_id (int)
        @param status "unknown or scheduled or cancelled" (string)
        '''
        db_client = pm.MongoClient(port=self.db_port)
        db = db_client[self.db_name]
        collection = db['sub_areas_reservations']

        sub_area_reservation = self.get_sub_area_reservation(sub_area_reservation_id)
        sub_area_reservation.status = status
        dict_sub_area_reservation = sub_area_reservation.to_dict()
        return collection.replace_one({'_id': sub_area_reservation_id}, dict_sub_area_reservation)
