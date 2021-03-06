import logging
from datetime import timezone, datetime

import pymongo as pm
from pymongo.errors import ServerSelectionTimeoutError
from ropod.structs.area import SubArea, SubAreaReservation
from ropod.structs.status import TaskStatus
from mrs.structs.timetable import Timetable


class CCUStore(object):
    """An interface for saving CCU data into and retrieving them from a database

    @author Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    """

    def __init__(self, db_name='ccu_store', port=27017, **_):
        self.logger = logging.getLogger('fms.db')
        self.db_name = db_name
        self.db_port = port

        try:
            # Default timeout is 30s
            self.client = pm.MongoClient(port=self.db_port)
            self.logger.debug(self.client.server_info())
        except ServerSelectionTimeoutError as err:
            self.logger.critical("Cannot connect to MongoDB", exc_info=True)
            return

        self.db = self.client[self.db_name]
        self.logger.info("Connected to %s on port %s", self.db_name, self.db_port)

    def __str__(self):
        return str(self.__dict__)

    def unique_insert(self, collection, dict_to_insert, key, value):
        """Inserts an element to a given collection but only if it's key doesn't
           already exist.
        """
        found_dict = collection.find_one({key: value})

        if found_dict is None:
            collection.insert(dict_to_insert)
        else:
            self.logger.warning("Element: %s already exist. Not adding!", dict_to_insert)

    def archive_task(self, task, task_status):
        """Saves the given task to a database as a new document under the "task_archive" collection.

        Keyword arguments:
        @param task a previously scheduled task
        @param task_status task status description
        """

        # adding the task to the "task_archive" collection
        dict_task = task.to_dict()
        # TODO: save the current timestamp
        dict_task['task_status'] = task_status.status
        for robot_id in task.robot_actions:
            completed_actions = task_status.completed_robot_actions[robot_id]
            for i in range(len(task.robot_actions[robot_id])):
                actions = task.robot_actions[robot_id]
                for action in actions:
                    if action.id in completed_actions:
                        task.robot_actions[robot_id]['status'][i] = 'completed'

        archive_collection = self.db['task_archive']
        archive_collection.insert_one(dict_task)

        # removing the task from the "ongoing_tasks" collection
        ongoing_tasks_collection = self.db['ongoing_tasks']
        ongoing_tasks_collection.delete_one({'task_id': task.id})

        # removing the task from the "ongoing_task_status" collection
        task_status_collection = self.db['ongoing_task_status']
        task_status_collection.delete_one({'task_id': task.id})

        # removing the task from the "tasks" collection
        scheduled_task_collection = self.db['tasks']
        scheduled_task_collection.delete_one({'id': task.id})

    def add_sub_area(self, sub_area):
        """Adds sub area to the sub_areas table.

        Keyword arguments:
          @param sub_area sub area object
        """
        collection = self.db['sub_areas']
        dict_sub_area = sub_area.to_dict()
        self.unique_insert(collection, dict_sub_area, 'id', dict_sub_area['id'])

    def get_sub_area(self, sub_area_id):
        """Get sub area from sub_areas table using id.

        Keyword arguments:
        @param sub_area_id sub area id (int)
        """
        collection = self.db['sub_areas']
        sub_area_dict = collection.find_one({'id': sub_area_id})
        return SubArea.from_dict(sub_area_dict)

    def get_sub_areas(self, type):
        """Get sub areas based on type.

        Keyword arguments:
        @param type  sub area type (string)
        """
        collection = self.db['sub_areas']
        sub_area_dicts = collection.find({'type': type})
        sub_areas = []
        for sub_area_dict in sub_area_dicts:
            sub_areas.append(SubArea.from_dict(sub_area_dict))
        return sub_areas

    def delete_sub_areas(self):
        """Deletes all sub areas (used only for unit testing).
        """
        collection = self.db['sub_areas']
        res = collection.delete_many({})
        return res.deleted_count

    def add_sub_area_reservation(self, sub_area_reservation):
        """Adds new sub area reservation.

        Keyword arguments:
        @param sub_area_reservation object
        """
        collection = self.db['sub_areas_reservations']
        dict_sub_area_reservation = sub_area_reservation.to_dict()
        return collection.insert_one(dict_sub_area_reservation).inserted_id

    def get_sub_area_reservation(self, sub_area_reservation_id):
        """Gets sub area reservations.

        Keyword arguments:
        @param sub_area_reservation_id (int)
        """
        collection = self.db['sub_areas_reservations']
        sub_area_reservation_dict = collection.find_one({'_id': sub_area_reservation_id})
        return SubAreaReservation.from_dict(sub_area_reservation_dict)

    def delete_sub_area_reservations(self):
        """Deletes all sub area reservations (used only for unit testing).
        """
        collection = self.db['sub_areas_reservations']
        res = collection.delete_many({})
        return  res.deleted_count

    def get_all_future_reservations(self, sub_area_id):
        """Gets all future reservations for given sub area.

        Keyword arguments:
        @param sub_area_id (int)
        """
        collection = self.db['sub_areas_reservations']
        # TODO This doesn't match the Unix time timestamp used in the rest of the code base
        future_reservation_dict_list = collection.find({'subAreaId': sub_area_id,
                                                        'startTime': {'$gte': datetime.now(timezone.utc).isoformat()}})
        future_reservations = []
        for future_reservation_dict in future_reservation_dict_list:
            future_reservations.append(SubAreaReservation.from_dict(future_reservation_dict))
        return future_reservations

    def update_sub_area_reservation(self, sub_area_reservation_id, status):
        """Updates of existing sub area reservation.

        Keyword arguments:
        @param sub_area_reservation_id (int)
        @param status "unknown or scheduled or cancelled" (string)
        """
        collection = self.db['sub_areas_reservations']

        sub_area_reservation = self.get_sub_area_reservation(sub_area_reservation_id)
        sub_area_reservation.status = status
        dict_sub_area_reservation = sub_area_reservation.to_dict()
        return collection.replace_one({'_id': sub_area_reservation_id}, dict_sub_area_reservation)

    def clean(self):
        self.client.drop_database(self.db_name)
