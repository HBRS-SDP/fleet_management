from fleet_management.structs.area import Area
from fleet_management.structs.action import Action

class RobotTask(object):
    def __init__(self):
        self.start_time = -1.
        self.estimated_end_time = -1.


class TaskRequest(object):
    def __init__(self):
        self.pickup_pose = Area()
        self.delivery_pose = Area()
        self.start_time = -1.
        self.user_id = ''
        self.cart_type = ''
        self.cart_id = ''

    def to_json(self):
        request_json = dict()
        request_json['pickup_pose'] = self.pickup_pose.to_json()
        request_json['delivery_pose'] = self.delivery_pose.to_json()
        request_json['start_time'] = self.start_time
        request_json['user_id'] = self.user_id
        request_json['cart_type'] = self.cart_type
        request_json['cart_id'] = self.cart_id
        return request_json


class Task(object):
    def __init__(self):
        self.id = ''
        self.robot_actions = dict()
        self.cart_type = ''
        self.cart_id = ''
        self.team_robot_ids = list()
        self.start_time = -1.
        self.estimated_duration = -1.

    def to_json(self):
        task_json = dict()
        task_json['id'] = self.id
        task_json['cart_type'] = self.cart_type
        task_json['cart_id'] = self.cart_id
        task_json['start_time'] = self.start_time
        task_json['estimated_duration'] = self.estimated_duration
        task_json['team_robot_ids'] = self.team_robot_ids
        task_json['robot_actions'] = dict()
        for robot_id, actions in self.robot_actions.items():
            task_json['robot_actions'][robot_id] = list()
            for action in actions:
                action_json = Action.to_json(action)
                task_json['robot_actions'][robot_id].append(action_json)
        return task_json

    @staticmethod
    def from_json(task_json):
        task = Task()
        task.id = task_json['id']
        task.cart_type = task_json['cart_type']
        task.cart_id = task_json['cart_id']
        task.start_time = task_json['start_time']
        task.estimated_duration = task_json['estimated_duration']
        task.team_robot_ids = task_json['team_robot_ids']
        for robot_id, actions in task_json['robot_actions'].items():
            task.robot_actions[robot_id] = list()
            for action_json in actions:
                action = Action.from_json(action_json)
                task.robot_actions[robot_id] = action
        return task
