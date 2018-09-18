from fleet_management.structs.area import Area
from fleet_management.structs.action import Action

class RobotTask(object):
    def __init__(self):
        self.earliest_start_time = -1.
        self.latest_start_time = -1.
        self.estimated_end_time = -1.
        self.priority = 0


class TaskRequest(object):
    def __init__(self):
        self.pickup_pose = Area()
        self.delivery_pose = Area()
        self.start_time = -1.
        self.user_id = ''
        self.cart_type = ''
        self.cart_id = ''

    def to_dict(self):
        request_dict = dict()
        request_dict['pickup_pose'] = self.pickup_pose.to_dict()
        request_dict['delivery_pose'] = self.delivery_pose.to_dict()
        request_dict['earliest_start_time'] = self.earliest_start_time
        request_dict['latest_start_time'] = self.latest_start_time
        request_dict['user_id'] = self.user_id
        request_dict['cart_type'] = self.cart_type
        request_dict['cart_id'] = self.cart_id
        request_dict['priority'] = self.priority
        return request_dict


class Task(object):
    def __init__(self):
        self.id = ''
        self.actions = dict()
        self.cart_type = ''
        self.cart_id = ''
        self.team_robot_ids = list()
        self.earliest_start_time = -1.
        self.latest_start_time = -1.
        self.estimated_duration = -1.
        self.priority = 0

    def to_dict(self):
        task_dict = dict()
        task_dict['id'] = self.id
        task_dict['cart_type'] = self.cart_type
        task_dict['cart_id'] = self.cart_id
        task_dict['priority'] = self.priority
        task_dict['earliest_start_time'] = self.earliest_start_time
        task_dict['latest_start_time'] = self.latest_start_time
        task_dict['estimated_duration'] = self.estimated_duration
        task_dict['team_robot_ids'] = self.team_robot_ids
        task_dict['actions'] = dict()
        for robot_id, actions in self.actions.items():
            task_dict['actions'][robot_id] = list()
            for action in actions:
                action_dict = Action.to_dict(action)
                task_dict['actions'][robot_id].append(action_dict)
        return task_dict

    @staticmethod
    def from_dict(task_dict):
        task = Task()
        task.id = task_dict['id']
        task.cart_type = task_dict['cart_type']
        task.cart_id = task_dict['cart_id']
        task.priority = task_dict['priority']
        task.earliest_start_time = task_dict['earliest_start_time']
        task.latest_start_time = task_dict['latest_start_time']
        task.estimated_duration = task_dict['estimated_duration']
        task.team_robot_ids = task_dict['team_robot_ids']
        for robot_id, actions in task_dict['actions'].items():
            task.actions[robot_id] = list()
            for action_dict in actions:
                action = Action.from_dict(action_dict)
                task.actions[robot_id] = action
        return task