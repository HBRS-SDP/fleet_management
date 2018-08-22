from fleet_management.structs.area import Area

class RobotStatus(object):
    def __init__(self):
        self.robot_id = ''
        self.current_location = Area()
        self.current_operation = ''
        self.status = ''
        self.available = False
        self.battery_status = -1.

    def to_json(self):
        status_json = dict()
        status_json['robot_id'] = self.robot_id
        status_json['current_location'] = self.current_location.to_json()
        status_json['current_operation'] = self.current_operation
        status_json['status'] = self.status
        status_json['available'] = self.available
        status_json['battery_status'] = self.battery_status
        return status_json

    @staticmethod
    def from_json(status_json):
        status = RobotStatus()
        status.robot_id = status_json['robot_id']
        status.current_location = Area.from_json(status_json['current_location'])
        status.current_operation = status_json['current_operation']
        status.status = status_json['status']
        status.available = status_json['available']
        status.battery_status = status_json['battery_status']
        return status


class TaskStatus(object):
    def __init__(self):
        self.task_id = ''
        self.status = ''
        self.current_robot_action = dict()
        self.completed_robot_actions = dict()
        self.estimated_task_duration = -1.

    def to_json(self):
        task_json = dict()
        task_json['task_id'] = self.task_id
        task_json['status'] = self.status
        task_json['estimated_task_duration'] = self.estimated_task_duration
        task_json['current_robot_actions'] = self.current_robot_action
        task_json['completed_robot_actions'] = self.completed_robot_actions
        return task_json

    @staticmethod
    def from_json(status_json):
        status = TaskStatus()
        status.task_id = status_json['task_id']
        status.status = status_json['status']
        status.estimated_task_duration = status_json['estimated_task_duration']
        status.current_robot_action = status_json['current_robot_actions']
        status.completed_robot_actions = status_json['completed_robot_actions']
        return status
