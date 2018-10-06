from fleet_management.structs.area import Area

class RobotStatus(object):
    def __init__(self):
        self.robot_id = ''
        self.current_location = Area()
        self.current_operation = ''
        self.status = ''
        self.available = False
        self.battery_status = -1.

    def to_dict(self):
        status_dict = dict()
        status_dict['robot_id'] = self.robot_id
        status_dict['current_operation'] = self.current_operation
        status_dict['current_location'] = self.current_location.to_dict()
        status_dict['status'] = self.status
        status_dict['available'] = self.available
        status_dict['battery_status'] = self.battery_status

        # TODO may wish to make the to/from_dict methods more verbose
        # or possibly just remove the below and reset to before
        #if isinstance(self.current_location, Area):
        #    status_dict['current_location'] = self.current_location.to_dict()
        #else:
        #    status_dict['current_location'] = self.current_location

        return status_dict

    @staticmethod
    def from_dict(status_dict):
        status = RobotStatus()
        status.robot_id = status_dict['robot_id']
        status.current_operation = status_dict['current_operation']
        status.current_location = Area.from_dict(status_dict['current_location'])
        status.status = status_dict['status']
        status.available = status_dict['available']
        status.battery_status = status_dict['battery_status']

        # TODO may wish to make the to/from_dict methods more verbose
        # or possibly just remove the below and reset to before
        #if isinstance(status.current_location, dict):
        #    status.current_location = Area.from_dict(status_dict['current_location'])
        #else:
        #    new_area = Area()
        #    Area.id = 'INVALID'
        #    Area.name = status_dict['current_location']
        #    status.current_location = new_area

        return status


class TaskStatus(object):
    def __init__(self):
        self.task_id = ''
        self.status = ''
        self.current_robot_action = dict()
        self.completed_robot_actions = dict()
        self.estimated_task_duration = -1.

    def to_dict(self):
        task_dict = dict()
        task_dict['task_id'] = self.task_id
        task_dict['status'] = self.status
        task_dict['estimated_task_duration'] = self.estimated_task_duration
        task_dict['current_robot_actions'] = self.current_robot_action
        task_dict['completed_robot_actions'] = self.completed_robot_actions
        return task_dict

    @staticmethod
    def from_dict(status_dict):
        status = TaskStatus()
        status.task_id = status_dict['task_id']
        status.status = status_dict['status']
        status.estimated_task_duration = status_dict['estimated_task_duration']
        status.current_robot_action = status_dict['current_robot_actions']
        status.completed_robot_actions = status_dict['completed_robot_actions']
        return status
